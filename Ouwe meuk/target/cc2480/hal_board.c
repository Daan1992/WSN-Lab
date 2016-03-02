/**************************************************************************************************
    Filename:       hal_board.c
    Revised:        $Date: 2008-03-28 15:13:08 -0700 (Fri, 28 Mar 2008) $
    Revision:       $Revision: 16675 $

    Description: Board specific implementations for the Z-Accel DB using the MSP430F2274.

    Copyright 2006-2007 Texas Instruments Incorporated. All rights reserved.

    IMPORTANT: Your use of this Software is limited to those specific rights
    granted under the terms of a software license agreement between the user
    who downloaded the software, his/her employer (which must be your employer)
    and Texas Instruments Incorporated (the "License").  You may not use this
    Software unless you agree to abide by the terms of the License. The License
    limits your use, and you acknowledge, that the Software may not be modified,
    copied or distributed unless embedded on a Texas Instruments microcontroller
    or used solely and exclusively in conjunction with a Texas Instruments radio
    frequency transceiver, which is integrated into your product.  Other than for
    the foregoing purpose, you may not use, reproduce, copy, prepare derivative
    works of, modify, distribute, perform, display or sell this Software and/or
    its documentation for any purpose.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

    Should you have any questions regarding your right to use this Software,
    contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_board.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

// Count for ~20msec - ADC needs 17msec for refV to settle.
#define ADC_WAIT_MSECS         20

// Debounce for ~10msec.
#define DEBOUNCE_MSECS         10

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define _PRAGMA(x) _Pragma(#x)
#define HAL_ISR_FUNC_DECLARATION(f,v) _PRAGMA(vector=v) __interrupt void f(void)
#define HAL_ISR_FUNC_PROTOTYPE(f)     __interrupt void f(void)
#define HAL_ISR_FUNCTION(f,v)         HAL_ISR_FUNC_PROTOTYPE(f); HAL_ISR_FUNC_DECLARATION(f,v)

/* Whereas a customer application may require one or more slaves on the SPI bus (which could be
 * but are not necessarily other ZACCEL modules).
 * Whereas it may be necessary to interrupt any message exchange on the SPI bus with any of the
 * slaves in order to exchange a message with a different (obviously higher priority) slave, and
 * then return to finish the original message with the original slave.
 * Therefore it is necessary that setting and clearing any particular slave select line be
 * physically separate from setting and clearing any particular protocol handshake.
 *
 * Whereas the RPC Protocol handshake requires that MRDY be set low and maintained low throughout
 * the possibly several, logically contiguous, exchanges of RPC commands on the SPI bus.
 * Whereas the cc2480 has the MPC430 host wired to only one ZACCEL slave on the SPI bus.
 * Therefore it was deemed simpler and easier to understand on a logical analyzer trace of the SPI
 * and RPC signals for the setting and clearing of the MRDY control line to be tied to the setting
 * and clearing of the slave select line.
 *
 * It may better serve the user to remove the control of the slave select line from the control of
 * the MRDY line and place it here, where it belongs, in the 4-wire SPI protocol control.
 *
 * Also to note: the cc2480 SPI slave does not tri-state the MOSI line when its SS is not asserted,
 * thus requiring external de-coupling hardware in order to support two or more slaves on the bus.
 */
#define SS_Clr()
#define SS_Set()

#define RST_Clr()       (P3OUT |= BV(7))
#define RST_Set()       (P3OUT &= ~BV(7))

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

// Flags set by ISR's to indicate events without blocking or invoking background functions.
volatile uint16 halEventFlags = HAL_EVT_NONE;

// Last value read from the ADC by ISR.
volatile uint16 halAdcVal = 0;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

// Software timers w/ +/- 1 sec resolution and 65535 msec max.
static volatile uint16 tmrTicks[HAL_TIMER_CNT];
static uint16 tmrPeriod[HAL_TIMER_CNT];
uint16 TACCR0_INIT;

#ifdef HOST_MT
static uint8 rx0Buf[ZACCEL_BUF_LEN];
static volatile uint8 rx0Head;
static volatile uint8 rx0Time;
static uint8 rx0Tail;
static uint8 tx0Buf[ZACCEL_BUF_LEN];
static uint8 tx0Head;
static volatile uint8 tx0Tail;
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void ioInit(void);
static void halMcuInit(void);
static void halTimerInit(void);
static void halSPIInit(void);
void halSPIRead(uint8 sss, uint8 *pBuf, uint8 len);
void halSPIWrite(uint8 sss, uint8 *pBuf, uint8 len);

#define INTERRUPT_PORT1()    HAL_ISR_FUNCTION(isrPort1, PORT1_VECTOR)
#define INTERRUPT_PORT2()    HAL_ISR_FUNCTION(isrPort2, PORT2_VECTOR)
#define INTERRUPT_TIMERA()   HAL_ISR_FUNCTION(isrTimerA, TIMERA0_VECTOR)
#define INTERRUPT_ADC10()    HAL_ISR_FUNCTION(isrADC, ADC10_VECTOR)

#ifdef HOST_MT
static void halUARTInit(void);
void halUARTWrite(uint8 port, uint8 *pBuf, uint8 len);
#define INTERRUPT_RX0()      HAL_ISR_FUNCTION(isrRx0, USCIAB0RX_VECTOR)
#define INTERRUPT_TX0()      HAL_ISR_FUNCTION(isrTx0, USCIAB0TX_VECTOR)
#endif

/**************************************************************************************************
 * @fn          halBoardInit
 *
 * @brief       This is the HAL board main initialization function for ZASA.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halBoardInit(void)
{
  ioInit();
  halMcuInit();
  halTimerInit();
  halSPIInit();
#ifdef HOST_MT
  halUARTInit();
#endif
  halEventFlags = HAL_EVT_NONE;   // Clear event flags
}

/**************************************************************************************************
 * @fn          halSlaveReset
 *
 * @brief       This function makes a clean reset of the slave Z-Accel device.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halSlaveReset(void)
{
  RST_Set();
  halDelay(10, TRUE);
  RST_Clr();
  halDelay(250, TRUE);
  while (!SRDY());  // Wait for the Z-Accel slave to signal that it is out of reset.
}

/**************************************************************************************************
 * @fn          ioInit
 *
 * @brief       This function initializes the DIO.
 *              MSP430x2xx: 6.2.7 Configuring Unused Port Pins
 *              Unused I/O pins should be configured as I/O function, output direction,
 *              and left unconnected on the PC board, to prevent a floating input and reduce power
 *              consumption. The value of the PxOUT bit is a don’t care.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void ioInit(void)
{
  /* P1.0/1 are DO to control LED_1/2, active high.
   * P1.2 is a DI w/ int. pull-up high to receive switch press interrupt.
   */
  P1SEL = 0x00;
  P1OUT = 0xFC;
  P1DIR = 0xFB;

  /* Configure the push button for ISR on press down.
   */
  P1REN = SW1_BV;
  P1IES = SW1_BV;
  P1IFG = 0;
  P1IE  = SW1_BV;

  /* P2.0 is the light sensor input to ADC10-0.
   * P2.1 is the light sensor input enable - set high to read ADC, then low to save power.
   */
  P2SEL = 0x01;
  P2OUT = 0xDD;
  P2DIR = 0xBA; //0xBE; //leave P2.2 as input (0)
  P2REN |= BV(2); //enable pull-up on P2.2
  
  /* P2.6 is a DI to receive SRDY (Slave Ready on RPC protocol.)
   * Configure the SRDY for ISR on active low so that slave does not timeout awaiting the RPC
   * handshake sequence and reset.
   */
  P2REN |= BV(6);
  HAL_CFG_SRDY_ISR;
  P2IFG = 0;
  P2IE  = BV(6);

  /* P2.3 is a DI (M.W.)
   * P2.3 pull up resistor enabled
   */
  P2DIR &= ~BV(3);
  P2REN |= BV(3);
  
  /* P3.0 is a DO to control SSn (Slave Select Not = active low.)
   * P3.6 is a DO to control MRDY (Master Ready on RPC protocol.)
   * P3.7 is a DO to control RSTn (Slave RESET Not = active low asserted.)
   */
  P3SEL = 0x00;
  P3OUT = 0x7F;
  P3DIR = 0xFF;

  /* P4.0 is a DO to control Cfg0 (Slave Configuration Switch 0.)
   * - Set high configures slave to use 32-kHz crystal installed and used.
   * - Set low configures slave to use internal oscillator.
   * P4.1 is a DO to control Cfg1 (Slave Configuration Switch 1.)
   * - Set high configures slave to use SPI transport.
   * - Set low configures slave to use UART transport.
   *
   * P4.3.is a DO to control a door motor (M.W.)
   * - Set high to close the door
   * - Set low to open
   */
  P4SEL = 0x00;
  P4OUT = 0xFF;
  P4DIR = 0xFF;
}

/**************************************************************************************************
 * @fn          halMcuInit
 *
 * @brief       Turn off watchdog and set up system clock. Set system clock to 8 MHz if Vdd permits.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void halMcuInit(void)
{
  volatile uint16 tmp;

  WDTCTL = WDTPW | WDTHOLD;           // Disable the watchdog.

  // Setup code taken from TI example code: msp430x22x4_adc11_temp.c.
  ADC10CTL1 = INCH_11 + ADC10DIV_3;   // Internal bus voltage, ADC10CLK/4
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE;

  /* Allow Vdd to settle before reading it and re-configuring the system clock.
   * MSP FAE confirmed that it can take up to 1 second for the Vdd to settle.
   */
  __delay_cycles(250);

  ADC10CTL0 |= ENC + ADC10SC;         // Sampling and conversion start.
  __low_power_mode_1();               // Low power during sampling period.

  // Measurement of AVcc/2 is in range 0 to 1023 indicating voltage from 0 to 1.5V.
  tmp = (halAdcVal * 30 + 512) >> 10; // Convert to units of 0.1V

  // Cannont use 8-MHz if the Calibration values have been erased or if the bus voltage < 2.2V.
  if ((tmp >= 22) && (CALBC1_8MHZ != 0xFF) && (CALDCO_8MHZ != 0xFF))
  {
    // Configure the system clock for 8-MHz by loading the calibrated values from the info memory.
    BCSCTL1 = CALBC1_8MHZ;
    DCOCTL = CALDCO_8MHZ;
  }
  else
  {
    halEventFlags |= HAL_EVT_NO_8MHz;
  }
}

/**************************************************************************************************
 * @fn          halTimerInit
 *
 * @brief       Turn off watchdog and set up system clock. Set system clock to 8 MHz.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void halTimerInit(void)
{
  uint8 loop = 128;         // Count 8mHz by 65536 timer overflow.

  if (halEventFlags & HAL_EVT_NO_8MHz)
  {
    loop = 16;              // Count 1mHz by 65536 timer overflow.
  }

  BCSCTL1 |= DIVA_3;        // ACLK/8
  BCSCTL3 |= LFXT1S_2;      // ACLK = VLO

  // Setup TimerB to calibrate TimerA.
  TBCCR0 = 65535;

  // Setup TimerA to be calibrated.
  TACCR0 = 65535;
  TACTL = TASSEL_1 + MC_1;  // ACLK, up count mode.

  // Calibrate TimerA.
  TBCTL = TBSSEL_2 + MC_1;  // SMCLK, up count mode.

  do {
    while ((TBCTL & TBIFG) == 0);
    TBCTL &= ~TBIFG;
  } while (--loop);
  TBCTL = TBCLR;            // Clear TimerB configuration.

  TACTL &= ~MC_1;           // Stop TimerA.
  TACCR0_INIT = TAR;
  TACTL |= MC_1;            // Resume TimerA.

  // Configure TimerA for 1-msec ISR for driving the HAL S/W timers, timing key press+hold, etc.
  TACCR0 = TACCR0_INIT;     // 1 second by 12kHz / 8 -> 1.5kHz
  TACCTL0 = CCIE;           // TACCR0 interrupt enabled
}

/**************************************************************************************************
 * @fn          halSPIInit
 *
 * @brief       This function initializes the SPI.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void halSPIInit(void)
{
  /* Set SWRST - UART logic held in reset state during configuration. */
  UCB0CTL1 = UCSWRST;

  /* Configure for Mastermode; 3-pin SPI mode; 8 bits per byte. */
  UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC;

  // Configure baud for ~2.7 MB (8-MHz/3), or max without an 8-MHz system clock.
  UCB0CTL1 |= UCSSEL_2;
  if (halEventFlags & HAL_EVT_NO_8MHz)
  {
    UCB0BR0 = 0x01;
  }
  else
  {
    UCB0BR0 = 0x03;
  }
  UCB0BR1 = 0x00;

  P3SEL |= 0x0E;   // Select SPI functionality on DIO pins for MOSI, MISO, & CLK.

  /* Clear SWRST - release reset to operation */
  UCB0CTL1 &= ~UCSWRST;
}

/**************************************************************************************************
 * @fn          halSPIWrite
 *
 * @brief       This function blocks (polling) on the SPI port transaction. Note that the SPI
 *              master (this host application) must write in order to read, so this one write
 *              function has such a dual use in the code.
 *
 * input parameters
 *
 * @param       sss - SPI Slave to select
 * @param       pBuf - Pointer to the buffer that contains the data to transmit.
 * @param       len - Length of the data to transmit.
 *
 * output parameters
 *
 * @param       pBuf - Pointer to the buffer that gets filled with the Rx bytes.
 *
 * @return      None.
 **************************************************************************************************
 */
void halSPIWrite(uint8 sss, uint8 *pBuf, uint8 len)
{
  if (sss == HAL_SPI_ZACCEL)
  {
    SS_Set();

    while (len--)
    {
      UCB0TXBUF = *pBuf;
      while (!(IFG2 & UCB0RXIFG));
      *pBuf++ = UCB0RXBUF;
    }

    SS_Clr();
  }
}

/**************************************************************************************************
 * @fn          halTimerSet
 *
 * @brief       This function sets the timer period and control flags as specified.
 *
 * input parameters
 *
 * @param       tIdx - HAL Timer index.
 * @param       period - Timer period in msecs (zero to turn off.)
 * @param       ctlFlags - Bit mask of timer control flags.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halTimerSet(uint8 tIdx, uint16 period, uint8 ctlFlags)
{
  halIntState_t s;

  if (ctlFlags & HAL_TIMER_AUTO)
  {
    tmrPeriod[tIdx] = period;
  }
  else
  {
    tmrPeriod[tIdx] = 0;
  }

  HAL_ENTER_CRITICAL_SECTION(s);
  tmrTicks[tIdx] = period;
  halEventFlags &= ~BV(tIdx);
  HAL_EXIT_CRITICAL_SECTION(s);
}

/**************************************************************************************************
 * @fn          halDelay
 *
 * @brief       Delay for milliseconds.
 *              Do not invoke with zero.
 *              Do not invoke with greater than 500 msecs.
 *              Invoking with very high frequency and/or with long delays will start to
 *              significantly impact the real time performance of TimerA tasks because this will
 *              invisibly overrun the period when the TimerA count remaining, when this function
 *              is invoked, is less than the delay requested.
 *
 * input parameters
 *
 * @param       msecs - Milliseconds to delay in low power mode.
 * @param       sleep - Enforces blocking delay in low power mode if set.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halDelay(uint8 msecs, uint8 sleep)
{
  // Scaling for msecs depends on result of TimerA calibration.
  // stop = msecs * TACCR0_INIT / 1000
  // Division by 10 to prevent overflow:
  //
  uint16 stop = msecs * (TACCR0_INIT / 10) / (1000 / 10);

  HAL_DISABLE_INTERRUPTS();
  TACTL &= ~MC_1;         // Stop the timer.

  if (TAR != TACCR0_INIT)
  {
    stop += TAR;
    if (stop == TACCR0_INIT)
    {
      stop++;
    }
  }
  TACCR0 = stop;

  TACTL |= MC_1;          // Re-start the timer.
  HAL_ENABLE_INTERRUPTS();

  if (sleep)
  {
    do {
      HAL_LOW_POWER_MODE();
    } while (TACCR0 != TACCR0_INIT);
  }
}

/**************************************************************************************************
 * @fn          halDelayDone
 *
 * @brief       Check to determine if a requested HAL delay is done.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
uint8 halDelayDone(void)
{
  return (TACCR0 == TACCR0_INIT);
}

#ifndef COORDINATOR
/**************************************************************************************************
 * @fn          halReadTemp
 *
 * @brief       This function starts the ADC and sets up to read the channel for temperature.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halReadTemp(void)
{
  // Setup code taken from eZ430-RF2500 Temperature Sensor End Device.
  ADC10CTL1 = INCH_10 + ADC10DIV_4;       // Temp Sensor ADC10CLK/5
  ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE + ADC10SR;

  halDelay(ADC_WAIT_MSECS, TRUE);         // Allow Vref to ADC to charge.
  ADC10CTL0 |= ENC | ADC10SC;             // Sampling and conversion start
}

/*******************************************************************************
* @fn halLightSensorRead
*
* @brief Starts reading the value from the light sensor on a ZAccel node.
* The value is ranging from 0x00 to 0x0400
*
* input parameters: None.
* output parameters: None.
* @return None.
*******************************************************************************
*/
void halLightSensorRead(void)
{
 // Code partly taken from CC2480 Software Examples code (swru169.zip):
 P2OUT |= BIT1; // Turn on the light sensor
 ADC10AE0 |= 0x01; // Sample channel A0
 ADC10CTL1 = INCH_0 + ADC10DIV_3; // A0, ADC10CLK/4
 ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE; // REF2_5V;
 halDelay (ADC_WAIT_MSECS, TRUE); // Allow Vref the ADC to charge.
 ADC10CTL0 |= (ENC + ADC10SC); // Sampling and conversion start
}

/**************************************************************************************************
 * @fn          halReadBusV
 *
 * @brief       This function sets up to read the channel for bus voltage.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halReadBusV(void)
{
  // Setup code taken from TI example code: msp430x22x4_adc10_temp.c.
  ADC10CTL1 = INCH_11 + ADC10DIV_3;       // Internal bus voltage, ADC10CLK/4
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE;

  halDelay(ADC_WAIT_MSECS, TRUE);         // Allow Vref to ADC to charge.
  ADC10CTL0 |= ENC | ADC10SC;             // Sampling and conversion start
}
#endif

/**************************************************************************************************
 * @fn          isrPort1
 *
 * @brief       This function services the Push Button interrupt.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
INTERRUPT_PORT1()
{
  P1IE  = 0;
  halDelay(DEBOUNCE_MSECS, TRUE);
  P1IFG = 0;
  P1IE  = SW1_BV;
  halEventFlags |= HAL_EVT_BTN_PRESS;  // Signal that a key press+hold event has taken place.

  __low_power_mode_off_on_exit();
}

/**************************************************************************************************
 * @fn          isrPort2
 *
 * @brief       This function services the SRDY interrupt.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
INTERRUPT_PORT2()
{
  P2IFG = 0;
  /* A sleeping host just needs to wake on SRDY. No flag needs to be set since Application-level
   * logic automatically polls for an asynchronous message from the slave.
   */
  __low_power_mode_off_on_exit();
}

/**************************************************************************************************
 * @fn          isrTimerA
 *
 * @brief       This function services the Timer-A interrupt.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
INTERRUPT_TIMERA()
{
  uint8 idx;

  if (TACCR0 != TACCR0_INIT)
  {
    __low_power_mode_off_on_exit();

    if (TACCR0 < TACCR0_INIT)
    {
      TACCR0 = TACCR0_INIT;
      return;
    }
    TAR = 0;
    TACCR0 = TACCR0_INIT;
  }

#ifdef HOST_MT
  if (rx0Time != 0)
  {
    // If an Rx byte is waiting.
    if (rx0Time >= HOST_MT_RX_OLD)
    {
      halEventFlags |= HAL_EVT_MT_RX_RDY;
      rx0Time = 0;
    }
    else
    {
      rx0Time++;
    }
  }
#endif

  // Update all active S/W timers.
  for (idx = 0; idx < HAL_TIMER_CNT; idx++)
  {
    if ((tmrTicks[idx] != 0) && (--tmrTicks[idx] == 0))
    {
      tmrTicks[idx] = tmrPeriod[idx];
      halEventFlags |= BV(idx);
    }
  }

  if (halEventFlags != HAL_EVT_NONE)
  {
    __low_power_mode_off_on_exit();
  }
}

/**************************************************************************************************
 * @fn          isrADC
 *
 * @brief       This function services the ADC interrupt.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
INTERRUPT_ADC10()
{
  halAdcVal = ADC10MEM;             // Get sample from ADC
  halEventFlags |= HAL_EVT_ADC;     // Signal that the ADC sample is ready.

  ADC10CTL0 &= ~ENC;                // Disable the ADC
  ADC10CTL0 &= ~(REFON + ADC10ON);  // Turn off A/D to save power

  __low_power_mode_off_on_exit();
  P2OUT &= ~BIT1;                   // Turn off the light sensor
}

#ifdef HOST_MT
/**************************************************************************************************
 * @fn          halUARTInit
 *
 * @brief       This function initializes the UART.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void halUARTInit(void)
{
  uint16 ubr;

  if (halEventFlags & HAL_EVT_NO_8MHz)
  {
    ubr = (1000000UL + HOST_MT_BAUD/2) / (uint32)HOST_MT_BAUD;
  }
  else
  {
    ubr = (8000000UL + HOST_MT_BAUD/2) / (uint32)HOST_MT_BAUD;
  }

  UCA0CTL1 = UCSWRST;     // Set SWRST - UART logic held in reset state.

  UCA0CTL0 = 0;           // Set Frame Format: no parity; one stop-bit; 8 bits per byte.

  /* Set source clock & baud rate. */
  UCA0CTL1 |= UCSSEL1;
  UCA0BR1 = ubr >> 8;
  UCA0BR0 = ubr;

  P3SEL |= 0x30;  /* P3.4, 5 - UART0 TXD, RXD */

  UCA0CTL1 &= ~UCSWRST;  // Clear SWRST - release reset to operation.
  IE2 |= UCA0RXIE;       // Enable Rx ISR.
}

/**************************************************************************************************
 * @fn          halUARTRead
 *
 * @brief       This function transfers the bytes read by ISR to the parameter buffer.
 *
 * input parameters
 *
 * @param       port - HAL_PORT_NUM to read.
 * @param       pBuf - Pointer to the buffer to copy up to len bytes of Rx.
 * @param       len - Max number of bytes to copy into pBuf.
 *
 * output parameters
 *
 * @param       pBuf - Pointer to the buffer that gets filled with the Rx bytes.
 *
 * @return      Number of bytes actually copied into pBuf.
 **************************************************************************************************
 */
uint8 halUARTRead(uint8 port, uint8 *pBuf, uint8 len)
{
  uint8 cnt = 0;

  if (port == HAL_PORT_MT)
  {
    while (len--)
    {
      if (rx0Tail == rx0Head)
      {
        break;
      }

      *pBuf++ = rx0Buf[rx0Tail];
      if (rx0Tail == 0)
      {
        rx0Tail = ZACCEL_BUF_LEN-1;
      }
      else
      {
        rx0Tail--;
      }
      cnt++;
    }
  }

  return cnt;
}

/**************************************************************************************************
 * @fn          halUARTWrite
 *
 * @brief       This function sets up the UART Tx buffer to transmit by ISR.
 *
 * input parameters
 *
 * @param       port - HAL_PORT_NUM to write.
 * @param       pBuf - Pointer to the buffer that contains the data to transmit.
 * @param       len - Length of the data to transmit.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halUARTWrite(uint8 port, uint8 *pBuf, uint8 len)
{
  if (port == HAL_PORT_MT)
  {
    while (len--)
    {
      tx0Buf[tx0Head] = *pBuf++;
      if (tx0Head == 0)
      {
        tx0Head = ZACCEL_BUF_LEN-1;
      }
      else
      {
        tx0Head--;
      }
    }
    IE2 |= UCA0TXIE;  // Enable UART Tx ISR.
  }
}

/**************************************************************************************************
 * @fn          UART ISRs
 *
 * @brief       These ISRs handle Rx/Tx on UART0.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
INTERRUPT_RX0()
{
  uint8 tmp = rx0Head;

  rx0Buf[tmp] = UCA0RXBUF;
  if (tmp == 0)
  {
    tmp = ZACCEL_BUF_LEN-1;
  }
  else
  {
    tmp--;
  }

  rx0Head = tmp;
  tmp = rx0Tail - tmp;
#if ZACCEL_BUF_LEN != 256
  if (tmp > ZACCEL_BUF_LEN)
  {
    tmp += ZACCEL_BUF_LEN;
  }
#endif
  if (tmp >= HOST_MT_RX_FULL-1)
  {
    halEventFlags |= HAL_EVT_MT_RX_RDY;
  }
  else
  {
    rx0Time = 1;
  }
}

INTERRUPT_TX0()
{
  if (tx0Tail != tx0Head)
  {
    UCA0TXBUF = tx0Buf[tx0Tail];
    if (tx0Tail == 0)
    {
      tx0Tail = ZACCEL_BUF_LEN-1;
    }
    else
    {
      tx0Tail--;
    }
  }
  else
  {
    IE2 &= ~UCA0TXIE;   // Disable interrupt.
  }
}
#endif
