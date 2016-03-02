/**************************************************************************************************
    Filename:       hal_board.h
    Revised:        $Date: 2008-03-28 15:13:08 -0700 (Fri, 28 Mar 2008) $
    Revision:       $Revision: 16675 $

    Description: Board specific definitions for the Z-Accel DB using the MSP430F2274.

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
#ifndef HAL_BOARD_H
#define HAL_BOARD_H

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */

#include <msp430.h>
#include "hal_types.h"
#include "hal_defs.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define SW1_BV  BV(2)

/* Changing this constant requires the corresponding changes to both the HAL_TIMER_X bit flags
 * in halEventFlag_t and  the HAL_TIMER_Y index values in halTimerIdx_t.
 */
#define HAL_TIMER_CNT       4     // 8

// Flag bits for halTimerSet() arg.
#define HAL_TIMER_AUTO     0x01   // Auto-reset timer period on every expiration (i.e. real-time.)

// Flag bits for halTimerGet() arg.
#define HAL_TIMER_INTERVAL 0x01   // The timer period (zero if not auto-resetting.)
#define HAL_TIMER_TICKS    0x02   // Ticks of msecs left on the timer.

// Changing the position of the 1st timer flag requires a change to the timer update logic.
#define HAL_EVT_NONE       0x0000
#define HAL_EVT_TIMER0     0x0001
#define HAL_EVT_TIMER1     0x0002
#define HAL_EVT_TIMER2     0x0004
#define HAL_EVT_TIMER3     0x0008
#define HAL_EVT_TIMER4     0x0010
#define HAL_EVT_TIMER5     0x0020
#define HAL_EVT_TIMER6     0x0040
#define HAL_EVT_TIMER7     0x0080
#define HAL_EVT_BTN_PRESS  0x0100
#define HAL_EVT_ADC        0x0200
#define HAL_EVT_MT_RX_RDY  0x0400
#define HAL_EVT_SPB        0x0800
#define HAL_EVT_SPC        0x1000
#define HAL_EVT_SPD        0x2000
#define HAL_EVT_SPE        0x4000
#define HAL_EVT_NO_8MHz    0x8000

#define HAL_EVT_TIMER_LED  HAL_EVT_TIMER0
#define HAL_EVT_TIMER_APP  HAL_EVT_TIMER1
#define HAL_EVT_TIMER_BTN  HAL_EVT_TIMER2
// Timers 3-7 are available.

// List of SPI slave devices.
#define HAL_SPI_ZACCEL     0

// List of UART ports.
#ifdef HOST_MT
#define HAL_PORT_MT        0
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef istate_t halIntState_t;

typedef enum {
  HAL_IDX_TIMER_LED,
  HAL_IDX_TIMER_APP,
  HAL_IDX_TIMER_BTN,
  HAL_IDX_TIMER3,
  HAL_IDX_TIMER4,
  HAL_IDX_TIMER5,
  HAL_IDX_TIMER6,
  HAL_IDX_TIMER7
} halTimerIdx_t;

/* ------------------------------------------------------------------------------------------------
 *                                            Macros
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_LOW_POWER_MODE()            __low_power_mode_3()

#define HAL_ENABLE_INTERRUPTS()         asm("eint")
#define HAL_DISABLE_INTERRUPTS()        st( asm("dint"); asm("nop"); )
#define HAL_INTERRUPTS_ARE_ENABLED()    (__get_SR_register() & GIE)

#define HAL_ENTER_CRITICAL_SECTION(x) \
  st( x = __get_interrupt_state();  HAL_DISABLE_INTERRUPTS(); )
#define HAL_EXIT_CRITICAL_SECTION(x) __set_interrupt_state((x))
#define HAL_CRITICAL_STATEMENT(x) \
  st( halIntState_t s; HAL_ENTER_CRITICAL_SECTION(s); x; HAL_EXIT_CRITICAL_SECTION(s); )

#define HAL_BTN_PRESSED         ((P1IN & SW1_BV) == 0)

#define HAL_TURN_OFF_RED()       (P1OUT &= ~BV(0))
#define HAL_TURN_OFF_GRN()       (P1OUT &= ~BV(1))

#define HAL_TURN_ON_RED()        (P1OUT |= BV(0))
#define HAL_TURN_ON_GRN()        (P1OUT |= BV(1))

#define HAL_TOGGLE_RED()         (P1OUT ^= BV(0))
#define HAL_TOGGLE_GRN()         (P1OUT ^= BV(1))

#define HAL_STATE_RED()          (P1OUT & BV(0))
#define HAL_STATE_GRN()          (P1OUT & BV(1))

#define SRDY()                  ((P2IN & BV(6)) == 0)
#define _MRDY_Clr()              (P3OUT |= BV(6))
#define _MRDY_Set()              (P3OUT &= ~BV(6))
#define _SS_Clr()                (P3OUT |= BV(0))
#define _SS_Set()                (P3OUT &= ~BV(0))

#define MRDY_Clr()             st(_MRDY_Clr(); _SS_Clr();)
#define MRDY_Set()             st(_MRDY_Set(); _SS_Set();)

// Configure for ISR on falling edge: SRDY.
#define HAL_CFG_SRDY_ISR         (P2IES = BV(6))

// Configure for ISR on rising edge: SRSP ready.
#define HAL_CFG_SRDY_RSP_ISR     (P2IES = 0)

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

// Flags set by ISR's to indicate events without blocking or invoking background functions.
extern volatile uint16 halEventFlags;

// Last value read from the ADC by ISR.
extern volatile uint16 halAdcVal;

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          halBoardInit
 *
 * @brief       This is the HAL board main initialization function.
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
void halBoardInit(void);

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
void halSlaveReset(void);

/**************************************************************************************************
 * @fn          halSPIWrite
 *
 * @brief       This function blocks (polling) on the SPI port; USART by ISR.
 *
 * input parameters
 *
 * @param       sss - SPI Slave to select
 * @param       pBuf - Pointer to the buffer that contains the data to transmit.
 * @param       len - Length of the data to transmit.
 *
 * output parameters
 *
 * @param       pBuf - SPI only: Pointer to the buffer that is filled with the Rx bytes.
 *
 * @return      None.
 **************************************************************************************************
 */
void halSPIWrite(uint8 sss, uint8 *pBuf, uint8 len);

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
void halTimerSet(uint8 tIdx, uint16 period, uint8 ctlFlags);

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
void halDelay(uint8 msecs, uint8 sleep);

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
uint8 halDelayDone(void);

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
void halReadTemp(void);

/*******************************************************************************
* @fn halLightSensorRead
*
* @brief Starts reading the value from the light sensor on a ZAccel node.
*
* The value is ranging from 0x00 to 0x0400
*
* input parameters: None.
*
* output parameters: None.
*
* @return None.
*******************************************************************************
*/
void halLightSensorRead(void);

/**************************************************************************************************
 * @fn          halReadBusV
 *
 * @brief       This function starts the ADC and sets up to read the channel for bus voltage.
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
void halReadBusV(void);

#endif

#ifdef HOST_MT
/**************************************************************************************************
 * @fn          halUARTRead
 *
 * @brief       This function blocks transfers the bytes read by ISR to the parameter buffer.
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
uint8 halUARTRead(uint8 port, uint8 *pBuf, uint8 len);

void halUARTWrite(uint8 port, uint8 *pBuf, uint8 len);
#endif

#endif
