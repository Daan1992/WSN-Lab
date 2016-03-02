/**************************************************************************************************
    Filename:       spi.c
    Revised:        $Date: 2008-03-28 15:13:08 -0700 (Fri, 28 Mar 2008) $
    Revision:       $Revision: 16675 $

    Description:

    This file contains the main functionality of the Host interface to the ZACCEL via RPC by SPI.

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
#include "spi.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

// RPC protocol on SREQ is to wait for slave to signal ready by setting its SRDY line to opposite.
#define SRDY_RSP()  (!SRDY())

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

#define SRDY_WAIT_MSECS  10

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void  getSRDY1(void);
static uint8 getSRDY2(void);

/**************************************************************************************************
 * @fn          spiSREQ
 *
 * @brief       This function effects a synchronous transaction with the NP slave
 *              according to the RPC protocol for the SREQ.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the buffer to Tx across the SPI.
 *
 * output parameters
 *
 * @param       pBuf - Pointer to the data received across the SPI.
 *
 * @return      None.
 **************************************************************************************************
 */
void spiSREQ(uint8 *pBuf)
{
  do {
    getSRDY1();
    // Send the host SREQ-message across SPI.
    halSPIWrite(HAL_SPI_ZACCEL, pBuf, (*pBuf+MT_RPC_FRAME_HDR_SZ));

    /* Now setup the POLL command in the buffer which has just been transmitted and which will now
     * be used to receive the SREQ in response.
     */
    *pBuf = 0;                    // POLL command has zero data bytes.
    *(pBuf+1) = MT_RPC_CMD_POLL;  // POLL command MSB.
    *(pBuf+2) = 0;                // POLL command LSB.
  } while (!getSRDY2());

  halSPIWrite(HAL_SPI_ZACCEL, pBuf, MT_RPC_FRAME_HDR_SZ);  // Send the POLL across the SPI.

  // Receive the rest of the slave's message if it is longer than the POLL.
  if (*pBuf != 0)
  {
    halSPIWrite(HAL_SPI_ZACCEL, pBuf+MT_RPC_FRAME_HDR_SZ, *pBuf);
  }

  MRDY_Clr();                     // MRDY must be cleared before setting it again to talk again.
}

/**************************************************************************************************
 * @fn          spiAREQ
 *
 * @brief       This function effects an asynchronous transaction with the NP slave
 *              according to the RPC protocol for the AREQ.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the buffer to Tx across the SPI.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void spiAREQ(uint8 *pBuf)
{
  do {
    getSRDY1();
    // Send the host AREQ-message across SPI.
    halSPIWrite(HAL_SPI_ZACCEL, pBuf, (*pBuf+MT_RPC_FRAME_HDR_SZ));
  } while (!getSRDY2());

  MRDY_Clr();                     // MRDY must be cleared before setting it again to talk again.
}

/**************************************************************************************************
 * @fn          spiPOLL
 *
 * @brief       This function polls the NP slave according to the RPC protocol for POLL.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the buffer to Tx a POLL across the SPI and then receive the AREQ.
 *
 * output parameters
 *
 * @param       pBuf - Pointer to the buffer to Tx a POLL across the SPI and then receive the AREQ.
 *
 * @return      None.
 **************************************************************************************************
 */
void spiPOLL(uint8 *pBuf)
{
  // Setup the POLL command in the buffer.
  *pBuf = 0;                      // POLL command has zero data bytes.
  *(pBuf+1) = MT_RPC_CMD_POLL;    // POLL command MSB.
  *(pBuf+2) = 0;                  // POLL command LSB.

  spiSREQ(pBuf);
}

/**************************************************************************************************
 * @fn          getSRDY1
 *
 * @brief       This function cycles setting MRDY and timing out waiting for SRDY until success.
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
static void getSRDY1(void)
{
  MRDY_Set();     // MRDY must be set before talking to the slave.

  halDelay(SRDY_WAIT_MSECS, FALSE);
  while (!SRDY() && !halDelayDone())
  {
    HAL_LOW_POWER_MODE();
  }

  if (!SRDY())
  {
    MRDY_Clr();   // MRDY must be cleared before setting it again to talk again.
    halSlaveReset();
    MRDY_Set();   // MRDY must be set before talking to the slave.
  }
}

/**************************************************************************************************
 * @fn          getSRDY2
 *
 * @brief       This function times out waiting for the SRDY_RSP.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the slave signals SRDY_RSP.
 **************************************************************************************************
 */
static uint8 getSRDY2(void)
{
  HAL_CFG_SRDY_RSP_ISR;

  halDelay(SRDY_WAIT_MSECS, FALSE);
  while (!SRDY_RSP() && !halDelayDone())
  {
    HAL_LOW_POWER_MODE();
  }

  HAL_CFG_SRDY_ISR;

  return SRDY_RSP();
}
