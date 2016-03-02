/**************************************************************************************************
    Filename:       mt.c
    Revised:        $Date: 2008-04-03 18:32:40 -0700 (Thu, 03 Apr 2008) $
    Revision:       $Revision: 16728 $

    Description: This file defines the main functionality of the interface to MT via Z-Accel.

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
#include "mt.h"

#ifdef HOST_MT
/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          mtInit
 *
 * @brief       This function initializes the host MT environment.
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
void mtInit(void)
{
}

/**************************************************************************************************
 * @fn          mtRx
 *
 * @brief       This function translates and then transfers a UART MT packet as an RPC packet.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the Rx data was processed; FALSE otherwise.
 **************************************************************************************************
 */
uint8 mtRx(void)
{
  uint8 *pBuf = zaccelMsgBuf;
  uint8 len = halUARTRead(HAL_PORT_MT, pBuf, ZACCEL_BUF_LEN-1);
  uint8 idx = 0;

  do {
    while (len)
    {
      if (pBuf[idx++] == SOP_VALUE)
      {
        break;
      }
      len--;
    }

    if (len > MT_RPC_FRAME_HDR_SZ)
    {
      uint8 cnt = pBuf[idx] + MT_RPC_FRAME_HDR_SZ;
      if ((calcFCS((pBuf+idx), cnt)) == pBuf[idx+cnt])
      {
        zaccelRPC(pBuf+idx);
      }
    }
  } while (len);

  return TRUE;
}

/**************************************************************************************************
 * @fn          mtTx
 *
 * @brief       This function translates and then transfers an RPC packet as a UART MT packet.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the buffer to be used.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void mtTx(uint8 *pBuf)
{
  /*uint8 tmp = SOP_VALUE;                      disabled for teraterm

  halUARTWrite(HAL_PORT_MT, &tmp, 1);
  tmp = *pBuf + MT_RPC_FRAME_HDR_SZ;
  halUARTWrite(HAL_PORT_MT, pBuf, tmp);
  tmp = calcFCS(pBuf, tmp);
  halUARTWrite(HAL_PORT_MT, &tmp, 1);*/
}

/**************************************************************************************************
 * @fn          calcFCS
 *
 * @brief       This function calculates the FCS checksum for the MT protocol serial message to the
 *              Z-Tool PC GUI.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the end of a buffer to calculate the FCS.
 * @param       len - Length of the pBuf.
 *
 * output parameters
 *
 * None.
 *
 * @return      The calculated FCS.
 **************************************************************************************************
 */
uint8 calcFCS(uint8 *pBuf, uint8 len)
{
  uint8 rtrn = 0;

  while (len--)
  {
    rtrn ^= *pBuf++;
  }

  return rtrn;
}
#endif
