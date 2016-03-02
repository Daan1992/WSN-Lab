/**************************************************************************************************
    Filename:       zaccel.c
    Revised:        $Date: 2008-03-28 15:13:08 -0700 (Fri, 28 Mar 2008) $
    Revision:       $Revision: 16675 $

    Description: This file contains the main functionality of the Host side of the RPC protocol.

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

#include "zaccel.h"
#include "hal_board.h"
#include "mt.h"
#include "sapi.h"
#include "spi.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

devStates_t zaccelNwkState;
uint16 zaccelNwkAddr;

uint16 zaccelBndAddr;

uint16 zaccelEvtFlags;
uint16 zaccelIndFlags;

uint8 zaccelMsgBuf[ZACCEL_BUF_LEN];
uint16 zaccelDataCmd, zaccelDataSrc, zaccelDataLen;
uint8 zaccelDataBuf[ZACCEL_BUF_LEN];

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void rpcRsp(uint8 *pBuf);
static uint16 sysRsp(uint8 *pBuf);
static uint16 sapiRsp(uint8 *pBuf);

/**************************************************************************************************
 * @fn          zaccelInit
 *
 * @brief       This function initializes the host RPC environment.
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
void zaccelInit(void)
{
  halSlaveReset();

  /* In case the Z-Accel is still doing a power-up CRC check, toggle MRDY to signal to
   * write the result and thereby bypass all future power-up CRC checks.
   */
  MRDY_Set();
  MRDY_Clr();

  zaccelNwkAddr  = INVALID_NODE_ADDR;
  zaccelEvtFlags = ZACCEL_EVT_NONE;
  zaccelIndFlags = ZACCEL_STATUS_CLEAR;
}

/**************************************************************************************************
 * @fn          zaccelPoll
 *
 * @brief       Poll the Z-Accel slave for asynchronous command and drive the RPC transaction.
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
void zaccelPoll(void)
{
  // If the NP slave indicates that an AREQ is ready, then poll for it.
  if (SRDY())
  {
    uint8 *pBuf = zaccelMsgBuf;

    spiPOLL(pBuf);
#ifdef HOST_MT
    mtTx(pBuf);
#endif
    rpcRsp(pBuf);
  }
}

/**************************************************************************************************
 * @fn          zaccelRPC
 *
 * @brief       This function effects the requested RPC transaction across the configured medium.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the RPC buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zaccelRPC(uint8 *pBuf)
{
  if (MT_RPC_CMD_SREQ == (*(pBuf+MT_RPC_POS_CMD0) & (MT_RPC_CMD_SREQ | MT_RPC_CMD_AREQ)))
  {
    spiSREQ(pBuf);
#ifdef HOST_MT
    mtTx(pBuf);
#endif
    rpcRsp(pBuf);
  }
  else
  {
    spiAREQ(pBuf);
  }
}

/**************************************************************************************************
 * @fn          rpcRsp
 *
 * @brief       This function acts on an RPC response message.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the RPC response buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void rpcRsp(uint8 *pBuf)
{
  uint16 event;

  switch (*(pBuf+MT_RPC_POS_CMD0) & MT_RPC_SUBSYSTEM_MASK)
  {
    case MT_RPC_SYS_SYS:
      event = sysRsp(pBuf);
      break;

    case MT_RPC_SYS_SAPI:
      event = sapiRsp(pBuf);
      break;

    default:
      event = ZACCEL_EVT_NONE;
      break;
  }

  zaccelEvtFlags |= event;
}

/**************************************************************************************************
 * @fn          sysRsp
 *
 * @brief       This function parses a MT_RPC_SYS_SYS RPC message.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the RPC response buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      The corresponding Host event flag, if a recognized response is parsed.
 **************************************************************************************************
 */
static uint16 sysRsp(uint8 *pBuf)
{
  uint16 event = ZACCEL_EVT_NONE;

  switch (*(pBuf+MT_RPC_POS_CMD1))
  {
    case MT_SYS_RESET_IND:
      event = ZACCEL_SYS_RESET_IND;
      break;

    default:
      break;
  }

  return event;
}

/**************************************************************************************************
 * @fn          sapiRsp
 *
 * @brief       This function parses an RPC MT_RPC_SYS_SAPI response message.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the RPC response buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      The corresponding Host event flag, if a recognized response is parsed.
 **************************************************************************************************
 */
static uint16 sapiRsp(uint8 *pBuf)
{
  uint16 event = ZACCEL_EVT_NONE;

  switch (*(pBuf+MT_RPC_POS_CMD1))
  {
    case MT_SAPI_GET_DEV_INFO_REQ:
      zb_GetDeviceInfoConfirm(*(pBuf+MT_RPC_POS_DAT0), (pBuf+MT_RPC_POS_DAT0+1));
      event = ZACCEL_DEV_INFO_RSP;
      break;

    case MT_SAPI_START_CNF:
      zb_StartConfirm(*(pBuf+MT_RPC_POS_DAT0));
      event = ZACCEL_START_CNF;
      break;

    case MT_SAPI_BIND_CNF:
      event = BUILD_UINT16(*(pBuf+MT_RPC_POS_DAT0), *(pBuf+MT_RPC_POS_DAT0+1));
      zb_BindConfirm(event, *(pBuf+MT_RPC_POS_DAT0+2));
      event = ZACCEL_BIND_CNF;
      break;

    case MT_SAPI_ALLOW_BIND_CNF:
      event = BUILD_UINT16(*(pBuf+MT_RPC_POS_DAT0), *(pBuf+MT_RPC_POS_DAT0+1));
      zb_AllowBindConfirm(event);
      event = ZACCEL_ALLOW_BIND_CNF;
      break;

    case MT_SAPI_SEND_DATA_CNF:
      zb_SendDataConfirm(*(pBuf+MT_RPC_POS_DAT0), *(pBuf+MT_RPC_POS_DAT0+1));
      event = ZACCEL_SEND_DATA_CNF;
      break;

    case MT_SAPI_READ_CFG_RSP:
      event = ZACCEL_READ_CFG_RSP;
      break;

    case MT_SAPI_FIND_DEV_CNF:
      zb_FindDeviceConfirm(*(pBuf+MT_RPC_POS_DAT0),(pBuf+MT_RPC_POS_DAT0+1),(pBuf+MT_RPC_POS_DAT0+3));
      event = ZACCEL_FIND_DEV_CNF;
      break;

    case MT_SAPI_RCV_DATA_IND:
      if (!(zaccelEvtFlags & ZACCEL_RCV_DATA_IND))  // Currently limited by 1 incoming data buffer.
      {
        event = ZACCEL_RCV_DATA_IND;

        pBuf += MT_RPC_POS_DAT0;
        zaccelDataSrc  = *pBuf++;
        zaccelDataSrc += (uint16)(*pBuf++) << 8;
        zaccelDataCmd  = *pBuf++;
        zaccelDataCmd += (uint16)(*pBuf++) << 8;
        zaccelDataLen  = *pBuf++;
        zaccelDataLen += (uint16)(*pBuf++) << 8;
        zb_ReceiveDataIndication(zaccelDataSrc, zaccelDataCmd, zaccelDataLen, pBuf);
      }
      break;

    default:
      break;
  }

  return event;
}
