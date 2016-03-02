/**************************************************************************************************
    Filename:       sapi.c
    Revised:        $Date: 2008-02-21 06:14:24 -0800 (Thu, 21 Feb 2008) $
    Revision:       $Revision: 16454 $

    Description:

    This file contains the main functionality of the Host interface to the Z-Accel SAPI control.

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

#include <string.h>
#include "zaccel.h"
#include "sapi.h"

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

static void sapiReq(uint8 cmd, uint8 arg, uint8 *req, uint8 len);

/**************************************************************************************************
 * @fn          zb_SapiAppRegister
 *
 * @brief       This function packs and sends an RPC SAPI endpoint registration.
 *
 *              Modified by MW to handle more than one cluster.
 *
 * input parameters
 *
 * @param       ep - The buffer containing the EndPoint payload.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_SapiAppRegister(const uint8 *ep)
{
  uint8 *pBuf = zaccelMsgBuf;

  // Determine the size of the endpoint data. Index 7 contains the number of input clusters.
  uint8 size = 2 * ep[7];   // Get the size of input cluster list (cluster id's are 2 bytes big)
  size += 2 * ep[8 + size]; // Add the size of output cluster list
  size += 9;                // Add the size of an endpoint with empty cluster lists

  *(pBuf+MT_RPC_POS_LEN) = size;
  *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI | (uint8)(MT_RPC_CMD_SREQ);
  *(pBuf+MT_RPC_POS_CMD1) = MT_SAPI_APP_REGISTER_REQ;
  memcpy((pBuf+MT_RPC_POS_DAT0), ep, size);

  zaccelRPC(pBuf);
}

/**************************************************************************************************
 * @fn          zb_SystemReset
 *
 * @brief       The zb_SystemReset function reboots the ZigBee device. The zb_SystemReset function
 *              can be called after a call to zb_WriteConfiguration to restart Z-Stack with the 
 *              updated configuration.
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
void zb_SystemReset(void)
{
  
  sapiReq(MT_SAPI_SYS_RESET, 0, NULL, 0);
}

/**************************************************************************************************
 * @fn          zb_StartRequest
 *
 * @brief       The zb_StartRequest function starts the ZigBee stack.  When the
 *              ZigBee stack starts, the device reads configuration parameters
 *              from Nonvolatile memory and the device joins its network.  The
 *              ZigBee stack calls the zb_StartConfirm callback function when
 *              the startup process completes.
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
void zb_StartRequest(void)
{
  uint8 val = (ZCD_STARTOPT_RESTORE_STATE | ZCD_STARTOPT_AUTO_START);
  zb_WriteConfiguration(ZCD_NV_STARTUP_OPTION, 1, &val);
  zb_SystemReset();
}

/**************************************************************************************************
 * @fn          zb_PermitJoiningRequest
 *
 * @brief       The zb_PermitJoiningRequest function is used to control the joining permissions and
 *              thus allow or disallow new devices from joining the network.
 *
 * input parameters
 *
 * @param       destination - The destination parameter indicates the address of the device for
 *                            which the joining permissions should be set. This is usually the 
 *                            local device address or the special broadcast address that denotes
 *                            all routers and coordinator ( 0xFFFC ). This way the joining 
 *                            permissions of a single device or the whole network can be controlled.
 * @param       timeout -  Indicates the amount of time in seconds for which the joining permissions
 *                         should be turned on.  If timeout is set to 0x00, the device will turn off
 *                         the joining permissions indefinitely. If it is set to 0xFF,
 *                         the joining permissions will be turned on indefinitely.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_PermitJoiningRequest(uint16 destination, uint8 timeout)
{
  uint8 *pBuf = zaccelMsgBuf;

  *(pBuf+MT_RPC_POS_LEN) = 3;
  *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI | (uint8)(MT_RPC_CMD_SREQ);
  *(pBuf+MT_RPC_POS_CMD1) = MT_SAPI_PMT_JOIN_REQ;
  *(pBuf+MT_RPC_POS_DAT0) = (uint8)destination;
  *(pBuf+MT_RPC_POS_DAT0+1) = (uint8)(destination >> 8);
  *(pBuf+MT_RPC_POS_DAT0+2) = timeout;

  zaccelRPC(pBuf);
}

/**************************************************************************************************
 * @fn          zb_BindDevice
 *
 * @brief       The zb_BindDevice function establishes or removes a ‘binding’
 *              between two devices.  Once bound, an application can send
 *              messages to a device by referencing the commandId for the
 *              binding.
 *
 * input parameters
 *
 * @param       create - TRUE to create a binding, FALSE to remove a binding
 * @param       commandId - The identifier of the binding
 * @param       pDestination - The 64-bit IEEE address of the device to bind to
 *
 * output parameters
 *
 * @return      None. The status of the bind operation is returned in the zb_BindConfirm callback.
 **************************************************************************************************
 */
void zb_BindDevice(uint8 create, uint16 commandId, uint8 *pDestination)
{
  uint8 *pBuf = zaccelMsgBuf;

  *(pBuf+MT_RPC_POS_LEN) = 11;
  *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI | (uint8)(MT_RPC_CMD_SREQ);
  *(pBuf+MT_RPC_POS_CMD1) = MT_SAPI_BIND_DEVICE_REQ;
  *(pBuf+MT_RPC_POS_DAT0) = create;
  *(pBuf+MT_RPC_POS_DAT0+1) = (uint8)commandId;
  *(pBuf+MT_RPC_POS_DAT0+2) = (uint8)(commandId >> 8);
  memcpy((pBuf+MT_RPC_POS_DAT0+3), pDestination, 8);

  zaccelRPC(pBuf);
}

/**************************************************************************************************
 * @fn          zb_AllowBind
 *
 * @brief       The zb_AllowBind function puts the device into the Allow Binding Mode for a given 
 *              period of time. A peer device can establish a binding to a device in the
 *              Allow Binding Mode by calling zb_BindDevice with a destination address of NULL.
 *
 * input parameters
 *
 * @param       timeout - The number of seconds to remain in the allow binding mode.
 *                        Valid values range from 1 through 65.
 *                        If 0, the Allow Bind mode will be set false without TO.
 *                        If greater than 64, the Allow Bind mode will be true.
 *
 * output parameters
 *
 * @return      None. The status of the bind operation is returned in the zb_BindConfirm callback.
 **************************************************************************************************
 */
void zb_AllowBind(uint8 timeout)
{
  uint8 *pBuf = zaccelMsgBuf;

  *(pBuf+MT_RPC_POS_LEN) = 1;
  *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI | (uint8)(MT_RPC_CMD_SREQ);
  *(pBuf+MT_RPC_POS_CMD1) = MT_SAPI_ALLOW_BIND_REQ;
  *(pBuf+MT_RPC_POS_DAT0) = timeout;

  zaccelRPC(pBuf);
}

/******************************************************************************
 * @fn          zb_SendDataRequest
 *
 * @brief       The zb_SendDataRequest function initiates transmission of data to a peer device.
 *
 * @param       destination - The destination of the data.  The destination must be one of these:
 *                            - 16-Bit short address of device [0-0xfffD]
 *                            - ZB_BROADCAST_ADDR sends the data to all devices in the network.
 *                            - ZB_BINDING_ADDR sends the data to a previously bound device.
 *
 * @param       commandId - The command ID to send with the message.  If the ZB_BINDING_ADDR
 *                          destination is used, this parameter also indicates the binding to use.
 * @param       handle - A handle used to identify the send data request.
 * @param       txOptions - TRUE if requesting acknowledgement from the destination.
 * @param       radius - The max number of hops the packet can travel through before it is dropped.
 * @param       len - The size of the pData buffer in bytes
 * @param       pData - Pointer to the data buffer containing bytes to send.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_SendDataRequest(uint16 destination, uint16 commandId, uint8 handle,
                         uint8 txOptions, uint8 radius, uint8 len, uint8 *pData)
{
  uint8 *pBuf = zaccelMsgBuf;

  *(pBuf+MT_RPC_POS_LEN) = 8 + len;
  *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI | (uint8)(MT_RPC_CMD_SREQ);
  *(pBuf+MT_RPC_POS_CMD1) = MT_SAPI_SEND_DATA_REQ;
  *(pBuf+MT_RPC_POS_DAT0) = (uint8)destination;
  *(pBuf+MT_RPC_POS_DAT0+1) = (uint8)(destination >> 8);
  *(pBuf+MT_RPC_POS_DAT0+2) = (uint8)commandId;
  *(pBuf+MT_RPC_POS_DAT0+3) = (uint8)(commandId >> 8);
  *(pBuf+MT_RPC_POS_DAT0+4) = handle;
  *(pBuf+MT_RPC_POS_DAT0+5) = txOptions;
  *(pBuf+MT_RPC_POS_DAT0+6) = radius;
  *(pBuf+MT_RPC_POS_DAT0+7) = len;
  if (pData)
  {
    memcpy((pBuf+MT_RPC_POS_DAT0+8), pData, len);
  }

  zaccelRPC(pBuf);
}

/**************************************************************************************************
 * @fn          zb_ReadConfiguration
 *
 * @brief       The zb_ReadConfiguration function is used to read a
 *              Configuration Protperty from Nonvolatile memory.
 *
 * input parameters
 *
 * @param       configId - The identifier for the configuration property
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_ReadConfiguration(uint8 configId)
{
  uint8 *pBuf = zaccelMsgBuf;

  *(pBuf+MT_RPC_POS_LEN) = 1;
  *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI | (uint8)(MT_RPC_CMD_SREQ);
  *(pBuf+MT_RPC_POS_CMD1) = MT_SAPI_READ_CFG_REQ;
  *(pBuf+MT_RPC_POS_DAT0) = configId;

  zaccelRPC(pBuf);
}

/**************************************************************************************************
 * @fn          zb_WriteConfiguration
 *
 * @brief       The zb_ReadConfiguration function is used to write a
 *              Configuration Protperty from Nonvolatile memory.
 *
 * input parameters
 *
 * @param       configId - The identifier for the configuration property
 * @param       len - The size of the pValue buffer in bytes
 * @param       pValue - A buffer containing the new value of the
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_WriteConfiguration(uint8 configId, uint8 len, void *pValue)
{
  uint8 *pBuf = zaccelMsgBuf;

  *(pBuf+MT_RPC_POS_LEN) = 2 + len;
  *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI | (uint8)(MT_RPC_CMD_SREQ);
  *(pBuf+MT_RPC_POS_CMD1) = MT_SAPI_WRITE_CFG_REQ;
  *(pBuf+MT_RPC_POS_DAT0) = configId;
  *(pBuf+MT_RPC_POS_DAT0+1) = len;
  memcpy((pBuf+MT_RPC_POS_DAT0+2), pValue, len);

  zaccelRPC(pBuf);
}

/**************************************************************************************************
 * @fn          zb_FindDeviceRequest
 *
 * @brief       The zb_FindDeviceRequest function is used to determine the
 *              short address for a device in the network.  The device initiating
 *              a call to zb_FindDeviceRequest and the device being discovered
 *              must both be a member of the same network.  When the search is
 *              complete, the zv_FindDeviceConfirm callback function is called.
 *
 * input parameters
 *
 * @param       searchType - The type of search to perform. Can be one of following:
 *                           ZB_IEEE_SEARCH - Search for 16-bit addr given IEEE addr.
 * @param       searchKey - Value to search on.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_FindDeviceRequest(uint8 searchType, void *searchKey)
{
  uint8 *pBuf = zaccelMsgBuf;

  if (searchType == ZB_IEEE_SEARCH)
  {
    *(pBuf+MT_RPC_POS_LEN) = 8;
    *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI | (uint8)(MT_RPC_CMD_SREQ);
    *(pBuf+MT_RPC_POS_CMD1) = MT_SAPI_FIND_DEV_REQ;
    memcpy((pBuf+MT_RPC_POS_DAT0), searchKey, 8);

    zaccelRPC(pBuf);
  }
}

/**************************************************************************************************
 * @fn          zb_GetDeviceInfo
 *
 * @brief       The zb_GetDeviceInfo function retrieves a Device Information Property.
 *
 * input parameters
 *
 * @param       param - The identifier for the device information
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_GetDeviceInfo(uint8 param)
{
  sapiReq(MT_SAPI_GET_DEV_INFO_REQ, param, NULL, 0);
}

/**************************************************************************************************
 * @fn          sapiReq
 *
 * @brief       This function packs and sends an RPC SAPI request.
 *
 * input parameters
 *
 * @param       cmd - A valid SAPI command.
 * @param       arg - A valid argument corresponding to the SAPI command.
 * @param       req - A buffer containing the contents of the request, or NULL.
 * @param       len - The non-zero length of a non-NULL req buffer, or NULL.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void sapiReq(uint8 cmd, uint8 arg, uint8 *req, uint8 len)
{
  uint8 *pBuf = zaccelMsgBuf;

  *(pBuf+MT_RPC_POS_LEN) = 2 + len;
  *(pBuf+MT_RPC_POS_CMD0) = MT_RPC_SYS_SAPI;
  *(pBuf+MT_RPC_POS_CMD1) = cmd;
  *(pBuf+MT_RPC_POS_DAT0) = arg;
  *(pBuf+MT_RPC_POS_DAT0+1) = len;
  if (req)
  {
    memcpy((pBuf+MT_RPC_POS_DAT0+2), req, len);
  }

  if (cmd == MT_SAPI_SYS_RESET)
  {
    *(pBuf+MT_RPC_POS_CMD0) |= MT_RPC_CMD_AREQ;
  }
  else
  {
    *(pBuf+MT_RPC_POS_CMD0) |= MT_RPC_CMD_SREQ;
  }
  zaccelRPC(pBuf);
}
