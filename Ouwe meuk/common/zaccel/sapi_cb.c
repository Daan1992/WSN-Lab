/**************************************************************************************************
    Filename:       sapi_cb.c
    Revised:        $Date: 2008-02-20 09:47:29 -0800 (Wed, 20 Feb 2008) $
    Revision:       $Revision: 16446 $

    Description:

    This file contains the main functionality of the Host interface to the Z-Accel SAPI responses.

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
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          zb_GetDeviceInfoConfirm
 *
 * @brief       The zb_GetDeviceInfoConfirm function returns the Device Information Property.
 *
 * input parameters
 *
 * @param       param - The identifier for the device information
 *              pValue - A buffer to hold the device information
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_GetDeviceInfoConfirm(uint8 param, void *pValue)
{
  switch (param)
  {
    case ZB_INFO_DEV_STATE:
      zaccelNwkState = *(devStates_t *)pValue;
      break;
  
    case ZB_INFO_IEEE_ADDR:
      break;
  
    case ZB_INFO_SHORT_ADDR:
      zaccelNwkAddr = BUILD_UINT16(*((uint8 *)pValue), (*((uint8 *)pValue+1)) << 8);
      break;
  
    case ZB_INFO_PARENT_SHORT_ADDR:
      break;
  
    case ZB_INFO_PARENT_IEEE_ADDR:
      break;
  
    case ZB_INFO_CHANNEL:
      break;
  
    case ZB_INFO_PAN_ID:
      break;
  
    case ZB_INFO_EXT_PAN_ID:
      break;
  }
}

/**************************************************************************************************
 * @fn          zb_StartConfirm
 *
 * @brief       The zb_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * input parameters
 *
 * @param       status - The status of the start operation.  Status of ZB_SUCCESS indicates the 
 *                       start operation completed successfully. Else the status is an error code.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_StartConfirm(uint8 status)
{
  // If not started sucessful restart again.
  if (status != ZB_SUCCESS)
  {
    zb_StartRequest();
  }
}

/**************************************************************************************************
 * @fn          zb_BindConfirm
 *
 * @brief       The zb_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * input parameters
 *
 * @param       commandId - The command ID of the binding being confirmed.
 * @param       status - The status of the bind operation.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_BindConfirm(uint16 commandId, uint8 status)
{
  (void)commandId;

  if (status == ZB_SUCCESS)
  {
    zaccelIndFlags |= ZACCEL_BIND_SUCCESS;
  }
  else
  {
    zaccelIndFlags &= ~ZACCEL_BIND_SUCCESS;
  }
}

/**************************************************************************************************
 * @fn          zb_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 * 
 *
 * input parameters
 *
 * @param       source - the address of the source device that requested a binding with this device.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_AllowBindConfirm(uint16 source)
{
  zaccelBndAddr = source;
}

/**************************************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee after a send data operation completes
 *
 * input parameters
 *
 * @param       handle - The handle identifying the data transmission.
 * @param       status - The status of the operation.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_SendDataConfirm(uint8 handle, uint8 status)
{
  (void)handle;

  if (status == ZB_SUCCESS)
  {
    zaccelIndFlags |= ZACCEL_SEND_SUCCESS;
  }
  else
  {
    zaccelIndFlags &= ~ZACCEL_SEND_SUCCESS;
  }
}

/**************************************************************************************************
 * @fn          zb_FindDeviceConfirm
 *
 * @brief       Callback function called by the ZigBee stack when a find device operation completes.
 *
 * input parameters
 *
 * @param       searchType - The search type that was requested for this search operation.
 * @param       searchKey - The searchKey parameter contains information unique to the device being
 *                          discovered. The searchKey parameter is dependant on the searchType.
 *                          The content of the searchKey for each searchType follows:
 *                          - ZB_IEEE_SEARCH - The searchKey is the 64-bit IEEE address
 *                            of the device being discovered.
 * @param       result - A pointer to data containing the result of the search.
 *                       If the search type was ZB_IEEE_SEARCH, then this is a 16-bit address
 *                       of the device that matched the search.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_FindDeviceConfirm(uint8 searchType, uint8 *searchKey, uint8 *result)
{
  (void)searchType;  // Currently only supports IEEE Addr Search.
  (void)searchKey;
  (void)result;
}

/**************************************************************************************************
 * @fn          zb_ReceiveDataIndication
 *
 * @brief       The zb_ReceiveDataIndication callback function is called
 *              asynchronously by the ZigBee stack to notify the application
 *              when data is received from a peer device.
 *
 * input parameters
 *
 * @param       source - The short address of the peer device that sent the data
 * @param       command - The commandId associated with the data
 * @param       len - The number of bytes in the pData parameter
 * @param       pData - The data sent by the peer device
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_ReceiveDataIndication(uint16 source, uint16 command, uint16 len, uint8 *pData)
{
  uint8 cnt;

  zaccelDataSrc = source;
  zaccelDataCmd = command;

  if (len > ZACCEL_BUF_LEN)  // If SAPI sent more than 255 bytes, we cannot currently handle.
  {
    zaccelDataLen = ZACCEL_BUF_LEN;
  }
  else
  {
    zaccelDataLen = len;
  }

  for (cnt = 0; cnt < zaccelDataLen; cnt++)
  {
    zaccelDataBuf[cnt] = *pData++;
  }
}
