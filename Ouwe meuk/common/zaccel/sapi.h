/**********************`****************************************************************************
    Filename:       sapi.h
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
#ifndef SAPI_H
#define SAPI_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "zaccel.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define ZB_IEEE_SEARCH                    1

// Device Info Constants
#define ZB_INFO_DEV_STATE                 0
#define ZB_INFO_IEEE_ADDR                 1
#define ZB_INFO_SHORT_ADDR                2
#define ZB_INFO_PARENT_SHORT_ADDR         3
#define ZB_INFO_PARENT_IEEE_ADDR          4
#define ZB_INFO_CHANNEL                   5
#define ZB_INFO_PAN_ID                    6
#define ZB_INFO_EXT_PAN_ID                7

// SAPI SendDataRequest destinations
#define ZB_BINDING_ADDR                   INVALID_NODE_ADDR
#define ZB_BROADCAST_ADDR                 0xffff

// SAPI Status Codes
#define ZB_SUCCESS                        ZSuccess
#define ZB_FAILURE                        ZFailure
#define ZB_INVALID_PARAMETER              ZInvalidParameter
#define ZB_ALREADY_IN_PROGRESS            0x20
#define ZB_TIMEOUT                        0x21
#define ZB_AF_FAILURE                     afStatus_FAILED
#define ZB_AF_MEM_FAIL                    afStatus_MEM_FAIL
#define ZB_AF_INVALID_PARAMETER           afStatus_INVALID_PARAMETER

// SAPI Scan Duration Values
#define ZB_SCAN_DURATION_0                0   //  19.2 ms
#define ZB_SCAN_DURATION_1                1   //  38.4 ms
#define ZB_SCAN_DURATION_2                2   //  76.8 ms
#define ZB_SCAN_DURATION_3                3   //  153.6 ms
#define ZB_SCAN_DURATION_4                4   //  307.2 ms
#define ZB_SCAN_DURATION_5                5   //  614.4 ms
#define ZB_SCAN_DURATION_6                6   //  1.23 sec
#define ZB_SCAN_DURATION_7                7   //  2.46 sec
#define ZB_SCAN_DURATION_8                8   //  4.92 sec
#define ZB_SCAN_DURATION_9                9   //  9.83 sec
#define ZB_SCAN_DURATION_10               10  //  19.66 sec
#define ZB_SCAN_DURATION_11               11  //  39.32 sec
#define ZB_SCAN_DURATION_12               12  //  78.64 sec
#define ZB_SCAN_DURATION_13               13  //  157.28 sec
#define ZB_SCAN_DURATION_14               14  //  314.57 sec

// Device types definitions ( from ZGlobals.h file )
#define ZG_DEVICETYPE_COORDINATOR         0x00
#define ZG_DEVICETYPE_ROUTER              0x01
#define ZG_DEVICETYPE_ENDDEVICE           0x02

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          zb_SapiAppRegister
 *
 * @brief       This function packs and sends an RPC SAPI endpoint registration.
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
void zb_SapiAppRegister(const uint8 *pBuf);

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
void zb_SystemReset(void);

/**************************************************************************************************
 * @fn          zb_StartRequest
 *
 * @brief       The zb_StartRequest function starts the ZigBee stack.  When the
 *              ZigBee stack starts, the device reads configuration parameters
 *              from Nonvolatile memory and the device joins its network.  The
 *              ZigBee stack calls the zb_StartConrifm callback function when
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
void zb_StartRequest(void);

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
void zb_PermitJoiningRequest(uint16 destination, uint8 timeout);

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
void zb_BindDevice(uint8 create, uint16 commandId, uint8 *pDestination);

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
void zb_AllowBind(uint8 timeout);

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
                         uint8 txOptions, uint8 radius, uint8 len, uint8 *pData);

/**************************************************************************************************
 * @fn          zb_ReadConfiguration
 *
 * @brief       The zb_ReadConfiguration function is used to get a
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
void zb_ReadConfiguration(uint8 configId);

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
void zb_WriteConfiguration(uint8 configId, uint8 len, void *pValue);

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
void zb_FindDeviceRequest(uint8 searchType, void *searchKey);

/**************************************************************************************************
 * @fn          zb_GetDeviceInfo
 *
 * @brief       The zb_GetDeviceInfo function requests the Device Information Property.
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
void zb_GetDeviceInfo(uint8 param);

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
void zb_StartConfirm(uint8 status);

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
 * @param       allowBind - TRUE if the bind operation was initiated by a call
 *                          to zb_AllowBindRespones.  FALSE if the operation
 *                          was initiated by a call to ZB_BindDevice
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_BindConfirm(uint16 commandId, uint8 status);

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
void zb_AllowBindConfirm(uint16 source);

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
void zb_SendDataConfirm(uint8 handle, uint8 status);

/**************************************************************************************************
 * @fn          zb_SendDataConfirm
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
void zb_FindDeviceConfirm(uint8 searchType, uint8 *searchKey, uint8 *result);

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
void zb_GetDeviceInfoConfirm(uint8 param, void *pValue);

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
void zb_ReceiveDataIndication(uint16 source, uint16 command, uint16 len, uint8 *pData);

#endif
/**************************************************************************************************
*/
