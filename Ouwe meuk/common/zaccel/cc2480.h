/**************************************************************************************************
    Filename:       cc2480.h
    Revised:        $Date: 2008-03-28 15:13:08 -0700 (Fri, 28 Mar 2008) $
    Revision:       $Revision: 16675 $

    Description: Public interface file for the Z-Accel Part # cc2480.

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
#ifndef CC2480_H
#define CC2480_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define ZSuccess                           0x00
#define ZFailure                           0x01
#define ZInvalidParameter                  0x02

#define Z_EXTADDR_LEN                      8
#define INVALID_NODE_ADDR                  0xFFFE

#define AF_FRAGMENTED                      0x01
#define AF_ACK_REQUEST                     0x10
#define AF_DISCV_ROUTE                     0x20
#define AF_EN_SECURITY                     0x40
#define AF_SKIP_ROUTING                    0x80

#define NWK_BROADCAST_SHORTADDR_DEVALL     0xFFFF
#define NWK_BROADCAST_SHORTADDR            NWK_BROADCAST_SHORTADDR_DEVALL
// The default network radius set twice the value of <nwkMaxDepth>.
#define DEF_NWK_RADIUS                     10
#define AF_DEFAULT_RADIUS                  DEF_NWK_RADIUS

// OSAL NV item IDs
#define ZCD_NV_EXTADDR                     0x0001
#define ZCD_NV_BOOTCOUNTER                 0x0002
#define ZCD_NV_STARTUP_OPTION              0x0003
#define ZCD_NV_START_DELAY                 0x0004

// ZDO NV Item IDs
#define ZCD_NV_USERDESC                    0x0081
#define ZCD_NV_NWKKEY                      0x0082
#define ZCD_NV_PANID                       0x0083
#define ZCD_NV_CHANLIST                    0x0084
#define ZCD_NV_LEAVE_CTRL                  0x0085
#define ZCD_NV_SCAN_DURATION               0x0086
#define ZCD_NV_LOGICAL_TYPE                0x0087

// NWK Layer NV item IDs
#define ZCD_NV_POLL_RATE                   0x0024
#define ZCD_NV_RESPONSE_POLL_RATE          0x0026

// ZCD_NV_STARTUP_OPTION values
//   These are bit weighted - you can OR these together.
//   Setting one of these bits will set their associated NV items
//   to code initialized values.
#define ZCD_STARTOPT_DEFAULT_CONFIG_STATE  0x01
#define ZCD_STARTOPT_DEFAULT_NETWORK_STATE 0x02
#define ZCD_STARTOPT_AUTO_START            0x04
#define ZCD_STARTOPT_CLEAR_CONFIG          ZCD_STARTOPT_DEFAULT_CONFIG_STATE
#define ZCD_STARTOPT_RESTORE_STATE         ZCD_STARTOPT_DEFAULT_NETWORK_STATE

/* 1st byte is the length of the data field, 2nd/3rd bytes are command field. */
#define MT_RPC_FRAME_HDR_SZ                3

/* maximum length of data in the general frame format */
#define MT_RPC_DATA_MAX                   (256 - MT_RPC_FRAME_HDR_SZ)

/* The 3 MSB's of the 1st command field byte are for command type. */
#define MT_RPC_CMD_TYPE_MASK               0xE0

/* The 5 LSB's of the 1st command field byte are for the subsystem. */
#define MT_RPC_SUBSYSTEM_MASK              0x1F

/* position of fields in the general format frame */
#define MT_RPC_POS_LEN                     0
#define MT_RPC_POS_CMD0                    1
#define MT_RPC_POS_CMD1                    2
#define MT_RPC_POS_DAT0                    3

/* Error codes */
#define MT_RPC_SUCCESS                     0     /* success */
#define MT_RPC_ERR_SUBSYSTEM               1     /* invalid subsystem */
#define MT_RPC_ERR_COMMAND_ID              2     /* invalid command ID */
#define MT_RPC_ERR_PARAMETER               3     /* invalid parameter */
#define MT_RPC_ERR_LENGTH                  4     /* invalid length */

/***************************************************************************************************
 * SYS COMMANDS
 ***************************************************************************************************/

// SYS MT Command Identifiers
/* AREQ to host */
#define MT_SYS_RESET_IND                    0x80

/***************************************************************************************************
 * AF COMMANDS
 ***************************************************************************************************/

/* SREQ/SRSP */
#define MT_AF_REGISTER                      0x00
#define MT_AF_DATA_REQUEST                  0x01

/* AREQ to host */
#define MT_AF_DATA_CONFIRM                  0x80
#define MT_AF_INCOMING_MSG                  0x81

/***************************************************************************************************
 * ZDO COMMANDS
 ***************************************************************************************************/

/* SREQ/SRSP */
#define MT_ZDO_MATCH_DESC_REQ               0x06

/* AREQ to host */
#define MT_ZDO_MATCH_DESC_RSP               0x86

/***************************************************************************************************
 * SAPI COMMANDS
 ***************************************************************************************************/

// SAPI MT Command Identifiers
/* AREQ from Host */
#define MT_SAPI_SYS_RESET                   0x09

/* SREQ/SRSP */
#define MT_SAPI_START_REQ                   0x00
#define MT_SAPI_BIND_DEVICE_REQ             0x01
#define MT_SAPI_ALLOW_BIND_REQ              0x02
#define MT_SAPI_SEND_DATA_REQ               0x03
#define MT_SAPI_READ_CFG_REQ                0x04
#define MT_SAPI_WRITE_CFG_REQ               0x05
#define MT_SAPI_GET_DEV_INFO_REQ            0x06
#define MT_SAPI_FIND_DEV_REQ                0x07
#define MT_SAPI_PMT_JOIN_REQ                0x08
#define MT_SAPI_APP_REGISTER_REQ            0x0a

/* AREQ to host */
#define MT_SAPI_START_CNF                   0x80
#define MT_SAPI_BIND_CNF                    0x81
#define MT_SAPI_ALLOW_BIND_CNF              0x82
#define MT_SAPI_SEND_DATA_CNF               0x83
#define MT_SAPI_READ_CFG_RSP                0x84
#define MT_SAPI_FIND_DEV_CNF                0x85
#define MT_SAPI_DEV_INFO_RSP                0x86
#define MT_SAPI_RCV_DATA_IND                0x87

/* ------------------------------------------------------------------------------------------------
 *                                          Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum
{
  DEV_HOLD,               // Initialized - not started automatically
  DEV_INIT,               // Initialized - not connected to anything
  DEV_NWK_DISC,           // Discovering PAN's to join
  DEV_NWK_JOINING,        // Joining a PAN
  DEV_NWK_REJOIN,         // ReJoining a PAN, only for end devices
  DEV_END_DEVICE_UNAUTH,  // Joined but not yet authenticated by trust center
  DEV_END_DEVICE,         // Started as device after authentication
  DEV_ROUTER,             // Device joined, authenticated and is a router
  DEV_COORD_STARTING,     // Started as Zigbee Coordinator
  DEV_ZB_COORD,           // Started as Zigbee Coordinator
  DEV_NWK_ORPHAN          // Device has lost information about its parent..
} devStates_t;

typedef enum {
  MT_RPC_CMD_POLL = 0x00,
  MT_RPC_CMD_SREQ = 0x20,
  MT_RPC_CMD_AREQ = 0x40,
  MT_RPC_CMD_SRSP = 0x60,
  MT_RPC_CMD_RES4 = 0x80,
  MT_RPC_CMD_RES5 = 0xA0,
  MT_RPC_CMD_RES6 = 0xC0,
  MT_RPC_CMD_RES7 = 0xE0
} mtRpcCmdType_t;

typedef enum {
  MT_RPC_SYS_RES0,   /* Reserved. */
  MT_RPC_SYS_SYS,
  MT_RPC_SYS_MAC,
  MT_RPC_SYS_NWK,
  MT_RPC_SYS_AF,
  MT_RPC_SYS_ZDO,
  MT_RPC_SYS_SAPI,   /* Simple API. */
  MT_RPC_SYS_UTIL,
  MT_RPC_SYS_DBG,
  MT_RPC_SYS_APP,
  MT_RPC_SYS_MAX     /* Maximum value, must be last */
  /* 10-32 available, not yet assigned. */
} mtRpcSysType_t;

#endif
