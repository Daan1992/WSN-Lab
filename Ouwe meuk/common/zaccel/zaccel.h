/**************************************************************************************************
    Filename:       zaccel.h
    Revised:        $Date: 2008-02-20 09:47:29 -0800 (Wed, 20 Feb 2008) $
    Revision:       $Revision: 16446 $

    Description:

    Public interface file for the Z-Accel.

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
#ifndef ZACCEL_H
#define ZACCEL_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

// Convenience to minimize the include files in each module.
#include "hal_defs.h"
#include "hal_types.h"

// Indirection on include to abstract specific Z-Accel part from the sample application dependency.
#include "cc2480.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

// Bit mask for zaccelEvtFlags.
#define ZACCEL_EVT_NONE         0x0000
#define ZACCEL_START_CNF        0x0001
#define ZACCEL_BIND_CNF         0x0002
#define ZACCEL_ALLOW_BIND_CNF   0x0004
#define ZACCEL_SEND_DATA_CNF    0x0008
#define ZACCEL_READ_CFG_RSP     0x0010
#define ZACCEL_FIND_DEV_CNF     0x0020
#define ZACCEL_DEV_INFO_RSP     0x0040
#define ZACCEL_RCV_DATA_IND     0x0080
#define ZACCEL_SYS_RESET_IND    0x8000

// Bit mask for zaccelIndFlags.
#define ZACCEL_STATUS_CLEAR     0x0000U
#define ZACCEL_BIND_SUCCESS     0x0001U
#define ZACCEL_SEND_SUCCESS     0x0002U

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

#define ZACCEL_NWK_CONN (                      \
  (zaccelNwkState == DEV_END_DEVICE_UNAUTH) || \
  (zaccelNwkState == DEV_END_DEVICE)        || \
  (zaccelNwkState == DEV_ROUTER)            || \
  (zaccelNwkState == DEV_ZB_COORD)             \
)

#define ZACCEL_NWK_RTR (                       \
  (zaccelNwkState == DEV_ROUTER)            || \
  (zaccelNwkState == DEV_ZB_COORD)             \
)

#define ZACCEL_NWK_COOR  (zaccelNwkState == DEV_ZB_COORD)

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

extern devStates_t zaccelNwkState;
extern uint16 zaccelNwkAddr;

extern uint16 zaccelBndAddr;

extern uint16 zaccelEvtFlags;
extern uint16 zaccelIndFlags;

extern uint8  zaccelMsgBuf[ZACCEL_BUF_LEN];
extern uint16 zaccelDataCmd, zaccelDataSrc, zaccelDataLen;
extern uint8  zaccelDataBuf[ZACCEL_BUF_LEN];

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          zaccelInit
 *
 * @brief       This function initializes the host-to-ZACCEL RPC environment.
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
void zaccelInit(void);

/**************************************************************************************************
 * @fn          zaccelPoll
 *
 * @brief       Poll for ZACCEL ready and execute RPC transactions.
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
void zaccelPoll(void);

/**************************************************************************************************
 * @fn          zaccelRPC
 *
 * @brief       This function effects the requested RPC transaction across the configured medium.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the SPI RPC buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zaccelRPC(uint8 *pBuf);

#endif
