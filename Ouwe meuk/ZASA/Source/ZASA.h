/**************************************************************************************************
    Filename:       ZASA.h
    Revised:        $Date: 2008-04-09 15:06:31 -0700 (Wed, 09 Apr 2008) $
    Revision:       $Revision: 16786 $

    Description: Public interface file for the Z-Accel Sample Application.

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
#ifndef ZASA_H
#define ZASA_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "cc2480.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

// Bit masks for appFlags.
#define appLowPwrF                        0x80
#define appPermittingF                    0x40
#define appSendingF                       0x20
#define appSinkingF                       0x10
#define appSetPollF                       0x08

#define appLuxF                           0x01
#define appIdleF                          0x00

#define ZASA_PROFILE_ID                   0x0F10
#define ZASA_PROFILE_ID_LSB               0x10
#define ZASA_PROFILE_ID_MSB               0x0F

#define SRCE_DOOR_ID                      0x0001
#define SRCE_DOOR_ID_LSB                  0x01
#define SRCE_DOOR_ID_MSB                  0x00

#define SRCE_LIGHT_ID                     0x0002
#define SRCE_LIGHT_ID_LSB                 0x02
#define SRCE_LIGHT_ID_MSB                 0x00
   
#define SRCE_KEY_ID                       0x0003
#define SRCE_KEY_ID_LSB                   0x03
#define SRCE_KEY_ID_MSB                   0x00  
   


#define SRCE_LIGHT_SZ                     2
#define SRCE_LIGHT_BTNSTATE               0  // btn value
#define SRCE_LIGHT_VALUE                  1  // Lightvalue    

#define SRCE_KEY_SZ                       1
#define SRCE_KEY_PRESS                    0 

#define SRCE_DOOR_SZ                      1
#define SRCE_DOOR_STATE                   0 

#define SINK_ENDPOINT_ID                  2
#define SINK_DEVICE_ID_LSB                4
#define SINK_DEVICE_ID_MSB                0
#define SINK_DEVICE_VERSION               1
#define SINK_LATENCY                      0
#define SINK_CLUSTER_IN_CNT               2 // light en key 
#define SINK_CLUSTER_OUT_CNT              1 // door 

#define SRCE_ENDPOINT_ID                  1
#define SRCE_DEVICE_ID_LSB                3
#define SRCE_DEVICE_ID_MSB                0
#define SRCE_DEVICE_VERSION               1
#define SRCE_LATENCY                      0
#define SRCE_CLUSTER_IN_CNT               1 // router: nothing, ed: door 
#define SRCE_CLUSTER_OUT_CNT              1 // router: light, ed: key

#define APP_BTN_INTERVAL                  2      // Count button presses in a 2-second interval.
#define APP_BLINK_INTERVAL                1      // Blink at 1-Hz.
#define APP_BLINK_ON_TIME                 1      // LED on for 1 msec is visible.

/* This timeout for attempting to join must be greatly lengthened as channels are added to the
 * ZACCEL_NV_CHANLIST in ZASA.cfg.
 * It is a trial and error empirical process to find the right setting according to the number of
 * channels to be scanned and the channel access availability on each of those channels.
 */
#define APP_JOIN_TIME                     6      //  6 secs to attempt to join.
#define APP_JOIN_WAIT                     30     // 30 secs to sleep before re-attempt to join.
#define APP_BIND_TIME                     10     // 10 secs to attempt to bind.
#define APP_BIND_WAIT                     30     // 30 secs to sleep before re-attempt to bind.

#define APP_PMT_BIND                      0xFF   // Allow binding indefinitely.
#define APP_DENY_BIND                     0x00   // Dis-allow binding indefinitely.
#define APP_PMT_JOIN                      0xFF   // Allow joining indefinitely.
#define APP_DENY_JOIN                     0x00   // Dis-allow joining indefinitely.

#define APP_REPORT_INTERVAL               60     // 60 secs between OTA reports.
#define APP_REPORT_RETRY                  2      //  2 secs between retries when no ack received.
#define APP_RETRY_CNT                     3      // Number of retry attempts.

#ifdef APP_DATA_CNF
#define APP_POLL_INTERVAL                (APP_REPORT_RETRY * 1000 / 2)
#define APP_RESPONSE_POLL                 100
#else
#define APP_POLL_INTERVAL                 0
#define APP_RESPONSE_POLL                 0
#endif

#define APP_STAT_LED                      0
#define APP_DATA_LED                      1

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum {
  appIniting,
  appWaiting,
  appStarting,
  appJoining,
  appJoinWaiting,
  appBinding,
  appBindWaiting,
  appRunning
} AppState;  // Valid appState values.

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

extern uint8 appFlags;

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

void appInit(void);
uint16 appExecHal(void);
uint16 appExecHost(void);

/**************************************************************************************************
*/
#endif
