/**************************************************************************************************
    Filename:       ZASA_coord.c
    Revised:        $Date: 2008-04-10 19:47:13 -0700 (Thu, 10 Apr 2008) $
    Revision:       $Revision: 16806 $

    Description:

    This file contains the main functionality for the Coordinator of the ZACCEL application.


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
#include "ZASA.h"
#include "sapi.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

uint8 appFlags = appIdleF;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static const uint8 sinkEP[] = {
  SINK_ENDPOINT_ID,
  ZASA_PROFILE_ID_LSB,
  ZASA_PROFILE_ID_MSB,
  SINK_DEVICE_ID_LSB,
  SINK_DEVICE_ID_MSB,
  SINK_DEVICE_VERSION,
  SINK_LATENCY,
  SINK_CLUSTER_IN_CNT,
  SRCE_LIGHT_ID_LSB,
  SRCE_LIGHT_ID_MSB,
  SRCE_KEY_ID_LSB,
  SRCE_KEY_ID_MSB,
  SINK_CLUSTER_OUT_CNT,
  SRCE_DOOR_ID_LSB,
  SRCE_DOOR_ID_MSB
};

static AppState appState;
static uint8 appMsgRtry;
static uint8 appMsgHandle;
static uint8 srceDoor[SRCE_DOOR_SZ];
static bool bindingUp = 0;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

// Triggered by HAL flags.
static void appExec(void);
static void appBtnPress(void);

// Helper functions for appBtnPress().
static void appStartRequest(void);
static void appToggleJoin(void);

// Triggered by ZACCEL flags or ZACCEL response.
static void appReset(void);
static void appStart(void);
static void appDataCnf(void);
static void appSrceBind(void);
static void appSendDoorState(void);
static void appSinkData(void);
#ifdef APP_BLINK_LEDS
static void appLedBlink(uint8 led);
#endif

/**************************************************************************************************
 * @fn          appInit
 *
 * @brief       This function is the host application initialization.
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
void appInit(void)
{
#ifdef APP_BLINK_LEDS
  // Setup the LED blink at 1-Hz.
  halTimerSet (HAL_IDX_TIMER_LED, APP_BLINK_INTERVAL, HAL_TIMER_AUTO);
#endif
  
  HAL_ENABLE_INTERRUPTS();

  appState = appIniting;
}

/**************************************************************************************************
 * @fn          appExecHal
 *
 * @brief       This function is the ZASA executive for HAL events.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if a HAL event was processed; FALSE otherwise.
 **************************************************************************************************
 */
uint16 appExecHal(void)
{
  uint16 event = HAL_EVT_NONE;

  if (halEventFlags & HAL_EVT_TIMER_LED)
  {
    event = HAL_EVT_TIMER_LED;
#ifdef APP_BLINK_LEDS
    appLedBlink (APP_STAT_LED);
#endif
  }
  else if (halEventFlags & HAL_EVT_TIMER_BTN)
  {
    event = HAL_EVT_TIMER_BTN;
    appBtnPress();
  }
  else if (halEventFlags & HAL_EVT_TIMER_APP)
  {
    event = HAL_EVT_TIMER_APP;
    appExec();
  }
  else if (halEventFlags & HAL_EVT_BTN_PRESS)
  {
    event = HAL_EVT_BTN_PRESS;
    halTimerSet (HAL_IDX_TIMER_BTN, APP_BTN_INTERVAL, 0);

    // Immediately turn of LEDs when user starts a join process.
    if (appState == appWaiting)
    {
      // Stop the LED blink during joining.
      halTimerSet (HAL_IDX_TIMER_LED, 0, 0);
      HAL_TURN_OFF_GRN();
      HAL_TURN_OFF_RED();
    }
  }
#ifdef HOST_MT
  else if (halEventFlags & HAL_EVT_MT_RX_RDY)
  {
    if (mtRx())
    {
      event = HAL_EVT_MT_RX_RDY;
    }
  }
#endif

  /* Since HAL event flags are set at the interrupt level, they must only be cleared within
   * a critical section.
   */
  if (event != HAL_EVT_NONE)
  {
    halIntState_t s;
    HAL_ENTER_CRITICAL_SECTION(s);
    //halEventFlags ^= event;
    event = halEventFlags & event;
    halEventFlags ^= event;
    HAL_EXIT_CRITICAL_SECTION(s);
    return TRUE;
  }

  return FALSE;
}

/**************************************************************************************************
 * @fn          appExecHost
 *
 * @brief       This function is the ZASA executive for ZACCEL events.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if a ZACCEL event was processed; FALSE otherwise.
 **************************************************************************************************
 */
uint16 appExecHost(void)
{
  uint16 event = ZACCEL_EVT_NONE;

  zaccelPoll();

  if (zaccelEvtFlags & ZACCEL_SYS_RESET_IND)
  {
    event = ZACCEL_SYS_RESET_IND;
    appReset();
  }
  else if (zaccelEvtFlags & ZACCEL_START_CNF)
  {
    event = ZACCEL_START_CNF;
    if (appState == appStarting)
    {
      appStart();
    }
  }
  else if (zaccelEvtFlags & ZACCEL_BIND_CNF)
  {
    event = ZACCEL_BIND_CNF;
    // Setup an auto-repeating timer to send periodic status reports OTA.
    halTimerSet (HAL_IDX_TIMER_APP, APP_REPORT_INTERVAL/4, HAL_TIMER_AUTO); 
    appState = appRunning;
    bindingUp = 1;
    halUARTWrite(HAL_PORT_MT, "BINDING UP\r\n", 12);
  }
  else if (zaccelEvtFlags & ZACCEL_SEND_DATA_CNF)
  {
    event = ZACCEL_SEND_DATA_CNF;
  }
  else if (zaccelEvtFlags & ZACCEL_RCV_DATA_IND)
  {
    event = ZACCEL_RCV_DATA_IND;
    appSinkData();
  }

  if (event != ZACCEL_EVT_NONE)
  {
    zaccelEvtFlags ^= event;
    return TRUE;
  }

  return FALSE;
}

/**************************************************************************************************
 * @fn          appExec
 *
 * @brief       This function is the ZASA executive run by a periodic timer event.
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
static void appExec(void)
{
  switch (appState)
  {
    case appIniting:
    case appWaiting:
      // Not expected in this state, so just stop the timer in case it is auto-repeating.
      halTimerSet (HAL_IDX_TIMER_APP, 0, 0);
      break;
      
    case appBinding:
      halUARTWrite(HAL_PORT_MT, "APPSTATE: WAITING\r\n", 19);
      halTimerSet (HAL_IDX_TIMER_APP, APP_BIND_WAIT, 0); 
      appState = appBindWaiting;
      break;

    case appBindWaiting: 
      halUARTWrite(HAL_PORT_MT, "APPBINDWAITINGSTATE\r\n", 21);
      appSrceBind();
      break;
  
    case appRunning:
      if (bindingUp == 1) {
        if (appFlags & appSendingF)
        {
          appDataCnf();
        }
        else
        {
          appSendDoorState();
        }
      }
      break;
  }
}

/**************************************************************************************************
 * @fn          appBtnPress
 *
 * @brief       This function acts on a button press.
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
static void appBtnPress(void)
{
  switch (appState)
  {
    case appWaiting:
      appStartRequest();
      break;

    case appRunning:
      appSendDoorState();
      break;

    default:
      break;
  }
}

/**************************************************************************************************
 * @fn          appStartRequest
 *
 * @brief       This function acts on a button press.
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
static void appStartRequest(void)
{
  uint8 tmp = ZG_DEVICETYPE_COORDINATOR;
  zb_WriteConfiguration (ZCD_NV_LOGICAL_TYPE, 1, &tmp);
  zb_StartRequest();
  appState = appStarting;
}

/**************************************************************************************************
 * @fn          appToggleJoin
 *
 * @brief       This function acts on a request to toggle the permit join state of the ZACCEL.
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
static void appToggleJoin(void)
{
  // Toggle the "permit joining" of the ZACCEL and set the LED behavior accordingly.
  if (appFlags & appPermittingF)
  {
    appFlags &= ~appPermittingF;
    zb_PermitJoiningRequest (zaccelNwkAddr, APP_DENY_JOIN);

    // A Coordinator-Sink device starts blinking the Red LED when not permitting join.
    HAL_TURN_OFF_RED();

    // Setup the LED blink at 1-Hz.
    halTimerSet (HAL_IDX_TIMER_LED, APP_BLINK_INTERVAL, HAL_TIMER_AUTO);
  }
  else
  {
    appFlags |= appPermittingF;
    zb_PermitJoiningRequest (zaccelNwkAddr, APP_PMT_JOIN);

    // Stop the LED blink.
    halTimerSet (HAL_IDX_TIMER_LED, 0, 0);

    // A Coordinator-Sink device sets the Red LED solid on when permitting join.
    HAL_TURN_ON_RED();
  }
}

/**************************************************************************************************
 * @fn          appReset
 *
 * @brief       This function is the host action on a ZACCEL reset.
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
static void appReset(void)
{
  // No previously received indication flag can be valid after the ZACCEL resets.
  zaccelIndFlags = ZACCEL_STATUS_CLEAR;

  /* No ZigBee Endpoints (not even the Simple Descriptor) are stored in the ZACCEL NV.
   * Therefore, the host must re-register anytime that the ZACCEL resets.
   */
  zb_SapiAppRegister (sinkEP);

  switch (appState)
  {
    case appIniting:
      {
        // Reset Network NV items.
        uint8 val = ZCD_STARTOPT_CLEAR_CONFIG;
        zb_WriteConfiguration (ZCD_NV_STARTUP_OPTION, 1, &val);
        zb_SystemReset();

        appState = appWaiting;
      }
      break;

    case appWaiting:
      // The last step of the appIniting above was to reset the ZACCEL.
      {
        // Configure the Host Application-specific defaults from ZASA.cfg into the ZACCEL.
        uint16 val16 = ZACCEL_NV_PANID;
        zb_WriteConfiguration (ZCD_NV_PANID, 2, &val16);
        uint32 val32 = ZACCEL_NV_CHANLIST;
        zb_WriteConfiguration (ZCD_NV_CHANLIST, 4, &val32);
      }
      break;

    default:
      break;
  }
}

/**************************************************************************************************
 * @fn          appStart
 *
 * @brief       This function is the host application registration with the ZACCEL SAPI.
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
static void appStart(void)
{
  // And now allow the ZACCEL slave to NV restore and auto re-start on resets.
  uint8 val = ZCD_STARTOPT_AUTO_START;
  zb_WriteConfiguration (ZCD_NV_STARTUP_OPTION, 1, &val);

  zb_GetDeviceInfo (ZB_INFO_DEV_STATE);
  zb_GetDeviceInfo (ZB_INFO_SHORT_ADDR);

  appFlags &= ~appPermittingF;
  appToggleJoin();  // This will set permit joining allowed and set LEDs accordingly.
  zb_AllowBind (APP_PMT_BIND);
  appState = appRunning;
}

/**************************************************************************************************
 * @fn          appDataCnf
 *
 * @brief       This function is the host application action upon receiving a message confirmation.
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
static void appDataCnf(void)
{
  if (zaccelIndFlags & ZACCEL_SEND_SUCCESS)
  {
    halUARTWrite(HAL_PORT_MT, "DOOR STATE CONFIRMED\r\n", 22);
    zaccelIndFlags &= ~ZACCEL_SEND_SUCCESS;
    appFlags &= ~appSendingF;
  }
  else if ((appMsgRtry == 0) || (--appMsgRtry == 0))
  {
    halUARTWrite(HAL_PORT_MT, "SEND FAILED, REBINDING\r\n", 24);
    appFlags &= ~appSendingF;
    appSrceBind(); 
   }
  else
  {
    halUARTWrite(HAL_PORT_MT, "SEND FAILED, RETRYING\r\n", 23);
    // Do not increment the message handle since this is the same message, only a re-try.
    zb_SendDataRequest (ZB_BINDING_ADDR, SRCE_DOOR_ID, appMsgHandle,
                        AF_ACK_REQUEST, AF_DEFAULT_RADIUS, SRCE_DOOR_SZ, srceDoor);
  }
}

/**************************************************************************************************
 * @fn          appSrceBind
 *
 * @brief       This function is the host application to request an Endpoint binding to send Door State.
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
static void appSrceBind(void)
{
  halUARTWrite(HAL_PORT_MT, "INITIATING BIND (DOOR)\r\n", 24);
  // IEEE address of the device to establish the binding with. Set the destIEEE to NULL
  // in order to bind with any other device that is in the Allow Binding Mode.
  uint8 destIEEE[Z_EXTADDR_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  zb_BindDevice (TRUE, SRCE_DOOR_ID, destIEEE);
  halTimerSet (HAL_IDX_TIMER_APP, APP_BIND_TIME, 0);
  appState = appBinding;
}
/**************************************************************************************************
 * @fn          appSendDoorState
 *
 * @brief       This function sends the state of the door (open/close) to the enddevice. 
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
static void appSendDoorState(void)
{
  P2SEL = 0x00;
  if ((P2IN & BV(2)) == BV(2)){
    srceDoor[SRCE_DOOR_STATE] = 0x00;
  }
  else{
    srceDoor[SRCE_DOOR_STATE] = 0x01;
  }
  
  // Increment the message handle so that the next message is unique.
  zb_SendDataRequest (ZB_BINDING_ADDR, SRCE_DOOR_ID, appMsgHandle++,
                      AF_ACK_REQUEST, AF_DEFAULT_RADIUS, SRCE_DOOR_SZ, srceDoor);
  halUARTWrite(HAL_PORT_MT, "DOOR STATE SEND TO KEY\r\n", 24);
   
  appMsgRtry = APP_RETRY_CNT;
  appFlags |= appSendingF;
}

/**************************************************************************************************
 * @fn          appSinkData
 *
 * @brief       This function is the host application to process received data.
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
static void appSinkData(void){
  halUARTWrite(HAL_PORT_MT, "RECEIVING DATA...\r\n", 19);
  switch (zaccelDataCmd){
    case SRCE_LIGHT_ID:
      if (zaccelDataLen != SRCE_LIGHT_SZ){
        break;
      }
      halUARTWrite(HAL_PORT_MT, "LIGHT VALUE RECEIVED: \r\n", 24);
      if (zaccelDataBuf[SRCE_LIGHT_BTNSTATE] == 0){
        HAL_TURN_OFF_GRN();
        halUARTWrite(HAL_PORT_MT, "MEASURING LIGHT OFF\r\n", 21);
      }
      else{
        halUARTWrite(HAL_PORT_MT, "MEASURING LIGHT ON\r\n", 20);
        if (zaccelDataBuf[SRCE_LIGHT_VALUE] <0x0F){
          HAL_TURN_ON_GRN();
          halUARTWrite(HAL_PORT_MT, "LIGHTNESS\r\n", 11);
        }
        else{
          HAL_TURN_OFF_GRN();
          halUARTWrite(HAL_PORT_MT, "DARKNESS\r\n", 10);
        }
      }
      break;
    case SRCE_KEY_ID:
      if (zaccelDataLen != SRCE_KEY_SZ)
        break;
      halUARTWrite(HAL_PORT_MT, "KEY RECEIVED, TOGGLING DOOR\r\n", 29);
      P4OUT ^= BV(3); 
      if (bindingUp == 0){
        appSrceBind();
      }
      else{
        appSendDoorState();
      }
      break;
  }
}

#ifdef APP_BLINK_LEDS
/**************************************************************************************************
 * @fn          appLedBlink
 *
 * @brief       Blink the LED specified.
 *
 * input parameters
 *
 * @param       led - Which LED to control: Status or Data.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void appLedBlink(uint8 led)
{
  switch (led)
  {
    case APP_STAT_LED:
      if (ZACCEL_NWK_CONN)
      {
        HAL_TURN_ON_RED();
        halDelay (APP_BLINK_ON_TIME, TRUE);
        HAL_TURN_OFF_RED();
      }
      else
      {
        HAL_TURN_ON_GRN();
        HAL_TURN_ON_RED();
        halDelay (APP_BLINK_ON_TIME, TRUE);
        HAL_TURN_OFF_GRN();
        HAL_TURN_OFF_RED();
      }
      break;

    default:
      break;
  }
}
#endif
