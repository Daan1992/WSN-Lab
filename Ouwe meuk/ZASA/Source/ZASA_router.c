/**************************************************************************************************
    Filename:       ZASA_router.c
    Revised:        $Date: 2008-04-10 19:47:13 -0700 (Thu, 10 Apr 2008) $
    Revision:       $Revision: 16806 $

    Description:

    This file contains the main functionality for the Router of the ZACCEL application.


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

static const uint8 srceEP[] = {
  SRCE_ENDPOINT_ID,
  ZASA_PROFILE_ID_LSB,
  ZASA_PROFILE_ID_MSB,
  SRCE_DEVICE_ID_LSB,
  SRCE_DEVICE_ID_MSB,
  SRCE_DEVICE_VERSION,
  SRCE_LATENCY,
  0,
  SRCE_CLUSTER_OUT_CNT,
  SRCE_LIGHT_ID_LSB,
  SRCE_LIGHT_ID_MSB
};


static AppState appState;
#ifdef APP_DATA_CNF
static uint8 appMsgRtry;
#endif
static uint8 appMsgHandle;
static uint8 srceLight[SRCE_LIGHT_SZ];
static uint8 btnState = 0;
//static bool bindingUp = 0;


/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

// Triggered by HAL flags.
static void appExec(void);
static void appBtnPress(void);

// Helper functions for appExec().
static void appJoinFail(void);

// Helper functions for appBtnPress().
static void appStartRequest(void);
static void appToggleJoin(void);

// Triggered by ZACCEL flags or ZACCEL response.
static void appReset(void);
static void appStart(void);
#ifdef APP_DATA_CNF
static void appDataCnf(void);
#endif
static void appSrceBind(void);
static void appSrceData(bool buttonPressed);
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
  else if (halEventFlags & HAL_EVT_TIMER_APP)
  {
    event = HAL_EVT_TIMER_APP;
    appExec();
  }
  else if (halEventFlags & HAL_EVT_TIMER_BTN)
  {
    event = HAL_EVT_TIMER_BTN;
    appBtnPress();
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
  else if (halEventFlags & HAL_EVT_ADC)
  {
    event = HAL_EVT_ADC;
    appSrceData(0);
  }

  /* Since HAL event flags are set at the interrupt level, they must only be cleared within
   * a critical section.
   */
  if (event != HAL_EVT_NONE)
  {
    halIntState_t s;
    HAL_ENTER_CRITICAL_SECTION(s);
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
    if (appState == appJoining)
    {
      appStart();
    }
  }
  else if (zaccelEvtFlags & ZACCEL_BIND_CNF)
  {
    event = ZACCEL_BIND_CNF;

    // Setup an auto-repeating timer to send periodic status reports OTA.
    halTimerSet (HAL_IDX_TIMER_APP, APP_REPORT_INTERVAL, HAL_TIMER_AUTO);
    // But send a report immediately from here to reduce latency of 1st report from a new node.
    appState = appRunning;
    appSrceData(0);
  }
  else if (zaccelEvtFlags & ZACCEL_SEND_DATA_CNF)
  {
    event = ZACCEL_SEND_DATA_CNF;
#ifdef APP_DATA_CNF
    appDataCnf();
#endif
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

    case appJoining:
      // Attempting to join did not succeed within the allowed APP_JOIN_WAIT time.
      appJoinFail();
      break;

    case appBinding:
      halTimerSet (HAL_IDX_TIMER_APP, APP_BIND_WAIT, 0); //wait till rebind
      appState = appBindWaiting;
      break;

    case appBindWaiting: 
      appSrceBind();
      break;

    case appRunning:
      if (appFlags & appSendingF)
      {
#ifdef APP_DATA_CNF
        appDataCnf();
#endif
      }
      else
      {
        appSrceData(0);
      }
      break;

  default:
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
    case appIniting:
      // Do not act on a button press in this state which should be only momentary after powerup.
      break;

    case appWaiting:
      appStartRequest();
      break;

    case appJoining:
    case appJoinWaiting:
    case appBinding:
    case appBindWaiting:
      // Do not act on a button press in these states.
      break;

    case appRunning:
      if(btnState == 0)
      {
        btnState = 1;
        HAL_TURN_ON_RED();
      }
      else
      {
        btnState = 0;
        HAL_TURN_OFF_RED();
      }
      appSrceData(btnState);
      break;

    default:
      break;
  }
}

/**************************************************************************************************
 * @fn          appJoinFail
 *
 * @brief       This function executes the specified behaviour when a join attempt fails.
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
static void appJoinFail(void)
{
  /* MT_SAPI_START_CNF can be missed by the race condition between re-registering the Endpoint
   * with the SAPI after the device does a reset to restore/auto-join and the join success.
   * This check catches the situation when a join succeeds before this host can even register
   * the Endpoint.
   */
  zb_GetDeviceInfo (ZB_INFO_DEV_STATE);
  if (ZACCEL_NWK_CONN)
  {
    appStart();
    return;
  }

  // If attempting to join as an FFD/Router fails, restart again.
  appStartRequest();
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
  uint8 tmp = ZG_DEVICETYPE_ROUTER;
  zb_WriteConfiguration (ZCD_NV_LOGICAL_TYPE, 1, &tmp);
  zb_StartRequest();
  appState = appJoining;

  /* The ZACCEL will try to join indefinitely, so the host is setting a timer in order to take
   * action if the join attempt does not succeed within a reasonable amount of time.
   */
  halTimerSet (HAL_IDX_TIMER_APP, APP_JOIN_TIME, 0);
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

    // A Router-Source device starts blinking the Green LED when not permitting join.
    HAL_TURN_OFF_GRN();

    // Setup the LED blink at 1-Hz.
    halTimerSet (HAL_IDX_TIMER_LED, APP_BLINK_INTERVAL, HAL_TIMER_AUTO);
  }
  else
  {
    appFlags |= appPermittingF;
    zb_PermitJoiningRequest (zaccelNwkAddr, APP_PMT_JOIN);

    // Stop the LED blink.
    halTimerSet (HAL_IDX_TIMER_LED, 0, 0);

    // A Router-Source device sets the Green LED solid on when permitting join.
    HAL_TURN_ON_GRN();
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

  /* No ZigBee Endpoints (not even the Simple Descriptor) are not stored in the ZACCEL NV.
   * Therefore, the host must re-register anytime that the ZACCEL resets.
   */
  zb_SapiAppRegister (srceEP);

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

  zb_AllowBind (APP_DENY_BIND);
  appSrceBind();
}

#ifdef APP_DATA_CNFS
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
    zaccelIndFlags &= ~ZACCEL_SEND_SUCCESS;
    appFlags &= ~appSendingF;
#ifdef APP_BLINK_LEDS
    appLedBlink (APP_DATA_LED);
#endif
  }
  else if ((appMsgRtry == 0) || (--appMsgRtry == 0))
  {
    appFlags &= ~appSendingF;
    appSrceBind();  // When a message confirmation fails, attempt to re-bind. TODO CHECK WHICH CLUSTER
   }
  else
  {
    // Do not increment the message handle since this is the same message, only a re-try.
    zb_SendDataRequest (ZB_BINDING_ADDR, SRCE_REPORT_ID, appMsgHandle,
                        AF_ACK_REQUEST, AF_DEFAULT_RADIUS, SRCE_REPORT_SZ, srceLight);
  }
}
#endif

/**************************************************************************************************
 * @fn          appSrceBind
 *
 * @brief       This function is the host application to request an Endpoint binding.
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
  /* IEEE address of the device to establish the binding with. Set the destIEEE to NULL
   * in order to bind with any other device that is in the Allow Binding Mode.
   */
  uint8 destIEEE[Z_EXTADDR_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  zb_BindDevice (TRUE, SRCE_LIGHT_ID, destIEEE);
  halTimerSet (HAL_IDX_TIMER_APP, APP_BIND_TIME, 0);
  appState = appBinding;
}

/**************************************************************************************************
 * @fn          appSrceData
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
static void appSrceData(bool buttonPressed)
{
  uint16 tmp;
  switch(appFlags & (appLuxF | appIdleF)){
	case appLuxF:
      appFlags &= ~appLuxF;
      srceLight[SRCE_LIGHT_BTNSTATE] = btnState;
      tmp = halAdcVal;
      srceLight[SRCE_LIGHT_VALUE] = (uint8)tmp;
      
    #ifdef APP_DATA_CNF
      // Increment the message handle so that the next message is unique.
      zb_SendDataRequest (ZB_BINDING_ADDR, SRCE_LIGHT_ID, appMsgHandle++,
                          AF_ACK_REQUEST, AF_DEFAULT_RADIUS, SRCE_LIGHT_SZ, srceLight);
s
      /* An FFD will always be awake and ready to poll for it.
       */
      appMsgRtry = APP_RETRY_CNT;
      appFlags |= appSendingF;
#else
      zb_SendDataRequest (ZB_BINDING_ADDR, SRCE_LIGHT_ID, appMsgHandle++,
                          0, AF_DEFAULT_RADIUS, SRCE_LIGHT_SZ, srceLight);
#ifdef APP_BLINK_LEDS
      appLedBlink (APP_DATA_LED);
#endif
#endif
      break;
	  
    case appIdleF:
        if(buttonPressed ==1){
            appFlags |= appLuxF;
            halLightSensorRead();
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
        HAL_TURN_ON_GRN();
        halDelay (APP_BLINK_ON_TIME, TRUE);
        HAL_TURN_OFF_GRN();
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
