//Button press appState switch

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
	  break;
    case appRunning:
      changeDoorState();
      break;
  }
}

/*
===========================================================================================================
*/

//Change door state function

static void changeDoorState(void)
{
  srceKey[SRCE_KEY_PRESS] = 0x01; // set srcKey on key press
  #ifdef APP_DATA_CNF
    // Increment the message handle so that the next message is unique.
    zb_SendDataRequest (ZB_BINDING_ADDR, SRCE_KEY_ID, appMsgHandle++,
                        AF_ACK_REQUEST, AF_DEFAULT_RADIUS, SRCE_KEY_SZ, srceKey);

    /* An FFD will always be awake and ready to poll for it.
     */
    appMsgRtry = APP_RETRY_CNT;
    appFlags |= appSendingF;
#else
      zb_SendDataRequest (ZB_BINDING_ADDR, SRCE_KEY_ID, appMsgHandle++,
                          0, AF_DEFAULT_RADIUS, SRCE_KEY_SZ, srceKey);
#ifdef APP_BLINK_LEDS
      appLedBlink (APP_DATA_LED);
#endif
#endif
}

/*
=================================================================================================================
*/

//Send change door state from coordinator

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