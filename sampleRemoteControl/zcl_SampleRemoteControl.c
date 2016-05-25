/**************************************************************************************************
  Filename:       zcl_SampleRemoteControl.c
  Revised:        $Date: 2014-06-03 16:29:28 -0700 (Tue, 03 Jun 2014) $
  Revision:       $Revision: 38778 $

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
  This device will be like an On/Off Switch device. This application
  is not intended to be a On/Off Switch device, but will use the device
  description to implement this sample code.

  ----------------------------------------
  Main:
    - SW1: Toggle remote light
    - SW2: Invoke EZMode
    - SW4: Enable/Disable Permit Join
    - SW5: Go to Help screen
  ----------------------------------------
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_SampleRemoteControl.h"
#include "zcl_ezmode.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"


#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
#include "zcl_ota.h"
#include "hal_ota.h"

#endif


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleRemoteControl_TaskID;

uint8 zclSampleRemoteControlSeqNum;

uint8 zclSampleRemoteControl_OnOffSwitchType = ON_OFF_SWITCH_TYPE_TOGGLE;

uint8 zclSampleRemoteControl_OnOffSwitchActions = ON_OFF_SWITCH_ACTIONS_2;   // Toggle -> Toggle

uint8  zclSampleRemoteControl_OnOff = LIGHT_OFF;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
#ifdef ZCL_ON_OFF
afAddrType_t zclSampleRemoteControl_DstAddr;
#endif

#ifdef ZCL_EZMODE
static void zclSampleRemoteControl_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleRemoteControl_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSampleRemoteControl_RegisterEZModeData =
{
  &zclSampleRemoteControl_TaskID,
  SAMPLEREMOTECONTROL_EZMODE_NEXTSTATE_EVT,
  SAMPLEREMOTECONTROL_EZMODE_TIMEOUT_EVT,
  &zclSampleRemoteControlSeqNum,
  zclSampleRemoteControl_EZModeCB
};

// NOT ZLC_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_GEN_ON_OFF
};
#define ZCLSAMPLEREMOTECONTROL_BINDINGLIST   (sizeof(bindingOutClusters)/sizeof(bindingOutClusters[0]))
#endif  // ZLC_EZMODE

// Endpoint to allow SYS_APP_MSGs
static endPointDesc_t SampleRemoteControl_TestEp =
{
  SAMPLEREMOTECONTROL_ENDPOINT,                  // endpoint
  &zclSampleRemoteControl_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

uint8 giSwScreenMode = SW_MAINMODE;   // display the main screen mode first

//static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclSampleRemoteControl_NwkState = DEV_INIT;

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
#define DEVICE_POLL_RATE                 8000   // Poll rate for end device
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleRemoteControl_HandleKeys( byte shift, byte keys );
static void zclSampleRemoteControl_BasicResetCB( void );
static void zclSampleRemoteControl_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleRemoteControl_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp );
static void zclSampleRemoteControl_ProcessIdentifyTimeChange( void );

// app display functions
void zclSampleRemoteControl_LcdDisplayUpdate(void);
void zclSampleRemoteControl_LcdDisplayMainMode(void);
void zclSampleRemoteControl_LcdDisplayHelpMode(void);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleRemoteControl_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleRemoteControl_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleRemoteControl_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleRemoteControl_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleRemoteControl_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleRemoteControl_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleRemoteControl_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
static void zclSampleRemoteControl_ProcessOTAMsgs( zclOTA_CallbackMsg_t* pMsg );
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleRemoteControl_CmdCallbacks =
{
  zclSampleRemoteControl_BasicResetCB,               // Basic Cluster Reset command
  zclSampleRemoteControl_IdentifyCB,                 // Identify command
#ifdef ZCL_EZMODE
  NULL,                                   // Identify EZ-Mode Invoke command
  NULL,                                   // Identify Update Commission State command
#endif
  NULL,                                   // Identify Trigger Effect command
  zclSampleRemoteControl_IdentifyQueryRspCB,         // Identify Query Response command
  NULL,                                   // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                   // Scene Store Request command
  NULL,                                   // Scene Recall Request command
  NULL,                                   // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                   // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                   // Get Event Log command
  NULL,                                   // Publish Event Log command
#endif
  NULL,                                   // RSSI Location command
  NULL                                    // RSSI Location Response command
};


/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sDeviceName[]   = "  SampleRemoteControl";
const char sClearLine[]    = " ";
//const char sSwRC[]      = "SW1: ToggleLight";
const char sSwEZMode[]     = "SW2: EZ-Mode";
const char sSwHelp[]       = "SW5: Help";
const char sCmdSent[]      = "  COMMAND SENT";
#endif

/*********************************************************************
 * @fn          zclSampleRemoteControl_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleRemoteControl_Init( byte task_id )
{
  zclSampleRemoteControl_TaskID = task_id;

#ifdef ZCL_ON_OFF
  // Set destination address to indirect
  zclSampleRemoteControl_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleRemoteControl_DstAddr.endPoint = 0;
  zclSampleRemoteControl_DstAddr.addr.shortAddr = 0;
#endif

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleRemoteControl_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLEREMOTECONTROL_ENDPOINT, &zclSampleRemoteControl_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( SAMPLEREMOTECONTROL_ENDPOINT, SAMPLEREMOTECONTROL_MAX_ATTRIBUTES, zclSampleRemoteControl_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleRemoteControl_TaskID );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleRemoteControl_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleRemoteControl_TaskID );

  // Register for a test endpoint
  afRegister( &SampleRemoteControl_TestEp );

  ZDO_RegisterForZDOMsg( zclSampleRemoteControl_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( zclSampleRemoteControl_TaskID, Match_Desc_rsp );

#ifdef LCD_SUPPORTED
  HalLcdWriteString ( (char *)sDeviceName, HAL_LCD_LINE_3 );
#endif
  
#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
  // Register for callback events from the ZCL OTA
  zclOTA_Register(zclSampleRemoteControl_TaskID);
#endif
  
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclSampleRemoteControl_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleRemoteControl_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSampleRemoteControl_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleRemoteControl_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleRemoteControl_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleRemoteControl_NwkState = (devStates_t)(MSGpkt->hdr.status);

          // now on the network
          if ( (zclSampleRemoteControl_NwkState == DEV_ZB_COORD) ||
               (zclSampleRemoteControl_NwkState == DEV_ROUTER)   ||
               (zclSampleRemoteControl_NwkState == DEV_END_DEVICE) )
          {
#ifndef HOLD_AUTO_START
            giSwScreenMode = SW_MAINMODE;
            zclSampleRemoteControl_LcdDisplayUpdate();
#endif
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif
          }
          break;

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
        case ZCL_OTA_CALLBACK_IND:
          zclSampleRemoteControl_ProcessOTAMsgs( (zclOTA_CallbackMsg_t*)MSGpkt  );
          break;          
#endif
          
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
           
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SAMPLEREMOTECONTROL_IDENTIFY_TIMEOUT_EVT )
  {
    zclSampleRemoteControl_IdentifyTime = 10;
    zclSampleRemoteControl_ProcessIdentifyTimeChange();

    return ( events ^ SAMPLEREMOTECONTROL_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & SAMPLEREMOTECONTROL_MAIN_SCREEN_EVT )
  {
    giSwScreenMode = SW_MAINMODE;

    zclSampleRemoteControl_LcdDisplayUpdate();

#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
    return ( events ^ SAMPLEREMOTECONTROL_MAIN_SCREEN_EVT );
  }

#ifdef ZCL_EZMODE
  if ( events & SAMPLEREMOTECONTROL_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLEREMOTECONTROL_EZMODE_NEXTSTATE_EVT );
  }

  if ( events & SAMPLEREMOTECONTROL_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLEREMOTECONTROL_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE  
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclSampleRemoteControl_HandleKeys( byte shift, byte keys )
{
 
  if ( keys & HAL_KEY_SW_1 )
  {
#ifdef  ZCL_LEVEL_CTRL
    zclGeneral_SendLevelControlMove(SAMPLEREMOTECONTROL_ENDPOINT, &zclSampleRemoteControl_DstAddr, MOVE_MODE_UP, 0x80, FALSE, 0);
#endif
  }

  // invoke EZ-Mode
  if ( keys & HAL_KEY_SW_2 )
  {
#ifdef  ZCL_LEVEL_CTRL
    zclGeneral_SendLevelControlStep(SAMPLEREMOTECONTROL_ENDPOINT, &zclSampleRemoteControl_DstAddr, STEP_MODE_UP, 0x19, 5, FALSE, 0);
#endif
  }
  
  if ( keys & HAL_KEY_SW_3 )
  {
#ifdef  ZCL_LEVEL_CTRL
    zclGeneral_SendLevelControlMove(SAMPLEREMOTECONTROL_ENDPOINT, &zclSampleRemoteControl_DstAddr, MOVE_MODE_DOWN, 0x80, FALSE, 0);
#endif
  }

  // toggle permit join
  if ( keys & HAL_KEY_SW_4 )
  {
#ifdef  ZCL_LEVEL_CTRL
    zclGeneral_SendLevelControlStep(SAMPLEREMOTECONTROL_ENDPOINT, &zclSampleRemoteControl_DstAddr, STEP_MODE_DOWN, 0x19, 5, FALSE, 0);
#endif
  }

  if (  keys & HAL_KEY_SW_5 )
  {
#ifdef ZCL_ON_OFF
    zclGeneral_SendOnOff_CmdToggle( SAMPLEREMOTECONTROL_ENDPOINT, &zclSampleRemoteControl_DstAddr, FALSE, 0 );
#endif
  }
}  

        
 

/*********************************************************************
 * @fn      zclSampleRemoteControl_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleRemoteControl_LcdDisplayUpdate(void)
{
  if ( giSwScreenMode == SW_HELPMODE )
  {
    zclSampleRemoteControl_LcdDisplayHelpMode();
  }
  else
  {
    zclSampleRemoteControl_LcdDisplayMainMode();
  }
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleRemoteControl_LcdDisplayMainMode(void)
{
  if ( zclSampleRemoteControl_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1(0);
  }
  else if ( zclSampleRemoteControl_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1(1);
  }
  else if ( zclSampleRemoteControl_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1(2);
  }

#ifdef LCD_SUPPORTED
  if ( ( zclSampleRemoteControl_NwkState == DEV_ZB_COORD ) ||
       ( zclSampleRemoteControl_NwkState == DEV_ROUTER ) )
  {
    // display help key with permit join status
    if ( gPermitDuration )
    {
      HalLcdWriteString("SW5: Help      *", HAL_LCD_LINE_3);
    }
    else
    {
      HalLcdWriteString("SW5: Help       ", HAL_LCD_LINE_3);
    }
  }
  else
  {
    // display help key
    HalLcdWriteString((char *)sSwHelp, HAL_LCD_LINE_3);
  }
#endif
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleRemoteControl_LcdDisplayHelpMode(void)
{
#ifdef LCD_SUPPORTED
  //HalLcdWriteString( (char *)sSwRC, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleRemoteControl_ProcessIdentifyTimeChange( void )
{
  if ( zclSampleRemoteControl_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleRemoteControl_TaskID, SAMPLEREMOTECONTROL_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleRemoteControl_OnOff )
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    else
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    osal_stop_timerEx( zclSampleRemoteControl_TaskID, SAMPLEREMOTECONTROL_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to  default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleRemoteControl_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network

  // restart device
  //MT_SysCommandProcessing( aProcessCmd );
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSampleRemoteControl_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSampleRemoteControl_IdentifyTime = pCmd->identifyTime;
  zclSampleRemoteControl_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - source address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSampleRemoteControl_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
  (void)pRsp;
#ifdef ZCL_EZMODE
  {
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction ( EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data );
  }
#endif
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleRemoteControl_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleRemoteControl_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleRemoteControl_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleRemoteControl_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleRemoteControl_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleRemoteControl_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleRemoteControl_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclSampleRemoteControl_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleRemoteControl_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleRemoteControl_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleRemoteControl_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleRemoteControl_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleRemoteControl_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleRemoteControl_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleRemoteControl_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return TRUE;
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleRemoteControl_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  // Device is notified of the Default Response command.
  (void)pInMsg;
  return TRUE;
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleRemoteControl_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleRemoteControl_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleRemoteControl_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}
#endif // ZCL_DISCOVER

#if ZCL_EZMODE
/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleRemoteControl_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{

  // Let EZ-Mode know of the Match Descriptor Reponse (same as ActiveEP Response)
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    zclEZMode_ActionData_t data;
    ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free(pRsp);
  }
}

/*********************************************************************
 * @fn      zclSampleRemoteControl_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSampleRemoteControl_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    zclSampleRemoteControl_IdentifyTime = (EZMODE_TIME / 1000);  // convert to seconds
    zclSampleRemoteControl_ProcessIdentifyTimeChange();
  }

  // autoclosing, show what happened (success, cancelled, etc...)
  if( state == EZMODE_STATE_AUTOCLOSE )
  {
#ifdef LCD_SUPPORTED
    pStr = NULL;
    err = pData->sAutoClose.err;
    if ( err == EZMODE_ERR_SUCCESS )
    {
      pStr = "EZMode: Success";
    }
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    if ( pStr )
    {
      if ( giSwScreenMode == SW_MAINMODE )
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclSampleRemoteControl_IdentifyTime = 0;
    zclSampleRemoteControl_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // "EZDst:1234 EP:34"
      osal_memcpy(szLine, "EZDst:", 6);
      zclHA_uint16toa( pData->sFinish.nwkaddr, &szLine[6]);
      osal_memcpy(&szLine[10], " EP:", 4);
      _ltoa( pData->sFinish.ep, (void *)(&szLine[14]), 16 );  // _ltoa NULL terminates
      pStr = szLine;
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giSwScreenMode == SW_MAINMODE )
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif  // LCD_SUPPORTED
    // show main UI screen 3 seconds after binding
    osal_start_timerEx( zclSampleRemoteControl_TaskID, SAMPLEREMOTECONTROL_MAIN_SCREEN_EVT, 3000 );
  }

}

#endif // ZCL_EZMODE

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
/*********************************************************************
 * @fn      zclSampleRemoteControl_ProcessOTAMsgs
 *
 * @brief   Called to process callbacks from the ZCL OTA.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleRemoteControl_ProcessOTAMsgs( zclOTA_CallbackMsg_t* pMsg )
{
  uint8 RxOnIdle;

  switch(pMsg->ota_event)
  {
  case ZCL_OTA_START_CALLBACK:
    if (pMsg->hdr.status == ZSuccess)
    {
      // Speed up the poll rate
      RxOnIdle = TRUE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle );
      NLME_SetPollRate( 2000 );
    }
    break;

  case ZCL_OTA_DL_COMPLETE_CALLBACK:
    if (pMsg->hdr.status == ZSuccess)
    {
      // Reset the CRC Shadow and reboot.  The bootloader will see the
      // CRC shadow has been cleared and switch to the new image
      HalOTAInvRC();
      SystemReset();
    }
    else
    {
      // slow the poll rate back down.
      RxOnIdle = FALSE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle );
      NLME_SetPollRate(DEVICE_POLL_RATE);
    }
    break;

  default:
    break;
  }
}
#endif // defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)

/****************************************************************************
****************************************************************************/


