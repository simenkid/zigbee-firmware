/**************************************************************************************************
  Filename:       zcl_SampleSmartPower.c
  Revised:        $Date: 2014-07-01 22:24:24 -0700 (Tue, 01 Jul 2014) $
  Revision:       $Revision: 39317 $


  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
  This application implements a ZigBee HA 1.2 Light. It can be configured as an
  On/Off light, or as a dimmable light. The following flags must be defined in
  the compiler's pre-defined symbols.

  ZCL_ON_OFF
  ZCL_LEVEL_CTRL    (only if dimming functionality desired)
  HOLD_AUTO_START
  ZCL_EZMODE

  This device supports all mandatory and optional commands/attributes for the
  OnOff (0x0006) and LevelControl (0x0008) clusters.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Toggle local light
    - SW2: Invoke EZMode
    - SW4: Enable/Disable local permit join
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
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_diagnostic.h"

#include "zcl_SampleSmartPower.h"

#include "onboard.h"

/* HAL */
#include "hal_adc.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_relay.h"

#if ( defined (ZGP_DEVICE_TARGET) || defined (ZGP_DEVICE_TARGETPLUS) \
      || defined (ZGP_DEVICE_COMBO) || defined (ZGP_DEVICE_COMBO_MIN) )
#include "zgp_translationtable.h"
  #if (SUPPORTED_S_FEATURE(SUPP_ZGP_FEATURE_TRANSLATION_TABLE))
    #define ZGP_AUTO_TT
  #endif
#endif

#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
#include "math.h"
#include "hal_timer.h"
#endif

#include "NLMEDE.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#if (defined HAL_BOARD_ZLIGHT)
#define LEVEL_MAX                 0xFE
#define LEVEL_MIN                 0x0
#define GAMMA_VALUE               2
#define PWM_FULL_DUTY_CYCLE       1000
#elif (defined HAL_PWM)  
#define LEVEL_MAX                 0xFE
#define LEVEL_MIN                 0x0
#define GAMMA_VALUE               2
#define PWM_FULL_DUTY_CYCLE       100
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleSmartPower_TaskID;
uint8 zclSampleSmartPowerSeqNum;
//uint8   relayonoff;
uint16 CurrentValue = 0.0;
uint8 Cal5V = 0;
bool change = 0;

uint16 PowerSensor_MeasuredPowerValue;
int16 PowerSensor_MeasuredCurrentValue;
uint16 PowerSensor_MeasuredValue;
/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleSmartPower_DstAddr;

#ifdef ZCL_EZMODE
static void zclSampleSmartPower_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleSmartPower_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );


// register EZ-Mode with task information (timeout events, callback, etc...)
static const zclEZMode_RegisterData_t zclSampleSmartPower_RegisterEZModeData =
{
  &zclSampleSmartPower_TaskID,
  SAMPLELIGHT_EZMODE_NEXTSTATE_EVT,
  SAMPLELIGHT_EZMODE_TIMEOUT_EVT,
  &zclSampleSmartPowerSeqNum,
  zclSampleSmartPower_EZModeCB
};

#else
uint16 bindingInClusters[] =
{
  ZCL_CLUSTER_ID_GEN_ON_OFF
#ifdef ZCL_LEVEL_CTRL
  , ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL
#endif
};
#define ZCLSAMPLELIGHT_BINDINGLIST (sizeof(bindingInClusters) / sizeof(bindingInClusters[0]))

#endif  // ZCL_EZMODE

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t SampleSmartPower_TestEp =
{
  SAMPLELIGHT_ENDPOINT,
  &zclSampleSmartPower_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

uint8 giLightScreenMode = LIGHT_MAINMODE;   // display the main screen mode first

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclSampleSmartPower_NwkState = DEV_INIT;

#if ZCL_LEVEL_CTRL
uint8 zclSampleSmartPower_WithOnOff;       // set to TRUE if state machine should set light on/off
uint8 zclSampleSmartPower_NewLevel;        // new level when done moving
bool  zclSampleSmartPower_NewLevelUp;      // is direction to new level up or down?
int32 zclSampleSmartPower_CurrentLevel32;  // current level, fixed point (e.g. 192.456)
int32 zclSampleSmartPower_Rate32;          // rate in units, fixed point (e.g. 16.123)
uint8 zclSampleSmartPower_LevelLastLevel;  // to save the Current Level before the light was turned OFF
#endif


afAddrType_t zclSampleSmartPower_DstAddr;
uint8 zclSampleSmartPowerSeqNum;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void sendData(void);

static void readPower(void);
static int16 ensureVdd(void);
static uint16 sampleAdcValue( uint8 channel );
static int16 convertToQFormat (int16 num, int16 maxNum);
static int16 convertToInt (int16 num, int32 amp);
static int q15Mul(int a, int b);

static void zclSampleSmartPower_HandleKeys( byte shift, byte keys );
static void zclSampleSmartPower_BasicResetCB( void );

static void zclSampleSmartPower_OnOffCB( uint8 cmd );




#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
void zclSampleSmartPower_UpdateLampLevel( uint8 level );
#endif

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleSmartPower_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleSmartPower_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleSmartPower_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleSmartPower_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleSmartPower_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleSmartPower_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleSmartPower_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sDeviceName[]   = "  Sample Light";
const char sClearLine[]    = " ";
const char sSwLight[]      = "SW1: ToggleLight";  // 16 chars max
const char sSwEZMode[]     = "SW2: EZ-Mode";
char sSwHelp[]             = "SW5: Help       ";  // last character is * if NWK open
const char sLightOn[]      = "    LIGHT ON ";
const char sLightOff[]     = "    LIGHT OFF";
 #if ZCL_LEVEL_CTRL
 char sLightLevel[]        = "    LEVEL ###"; // displays level 1-254
 #endif
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleSmartPower_CmdCallbacks =
{
  zclSampleSmartPower_BasicResetCB,            // Basic Cluster Reset command
  NULL,              // Identify command
#ifdef ZCL_EZMODE
  NULL,                                   // Identify EZ-Mode Invoke command
  NULL,                                   // Identify Update Commission State command
#endif
  NULL,                                   // Identify Trigger Effect command
  NULL,      // Identify Query Response command
  zclSampleSmartPower_OnOffCB,                 // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL, // Level Control Move to Level command
  NULL,        // Level Control Move command
  NULL,        // Level Control Step command
  NULL,        // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                  // Scene Store Request command
  NULL,                                  // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSampleSmartPower_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleSmartPower_Init( byte task_id )
{
  
  zclSampleSmartPower_TaskID = task_id;

  // Set destination address to indirect
  zclSampleSmartPower_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleSmartPower_DstAddr.endPoint = 0;
  zclSampleSmartPower_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleSmartPower_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLELIGHT_ENDPOINT, &zclSampleSmartPower_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( SAMPLELIGHT_ENDPOINT, zclSampleSmartPower_NumAttributes, zclSampleSmartPower_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleSmartPower_TaskID );

#ifdef ZCL_DISCOVER
  // Register the application's command list
  zcl_registerCmdList( SAMPLELIGHT_ENDPOINT, zclCmdsArraySize, zclSampleSmartPower_Cmds );
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleSmartPower_TaskID );

  // Register for a test endpoint
  afRegister( &SampleSmartPower_TestEp );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleSmartPower_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif


#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)   
  HalTimer1Init( 0 );
  halTimer1SetChannelDuty( WHITE_LED, 0 );
  halTimer1SetChannelDuty( RED_LED, 0 );
  halTimer1SetChannelDuty( BLUE_LED, 0 );
  halTimer1SetChannelDuty( GREEN_LED, 0 );

  // find if we are already on a network from NV_RESTORE
  uint8 state;
  NLME_GetRequest( nwkNwkState, 0, &state );
  
  if ( state < NWK_ENDDEVICE ) 
  {
    // Start EZMode on Start up to avoid button press
    osal_start_timerEx( zclSampleSmartPower_TaskID, SAMPLELIGHT_START_EZMODE_EVT, 500 );
  }
#if ZCL_LEVEL_CTRL
  zclSampleSmartPower_DefaultMove();
#endif  
#endif // #if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)

#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( SAMPLELIGHT_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );

  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif
  /*
  P0DIR &= ~BV(0);
  P0SEL |= BV(0);
  APCFG |= BV(0);
  P0DIR &= ~BV(4);
  P0SEL |= BV(4);
  APCFG |= BV(4);
  */
  //HalRelayInit();

#ifdef LCD_SUPPORTED
  HalLcdWriteString ( (char *)sDeviceName, HAL_LCD_LINE_3 );
#endif  // LCD_SUPPORTED

#ifdef ZGP_AUTO_TT
  zgpTranslationTable_RegisterEP ( &zclSampleSmartPower_SimpleDesc );
#endif
  //relayonoff = RELAY_OFF;
  zclSampleSmartPowerSeqNum = 0;
  osal_set_event(zclSampleSmartPower_TaskID, CURRENT_EVT);
  //osal_set_event(zclSampleSmartPower_TaskID, RELAY_TEST_EVT);
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
uint16 zclSampleSmartPower_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleSmartPower_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSampleSmartPower_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleSmartPower_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleSmartPower_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleSmartPower_NwkState = (devStates_t)(MSGpkt->hdr.status);

          // now on the network
          if ( (zclSampleSmartPower_NwkState == DEV_ZB_COORD) ||
               (zclSampleSmartPower_NwkState == DEV_ROUTER)   ||
               (zclSampleSmartPower_NwkState == DEV_END_DEVICE) )
          {
            giLightScreenMode = LIGHT_MAINMODE;
            
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif // ZCL_EZMODE
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SAMPLELIGHT_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSampleSmartPower_IdentifyTime > 0 )
      zclSampleSmartPower_IdentifyTime--;
    

    return ( events ^ SAMPLELIGHT_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & SAMPLELIGHT_MAIN_SCREEN_EVT )
  {
    giLightScreenMode = LIGHT_MAINMODE;
    

    return ( events ^ SAMPLELIGHT_MAIN_SCREEN_EVT );
  }

#ifdef ZCL_EZMODE
#if (defined HAL_BOARD_ZLIGHT)
  // event to start EZMode on startup with a delay 
  if ( events & SAMPLELIGHT_START_EZMODE_EVT )
  {
    // Invoke EZ-Mode
    zclEZMode_InvokeData_t ezModeData;

    // Invoke EZ-Mode
    ezModeData.endpoint = SAMPLELIGHT_ENDPOINT; // endpoint on which to invoke EZ-Mode
    if ( (zclSampleSmartPower_NwkState == DEV_ZB_COORD) ||
         (zclSampleSmartPower_NwkState == DEV_ROUTER)   ||
         (zclSampleSmartPower_NwkState == DEV_END_DEVICE) )
    {
      ezModeData.onNetwork = TRUE;      // node is already on the network
    }
    else
    {
      ezModeData.onNetwork = FALSE;     // node is not yet on the network
    }
    ezModeData.initiator = FALSE;          // OnOffLight is a target
    ezModeData.numActiveOutClusters = 0;
    ezModeData.pActiveOutClusterIDs = NULL;
    ezModeData.numActiveInClusters = 0;
    ezModeData.pActiveOutClusterIDs = NULL;
    zcl_InvokeEZMode( &ezModeData );
  
    return ( events ^ SAMPLELIGHT_START_EZMODE_EVT );
  }
#endif // #if (defined HAL_BOARD_ZLIGHT)
  
  // going on to next state
  if ( events & SAMPLELIGHT_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLELIGHT_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & SAMPLELIGHT_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLELIGHT_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE
  
 if ( events & CURRENT_EVT )
  {
    
    readPower();
    //sendData();
    osal_start_timerEx( zclSampleSmartPower_TaskID, CURRENT_EVT, 2000 );
    return ( events ^ CURRENT_EVT );
  }
  
  
if ( events & RELAY_TEST_EVT )
  {
    //uint16 data = 0;
    PowerSensor_MeasuredPowerValue = 0;
    //PowerSensor_MeasuredPowerValue = HalAdcRead(HAL_ADC_CHANNEL_4, HAL_ADC_RESOLUTION_14);
    //PowerSensor_MeasuredPowerValue = data;
    
  
  uint16 adcVdd3, adcChannel, voltage;
  uint32 vdd;
  HalAdcSetReference(HAL_ADC_REF_125V);
  adcVdd3 = HalAdcRead(HAL_ADC_CHN_VDD3, HAL_ADC_RESOLUTION_14);
  vdd = ((uint32)(adcVdd3)) * 42;  // (115000 * 3 / 8191), ADC internal voltage for CC253x is 1.15V

  HalAdcSetReference( HAL_ADC_REF_AVDD );
  adcChannel = HalAdcRead( HAL_ADC_CHANNEL_4, HAL_ADC_RESOLUTION_14);
  PowerSensor_MeasuredPowerValue = (uint16)( ((uint32)adcChannel * vdd) / 81910 );
    
    
    /*
  uint16 adcVdd3, adcChannel, vdd;
  
  HalAdcSetReference(HAL_ADC_REF_125V);
  adcVdd3 = HalAdcRead(HAL_ADC_CHN_VDD3, HAL_ADC_RESOLUTION_8);
  vdd = (adcVdd3) * 27;  // (1150 * 3 / 127), ADC internal voltage for CC254x is 1.24V

  HalAdcSetReference( HAL_ADC_REF_AVDD );
  adcChannel = HalAdcRead( HAL_ADC_CHANNEL_4, HAL_ADC_RESOLUTION_8);
  PowerSensor_MeasuredPowerValue = (uint16)( ((uint32)adcChannel * (uint32)vdd) / 127 );
  */
    /*
  if ( change )
    {
      change = 0;
      HalRelayOff();
    }
    else
    {
      change = 1;
      HalRelayOn();
    }
  */
   osal_start_timerEx( zclSampleSmartPower_TaskID, RELAY_TEST_EVT, 1000 );
  
  return ( events ^ RELAY_TEST_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleSmartPower_HandleKeys
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
static void zclSampleSmartPower_HandleKeys( byte shift, byte keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    /*
    giLightScreenMode = LIGHT_MAINMODE;

    // toggle local light immediately
    zclSampleSmartPower_OnOff = zclSampleSmartPower_OnOff ? LIGHT_OFF : LIGHT_ON;
#ifdef ZCL_LEVEL_CTRL
    zclSampleSmartPower_LevelCurrentLevel = zclSampleSmartPower_OnOff ? zclSampleSmartPower_LevelOnLevel : ATTR_LEVEL_MIN_LEVEL;
#endif
    */
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /*
#if (defined HAL_BOARD_ZLIGHT) 
    
    zclSampleSmartPower_BasicResetCB();

#else    
    
    giLightScreenMode = LIGHT_MAINMODE;

#ifdef ZCL_EZMODE
    {
      // Invoke EZ-Mode
      zclEZMode_InvokeData_t ezModeData;

      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLELIGHT_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( (zclSampleSmartPower_NwkState == DEV_ZB_COORD) ||
          (zclSampleSmartPower_NwkState == DEV_ROUTER)   ||
            (zclSampleSmartPower_NwkState == DEV_END_DEVICE) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = FALSE;          // OnOffLight is a target
      ezModeData.numActiveOutClusters = 0;
      ezModeData.pActiveOutClusterIDs = NULL;
      ezModeData.numActiveInClusters = 0;
      ezModeData.pActiveOutClusterIDs = NULL;
      zcl_InvokeEZMode( &ezModeData );
    }

#else // NOT EZ-Mode
    {
      zAddrType_t dstAddr;
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request, this bind request will
      // only use a cluster list that is important to binding.
      dstAddr.addrMode = afAddr16Bit;
      dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                           SAMPLELIGHT_ENDPOINT,
                           ZCL_HA_PROFILE_ID,
                           ZCLSAMPLELIGHT_BINDINGLIST, bindingInClusters,
                           0, NULL,   // No Outgoing clusters to bind
                           TRUE );
    }
#endif // ZCL_EZMODE
#endif // HAL_BOARD_ZLIGHT     
    */
  }

  if ( keys & HAL_KEY_SW_3 )
  {
   
    
    //HalI2CWrite(I2C_Address, Register_Address, write_byte, &write_data);
    
    /*
    zclSampleSmartPower_NewLevelUp = zclSampleSmartPower_OnOff ? TRUE : FALSE;
    if(zclSampleSmartPower_NewLevelUp)
      HalLcdWriteString("1", HAL_LCD_LINE_1);
    else
      HalLcdWriteString("0", HAL_LCD_LINE_1);
    */
  }
  
  if ( keys & HAL_KEY_SW_4 )
  {
    /*
    giLightScreenMode = LIGHT_MAINMODE;

    if ( ( zclSampleSmartPower_NwkState == DEV_ZB_COORD ) ||
          ( zclSampleSmartPower_NwkState == DEV_ROUTER ) )
    {
      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;
      NLME_PermitJoiningRequest( gPermitDuration );
    }
    */
  }

  // Shift F5 does a Basic Reset (factory defaults)
  if ( shift && ( keys & HAL_KEY_SW_5 ) )
  {
    zclSampleSmartPower_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    //giLightScreenMode = giLightScreenMode ? LIGHT_MAINMODE : LIGHT_HELPMODE;
  }

}

/*********************************************************************
 * @fn      zclSampleSmartPower_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleSmartPower_BasicResetCB( void )
{
  NLME_LeaveReq_t leaveReq;
  // Set every field to 0
  osal_memset( &leaveReq, 0, sizeof( NLME_LeaveReq_t ) );
  
  // This will enable the device to rejoin the network after reset.
  leaveReq.rejoin = TRUE;
  
  // Set the NV startup option to force a "new" join.
  zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE );    
  
  // Leave the network, and reset afterwards
  if ( NLME_LeaveReq( &leaveReq ) != ZSuccess )
  {
    // Couldn't send out leave; prepare to reset anyway
    ZDApp_LeaveReset( FALSE );
  }
}

/*********************************************************************
 * @fn      zclSampleSmartPower_OnOffCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   cmd - COMMAND_ON, COMMAND_OFF or COMMAND_TOGGLE
 *
 * @return  none
 */
static void zclSampleSmartPower_OnOffCB( uint8 cmd )
{
  //osal_set_event(zclSampleSmartPower_TaskID, RELAY_TEST_EVT);
  // Turn on the light
  if ( cmd == COMMAND_ON )
  {
    zclSampleSmartPower_OnOff = LIGHT_ON;
    
      HalRelayOff();
  }
  // Turn off the light
  else if ( cmd == COMMAND_OFF )
  {
    zclSampleSmartPower_OnOff = LIGHT_OFF;
    HalRelayOn();
  }
  // Toggle the light
  else if ( cmd == COMMAND_TOGGLE )
  {
    
    if ( zclSampleSmartPower_OnOff == LIGHT_OFF )
    {
      zclSampleSmartPower_OnOff = LIGHT_ON;
      HalRelayOff();
    }
    else
    {
      zclSampleSmartPower_OnOff = LIGHT_OFF;
      HalRelayOn();
    }
  }


}



/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleSmartPower_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleSmartPower_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleSmartPower_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleSmartPower_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // Attribute Reporting implementation should be added here
    case ZCL_CMD_CONFIG_REPORT:
      // zclSampleSmartPower_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      // zclSampleSmartPower_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      // zclSampleSmartPower_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      // zclSampleSmartPower_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      // zclSampleSmartPower_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleSmartPower_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleSmartPower_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleSmartPower_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleSmartPower_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleSmartPower_ProcessInDiscAttrsExtRspCmd( pInMsg );
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
 * @fn      zclSampleSmartPower_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSmartPower_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
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

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleSmartPower_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSmartPower_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleSmartPower_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSmartPower_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleSmartPower_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSmartPower_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleSmartPower_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSmartPower_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleSmartPower_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSmartPower_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER

#if ZCL_EZMODE
/*********************************************************************
 * @fn      zclSampleSmartPower_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleSmartPower_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{
  zclEZMode_ActionData_t data;
  ZDO_MatchDescRsp_t *pMatchDescRsp;

  // Let EZ-Mode know of the Simple Descriptor Response
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    pMatchDescRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pMatchDescRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free( pMatchDescRsp );
  }
}

/*********************************************************************
 * @fn      zclSampleSmartPower_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSampleSmartPower_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

    zclSampleSmartPower_IdentifyTime = ( EZMODE_TIME / 1000 );  // convert to seconds
    
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
      if ( giLightScreenMode == LIGHT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclSampleSmartPower_IdentifyTime = 0;
    

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // already stated on autoclose
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_TIMEDOUT )
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giLightScreenMode == LIGHT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif
    // show main UI screen 3 seconds after binding
    osal_start_timerEx( zclSampleSmartPower_TaskID, SAMPLELIGHT_MAIN_SCREEN_EVT, 3000 );
  }
}
#endif // ZCL_EZMODE

static void sendData(void){
  
  
  
    CurrentValue = HalAdcRead(HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_8);
    Cal5V = HalAdcRead(HAL_ADC_CHANNEL_4, HAL_ADC_RESOLUTION_8);
  /*
  zclReportCmd_t *pReportCmd;
  
  
  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_IOV_BASIC_PRESENT_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_SINGLE_PREC;
    pReportCmd->attrList[0].attrData = (void *)(&CurrentValue);
    
  zcl_SendReportCmd(SAMPLELIGHT_ENDPOINT, &zclSampleSmartPower_DstAddr,
                       ZCL_CLUSTER_ID_GEN_ANALOG_INPUT_BASIC,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSampleSmartPowerSeqNum++);
  }
  osal_mem_free( pReportCmd );
*/
}

/*********************************************************************
*********************************************************************/
static void readPower( void )
{
  uint8 i;
  uint16 read[5], current, volt = 0x7FFF, vddCoffi, currCoffi, noCurrSample = 0; //a,b
  int16 power;
  int32 vdd, cofficient1, cofficient2, readPower, readCurrent, readAvg = 0;
  
  //noCurrSample = 4840;//5170  //
  //5VCal(2.477V~2.517V), the Value which read from adc channel 4 is always 8191(close to 1.65V) but the value which is close to 2.3V
  noCurrSample = HalAdcRead(HAL_ADC_CHANNEL_4, HAL_ADC_RESOLUTION_14);
  noCurrSample = (uint16)((uint32)noCurrSample * 5 / 6);
  //noCurrSample = (noCurrSample+100);
  noCurrSample = convertToQFormat(noCurrSample, 8191);//20682
  
  for (i = 0; i<5; i++){
    read[i] = sampleAdcValue(HAL_ADC_CHANNEL_0);
    //if (readAvg < read[i]) readAvg = read[i];
    readAvg += read[i];
  }
  readAvg = readAvg / 5;
  vdd = ensureVdd();
  vdd = (vdd*345)/511;  //vdd*1.15*3/511
  //vdd = convertToInt(vdd, 3450);
  cofficient1 = (vdd*459)/1000;  //vdd*(1/1.414)*(1/0.154)//vdd*(1/1.414)*(1/0.185)=(vdd*382)/1000
  cofficient2 = cofficient1*11;  //vdd*(1/1.414)*(1/0.185)*(110)
  
  currCoffi = convertToQFormat(cofficient1, 1000);//4227
  vddCoffi = convertToQFormat( cofficient2, 10000);
  
  current = convertToQFormat( readAvg, 8191);
  current = current - noCurrSample;   //25182@2.5V
  readCurrent = q15Mul(current, currCoffi);
  readCurrent = convertToInt(readCurrent, 1000000);
  PowerSensor_MeasuredCurrentValue = readCurrent / 10;
  
  power = q15Mul(current, volt);
  readPower = q15Mul(power, vddCoffi);
  readPower = convertToInt(readPower, 1000000);
  PowerSensor_MeasuredPowerValue = readPower / 100;
  
  //HalAdcSetReference(HAL_ADC_REF_AVDD);
  //a = HalAdcRead(HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);
  //b = ensureVdd();
  
  //Power_SetParameter( SENSOR_DATA1, CURRENT_DATA_LEN, &PowerSensor_MeasuredCurrentValue );
  //Power_SetParameter( SENSOR_DATA, POWER_DATA_LEN, &PowerSensor_MeasuredValue );
  //HalLcdWriteValue( 1000000+readAvg, 10, HAL_LCD_LINE_5 );
  //HalLcdWriteValue( 1000000+b, 10, HAL_LCD_LINE_6 );
  
}

static uint16 sampleAdcValue( uint8 channel ) //Find the maximum of input sine wave
{
  uint16 read[16], read1[10], read2[10], maxNum = 0;
  uint8 i, j, k;
  HalAdcSetReference(HAL_ADC_REF_AVDD);
  
  for (i = 0; i<16; i++)
  {
    read[i] = HalAdcRead(channel, HAL_ADC_RESOLUTION_14);
    //ST_HAL_DELAY(130);//
    Onboard_wait(1041);
    read[i] = read[i];
    
    if(maxNum < read[i]) {
      maxNum = read[i];
      j = i;
    }
  }
  //ST_HAL_DELAY(130);//
  Onboard_wait(1041);
  
  for (i = 0; i<16; i++)
  {
    //ST_HAL_DELAY(130);//
    Onboard_wait(1041);
    if (i == j) {
      for (k = 0; k < 10; k++) {
        read1[k] = HalAdcRead(channel, HAL_ADC_RESOLUTION_14);
        //ST_HAL_DELAY(13);//
        Onboard_wait(104);
        
        if(maxNum < read1[k]) {
          maxNum = read1[k];
        }
      }
    }
  }
  //ST_HAL_DELAY(130);//
  Onboard_wait(1041);
  
  for (i = 0; i<16; i++)
  {
    //ST_HAL_DELAY(130);//
    Onboard_wait(1041);
    if (i == j-1) {
      for (k = 0; k < 10; k++) {
        read2[k] = HalAdcRead(channel, HAL_ADC_RESOLUTION_14);
        //ST_HAL_DELAY(13);//
        Onboard_wait(104);
        
        if(maxNum < read2[k]) {
          maxNum = read2[k];
        }
      }
    }
  }
  
  //HalLcdWriteValue( 100000+maxNum, 10, HAL_LCD_LINE_5 );
  
  return maxNum;
}

static int16 convertToQFormat (int16 num, int16 maxNum)
{
  int16 i = 0;
  
  if (num > 0) {
    i = (int16)(((int32)num << 15) / (int32)maxNum);
  }
  
  return i;
}

static int16 convertToInt (int16 num, int32 amp)
{
  uint8 i;
  int32 tempNum = 0, conNum = 0;
  for (i = 1; i < 16; i++)
  {
    tempNum = (num >> (15-i));
    if (tempNum%2 == 1)
    {
      tempNum = 1;
      tempNum = (tempNum*amp) >> i;
      conNum += tempNum;
    } else 
    {
      tempNum = 0;
    }   
  }
  //HalLcdWriteValue( conNum, 10, HAL_LCD_LINE_1 );
  return conNum;
}

static int q15Mul(int a, int b)
{
    long int tmp = ((long int)a * (long int)b) / 32768; // shift right 15-bits

    if ( tmp >= 32767 ) tmp = 32767;
    if ( tmp <= (-32767) ) tmp = -32767;
    
    return (int)tmp;
}

static int16 ensureVdd(void) 
{
  uint16 adcValue = 0;

  HalAdcSetReference(HAL_ADC_REF_125V);
  adcValue = HalAdcRead(HAL_ADC_CHN_VDD3, HAL_ADC_RESOLUTION_10);
  
  return adcValue;
}

