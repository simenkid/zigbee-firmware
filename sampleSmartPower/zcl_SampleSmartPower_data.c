/**************************************************************************************************
  Filename:       zcl_SampleSmartPower.c
  Revised:        $Date: 2014-05-12 13:14:02 -0700 (Mon, 12 May 2014) $
  Revision:       $Revision: 38502 $


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
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_poll_control.h"
#include "zcl_electrical_measurement.h"
#include "zcl_diagnostic.h"
#include "zcl_meter_identification.h"
#include "zcl_appliance_identification.h"
#include "zcl_appliance_events_alerts.h"
#include "zcl_power_profile.h"
#include "zcl_appliance_control.h"
#include "zcl_appliance_statistics.h"
#include "zcl_hvac.h"
#include "zcl_se.h"

#include "zcl_SampleSmartPower.h"
   #include "zcl_ms.h"

/*********************************************************************
 * CONSTANTS
 */

#define SAMPLELIGHT_DEVICE_VERSION     0
#define SAMPLELIGHT_FLAGS              0

#define SAMPLELIGHT_HWVERSION          1
#define SAMPLELIGHT_ZCLVERSION         1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Basic Cluster
const uint8 zclSampleSmartPower_HWRevision = SAMPLELIGHT_HWVERSION;
const uint8 zclSampleSmartPower_ZCLVersion = SAMPLELIGHT_ZCLVERSION;
const uint8 zclSampleSmartPower_ManufacturerName[] = { 16, 'T','e','x','a','s','I','n','s','t','r','u','m','e','n','t','s' };
const uint8 zclSampleSmartPower_ModelId[] = { 16, 'T','I','0','0','0','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSampleSmartPower_DateCode[] = { 16, '2','0','0','6','0','8','3','1',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSampleSmartPower_PowerSource = POWER_SOURCE_MAINS_1_PHASE;

uint8 zclSampleSmartPower_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclSampleSmartPower_PhysicalEnvironment = 0;
uint8 zclSampleSmartPower_DeviceEnable = DEVICE_ENABLED;
 // in mv
uint8 zclSampleGasSensorOutOfService = 0;
uint8 zclSampleGasSensorStatusFlag = 0;

// Identify Cluster
uint16 zclSampleSmartPower_IdentifyTime = 0;
#ifdef ZCL_EZMODE
uint8  zclSampleSmartPower_IdentifyCommissionState;
#endif

// On/Off Cluster
uint8  zclSampleSmartPower_OnOff = LIGHT_OFF;
uint8  relayonoff = RELAY_OFF;

// Level Control Cluster
#ifdef ZCL_LEVEL_CTRL
uint8  zclSampleSmartPower_LevelCurrentLevel = ATTR_LEVEL_MIN_LEVEL;
uint16 zclSampleSmartPower_LevelRemainingTime;
uint16 zclSampleSmartPower_LevelOnOffTransitionTime = 20;
uint8  zclSampleSmartPower_LevelOnLevel = ATTR_LEVEL_MID_LEVEL;
uint16 zclSampleSmartPower_LevelOnTransitionTime = 20;
uint16 zclSampleSmartPower_LevelOffTransitionTime = 20;
uint8  zclSampleSmartPower_LevelDefaultMoveRate = 0;   // as fast as possible
#endif

#if ZCL_DISCOVER
CONST zclCommandRec_t zclSampleSmartPower_Cmds[] =
{
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    COMMAND_BASIC_RESET_FACT_DEFAULT,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_ON,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_TOGGLE,
    CMD_DIR_SERVER_RECEIVED
  },
#ifdef ZCL_LEVEL_CONTROL
  ,{
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_TO_LEVEL,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STEP,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STOP,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_TO_LEVEL_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STEP_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STOP_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  }
#endif // ZCL_LEVEL_CONTROL
};

CONST uint8 zclCmdsArraySize = ( sizeof(zclSampleSmartPower_Cmds) / sizeof(zclSampleSmartPower_Cmds[0]) );
#endif // ZCL_DISCOVER

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclSampleSmartPower_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSampleSmartPower_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleSmartPower_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleSmartPower_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleSmartPower_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleSmartPower_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleSmartPower_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclSampleSmartPower_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleSmartPower_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleSmartPower_DeviceEnable
    }
  },

#ifdef ZCL_IDENTIFY
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleSmartPower_IdentifyTime
    }
  },
 #ifdef ZCL_EZMODE
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_COMMISSION_STATE,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ),
      (void *)&zclSampleSmartPower_IdentifyCommissionState
    }
  },
 #endif // ZCL_EZMODE
#endif

  // *** On/Off Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleSmartPower_OnOff
    }
  }

#ifdef ZCL_LEVEL_CTRL
  , {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_CURRENT_LEVEL,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleSmartPower_LevelCurrentLevel
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_REMAINING_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleSmartPower_LevelRemainingTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_OFF_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleSmartPower_LevelOnOffTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_LEVEL,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleSmartPower_LevelOnLevel
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleSmartPower_LevelOnTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_OFF_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleSmartPower_LevelOffTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_DEFAULT_MOVE_RATE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleSmartPower_LevelDefaultMoveRate
    }
  }
#endif
 #ifdef ZCL_DIAGNOSTIC
  , {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NUMBER_OF_RESETS,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_PERSISTENT_MEMORY_WRITES,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_RX_BCAST,
      ZCL_DATATYPE_UINT32,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_TX_BCAST,
      ZCL_DATATYPE_UINT32,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_RX_UCAST,
      ZCL_DATATYPE_UINT32,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_TX_UCAST,
      ZCL_DATATYPE_UINT32,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_TX_UCAST_RETRY,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_TX_UCAST_FAIL,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_RX_BCAST,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_TX_BCAST,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_RX_UCAST,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_TX_UCAST_SUCCESS,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_TX_UCAST_RETRY,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_TX_UCAST_FAIL,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_ROUTE_DISC_INITIATED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NEIGHBOR_ADDED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NEIGHBOR_REMOVED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NEIGHBOR_STALE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_JOIN_INDICATION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_CHILD_MOVED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NWK_FC_FAILURE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_FC_FAILURE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_UNAUTHORIZED_KEY,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NWK_DECRYPT_FAILURES,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_DECRYPT_FAILURES,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_PACKET_BUFFER_ALLOCATE_FAILURES,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_RELAYED_UCAST,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_PHY_TO_MAC_QUEUE_LIMIT_REACHED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_PACKET_VALIDATE_DROP_COUNT,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_AVERAGE_MAC_RETRY_PER_APS_MESSAGE_SENT,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_LAST_MESSAGE_LQI,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_LAST_MESSAGE_RSSI,
      ZCL_DATATYPE_INT8,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
#endif // ZCL_DIAGNOSTIC

// *** Analog Input Cluster ***
  {
    ZCL_CLUSTER_ID_SE_SIMPLE_METERING,
    { // Attribute record
      ATTRID_SE_CURRENT_SUMMATION_DELIVERED,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&PowerSensor_MeasuredCurrentValue
    }
  },
  
  {
    ZCL_CLUSTER_ID_SE_SIMPLE_METERING,
    { // Attribute record
      ATTRID_SE_POWER_FACTOR,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&PowerSensor_MeasuredPowerValue
    }
  },
  
  /*
  {
    ZCL_CLUSTER_ID_GEN_POWER_CFG,
    { // Attribute record
      ATTRID_POWER_CFG_MAINS_DWELL_TRIP_POINT,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&PowerSensor_MeasuredPowerValue
    }
  },
  */
};

uint8 CONST zclSampleSmartPower_NumAttributes = ( sizeof(zclSampleSmartPower_Attrs) / sizeof(zclSampleSmartPower_Attrs[0]) );

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t zclSampleSmartPower_InClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_GEN_GROUPS,
  ZCL_CLUSTER_ID_GEN_SCENES,
  ZCL_CLUSTER_ID_GEN_ON_OFF,
  ZCL_CLUSTER_ID_SE_SIMPLE_METERING
#ifdef ZCL_LEVEL_CTRL
  , ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL
#endif
};
// work-around for compiler bug... IAR can't calculate size of array with #if options.
#ifdef ZCL_LEVEL_CTRL
 #define ZCLSAMPLELIGHT_MAX_INCLUSTERS   7
#else
 #define ZCLSAMPLELIGHT_MAX_INCLUSTERS   6
#endif

const cId_t zclSampleSmartPower_OutClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC
};
#define ZCLSAMPLELIGHT_MAX_OUTCLUSTERS  (sizeof(zclSampleSmartPower_OutClusterList) / sizeof(zclSampleSmartPower_OutClusterList[0]))

SimpleDescriptionFormat_t zclSampleSmartPower_SimpleDesc =
{
  SAMPLELIGHT_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
#ifdef ZCL_LEVEL_CTRL
  ZCL_HA_DEVICEID_DIMMABLE_LIGHT,        //  uint16 AppDeviceId;
#else
  ZCL_HA_DEVICEID_ON_OFF_LIGHT,          //  uint16 AppDeviceId;
#endif
  SAMPLELIGHT_DEVICE_VERSION,            //  int   AppDevVer:4;
  SAMPLELIGHT_FLAGS,                     //  int   AppFlags:4;
  ZCLSAMPLELIGHT_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSampleSmartPower_InClusterList, //  byte *pAppInClusterList;
  ZCLSAMPLELIGHT_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSampleSmartPower_OutClusterList //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/****************************************************************************
****************************************************************************/


