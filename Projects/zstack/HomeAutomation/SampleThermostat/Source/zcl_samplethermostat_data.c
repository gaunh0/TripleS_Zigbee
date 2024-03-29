/**************************************************************************************************
  Filename:       zcl_samplethermostat_data.c
  Revised:        $Date: 2014-09-25 13:20:41 -0700 (Thu, 25 Sep 2014) $
  Revision:       $Revision: 40295 $


  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2013-2014 Texas Instruments Incorporated. All rights reserved.

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
#include "zcl_ezmode.h"

#include "zcl_samplethermostat.h"

/*********************************************************************
 * CONSTANTS
 */

#define SAMPLETHERMOSTAT_DEVICE_VERSION     0
#define SAMPLETHERMOSTAT_FLAGS              0

#define SAMPLETHERMOSTAT_HWVERSION          1
#define SAMPLETHERMOSTAT_ZCLVERSION         1

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
const uint8 zclSampleThermostat_HWRevision = SAMPLETHERMOSTAT_HWVERSION;
const uint8 zclSampleThermostat_ZCLVersion = SAMPLETHERMOSTAT_ZCLVERSION;
const uint8 zclSampleThermostat_ManufacturerName[] = { 16, 'T','e','x','a','s','I','n','s','t','r','u','m','e','n','t','s' };
const uint8 zclSampleThermostat_ModelId[] = { 16, 'T','I','0','0','0','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSampleThermostat_DateCode[] = { 16, '2','0','0','6','0','8','3','1',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSampleThermostat_PowerSource = POWER_SOURCE_MAINS_1_PHASE;

uint8 zclSampleThermostat_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclSampleThermostat_PhysicalEnvironment = 0;
uint8 zclSampleThermostat_DeviceEnable = DEVICE_ENABLED;

// Identify Cluster
uint16 zclSampleThermostat_IdentifyTime = 0;

// On/Off Cluster
uint8  zclSampleThermostat_OnOff = COMMAND_OFF;

// HVAC Thermostat Cluster
int16 zclSampleThermostat_LocalTemperature = NULL;
int16 zclSampleThermostat_MinHeatSetpointLimit = 1700;  // 17.00C
int16 zclSampleThermostat_MaxHeatSetpointLimit = 2700;  // 27.00C
int16 zclSampleThermostat_MinCoolSetpointLimit = 1700;  // 17.00C
int16 zclSampleThermostat_MaxCoolSetpointLimit = 2700;  // 27.00C
int16 zclSampleThermostat_OccupiedHeatingSetpoint = 2000; // 20.00C
int16 zclSampleThermostat_OccupiedCoolingSetpoint = 3000; // 24.00C
uint8 zclSampleThermostat_HeatingDemand = 100;   // 100% heating demanded of heating device
uint8 zclSampleThermostat_CoolingDemand = 100;   // 100% cooling demanded of cooling device
uint8 zclSampleThermostat_ControlSequenceOfOperation = HVAC_THERMOSTAT_CTRL_SEQ_OF_OPER_COOLING_HEATING;    // Both heating and cooling is possible
uint8 zclSampleThermostat_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_OFF;

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclSampleThermostat_Attrs[SAMPLETHERMOSTAT_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSampleThermostat_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleThermostat_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleThermostat_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleThermostat_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclSampleThermostat_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleThermostat_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleThermostat_DeviceEnable
    }
  },

  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleThermostat_IdentifyTime
    }
  },

  // *** On/Off Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_OnOff
    }
  },

  // *** HVAC Thermostat Cluster Attributes *** //
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_OCCUPIED_COOLING_SETPOINT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleThermostat_OccupiedCoolingSetpoint
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_OCCUPIED_HEATING_SETPOINT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleThermostat_OccupiedHeatingSetpoint
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_LOCAL_TEMPERATURE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_LocalTemperature
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_MinHeatSetpointLimit
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_MaxHeatSetpointLimit
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_MinCoolSetpointLimit
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_MaxCoolSetpointLimit
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_PI_COOLING_DEMAND,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_CoolingDemand
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_PI_HEATING_DEMAND,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleThermostat_HeatingDemand
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_CTRL_SEQ_OF_OPER,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleThermostat_ControlSequenceOfOperation
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_SYSTEM_MODE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleThermostat_SystemMode
    }
  },
};

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
#define ZCLSAMPLETHERMOSTAT_MAX_INCLUSTERS       3
const cId_t zclSampleThermostat_InClusterList[ZCLSAMPLETHERMOSTAT_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_HVAC_THERMOSTAT
};

#define ZCLSAMPLETHERMOSTAT_MAX_OUTCLUSTERS       1
const cId_t zclSampleThermostat_OutClusterList[ZCLSAMPLETHERMOSTAT_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
};

SimpleDescriptionFormat_t zclSampleThermostat_SimpleDesc =
{
  SAMPLETHERMOSTAT_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                          //  uint16 AppProfId[2];
  ZCL_HA_DEVICEID_THERMOSTAT,                 //  uint16 AppDeviceId[2];
  SAMPLETHERMOSTAT_DEVICE_VERSION,            //  int   AppDevVer:4;
  SAMPLETHERMOSTAT_FLAGS,                     //  int   AppFlags:4;
  ZCLSAMPLETHERMOSTAT_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSampleThermostat_InClusterList, //  byte *pAppInClusterList;
  ZCLSAMPLETHERMOSTAT_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSampleThermostat_OutClusterList //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/****************************************************************************
****************************************************************************/


