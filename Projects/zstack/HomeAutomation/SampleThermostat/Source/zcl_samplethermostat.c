/**************************************************************************************************
  Filename:       zcl_samplethermostat.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2013 - 2014 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
  This device will act as a thermostat.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Set heating setpoint
    - SW2: Invoke EZMode
    - SW3: Set cooling setpoint
    - SW4: Enable/Disable Permit Join
    - SW5: Go to Help screen

  Heating Setpoint or Cooling Setpoint:
    - SW1: Increase temperature
    - SW3: Decrease temperature
    - SW5: Save temperature
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
#include "MT_APP.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_hvac.h"
#include "zcl_ms.h"

#include "zcl_samplethermostat.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

/* MY INCLUDES */
#include "MS_UART.h"
#include "MS_UART_CMD.h"
#include "MS_GPIO.h"
#include "uti.h"

#if ( defined (ZGP_DEVICE_TARGET) || defined (ZGP_DEVICE_TARGETPLUS) \
      || defined (ZGP_DEVICE_COMBO) || defined (ZGP_DEVICE_COMBO_MIN) )
#include "zgp_translationtable.h"
  #if (SUPPORTED_S_FEATURE(SUPP_ZGP_FEATURE_TRANSLATION_TABLE))
    #define ZGP_AUTO_TT
  #endif
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
byte zclSampleThermostat_TaskID;
uint8 zclSampleThermostatSeqNum;

static byte gPermitDuration = 0x00;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleThermostat_DstAddr;

#ifdef ZCL_EZMODE
static void zclSampleThermostat_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleThermostat_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSampleThermostat_RegisterEZModeData =
{
  &zclSampleThermostat_TaskID,
  SAMPLETHERMOSTAT_EZMODE_NEXTSTATE_EVT,
  SAMPLETHERMOSTAT_EZMODE_TIMEOUT_EVT,
  &zclSampleThermostatSeqNum,
  zclSampleThermostat_EZModeCB
};

// NOT ZCL_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_HVAC_THERMOSTAT
};
#define ZCLSAMPLETHERMOSTAT_BINDINGLIST_OUT     1

static cId_t bindingInClusters[] =
{
  ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
};
#define ZCLSAMPLETHERMOSTAT_BINDINGLIST_IN      2
#endif

uint8 giThermostatScreenMode = THERMOSTAT_MAINMODE;   // display the main screen mode first

devStates_t zclSampleThermostat_NwkState = DEV_INIT;

static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleThermostat_TestEp =
{
  20,                                 // Test endpoint
  &zclSampleThermostat_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
#ifndef COORDINATOR
static void zclSampleThermostat_RollCall(void);
#endif

static void zclSampleThermostat_HandleKeys( byte shift, byte keys );
static void zclSampleThermostat_BasicResetCB( void );
static void zclSampleThermostat_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleThermostat_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclSampleThermostat_ProcessIdentifyTimeChange( void );
static void zclSampleThermostat_ProcessAppMsg( uint8 srcEP, uint8 len, uint8 *msg );
static void zclSampleThermostat_ProcessFoundationMsg( afAddrType_t *dstAddr, uint16 clusterID,
                                                      zclFrameHdr_t *hdr, zclParseCmd_t *pParseCmd );

// app display functions
void zclSampleThermostat_LcdDisplayUpdate(void);
void zclSampleThermostat_LcdDisplayMainMode(void);
void zclSampleThermostat_LcdDisplayHeatMode(void);
void zclSampleThermostat_LcdDisplayCoolMode(void);
void zclSampleThermostat_LcdDisplayHelpMode(void);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleThermostat_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleThermostat_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleThermostat_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_REPORT
static void zclSampleThermostat_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleThermostat_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]     = " ";
const char sDeviceName[]    = "   Thermostat";
const char sSwHeatSet[]     = "SW1: Set Heating";
const char sSwEZMode[]      = "SW2: EZ-Mode";
const char sSwCoolSet[]     = "SW3: Set Cooling";
const char sTempLine2[]     = "SW1:+";
const char sTempLine3[]     = "SW3:-  SW5:Enter";
const char sSwHelp[]        = "SW5: Help";
const char sStoreHeatTemp[] = "HEAT TEMP SAVED";
const char sStoreCoolTemp[] = "COOL TEMP SAVED";
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleThermostat_CmdCallbacks =
{
  zclSampleThermostat_BasicResetCB,            // Basic Cluster Reset command
  zclSampleThermostat_IdentifyCB,              // Identify command
#ifdef ZCL_EZMODE
  NULL,                                        // Identify EZ-Mode Invoke command
  NULL,                                        // Identify Update Commission State command
#endif
  NULL,                                        // Identify Trigger Effect command
  zclSampleThermostat_IdentifyQueryRspCB,      // Identify Query Response command
  NULL,             				                   // On/Off cluster command
  NULL,                                        // On/Off cluster enhanced command Off with Effect
  NULL,                                        // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                        // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                        // Level Control Move to Level command
  NULL,                                        // Level Control Move command
  NULL,                                        // Level Control Step command
  NULL,                                        // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                        // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                        // Scene Store Request command
  NULL,                                        // Scene Recall Request command
  NULL,                                        // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                        // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                        // Get Event Log command
  NULL,                                        // Publish Event Log command
#endif
  NULL,                                        // RSSI Location command
  NULL                                         // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSampleThermostat_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleThermostat_Init( byte task_id )
{
  zclSampleThermostat_TaskID = task_id;

  // Set destination address to indirect
  zclSampleThermostat_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleThermostat_DstAddr.endPoint = 0;
  zclSampleThermostat_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleThermostat_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLETHERMOSTAT_ENDPOINT, &zclSampleThermostat_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( SAMPLETHERMOSTAT_ENDPOINT, SAMPLETHERMOSTAT_MAX_ATTRIBUTES, zclSampleThermostat_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleThermostat_TaskID );

	#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleThermostat_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
	#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleThermostat_TaskID );

  // Register for a test endpoint
  afRegister( &sampleThermostat_TestEp );

  ZDO_RegisterForZDOMsg( zclSampleThermostat_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( zclSampleThermostat_TaskID, Match_Desc_rsp );

	#ifdef LCD_SUPPORTED
  // display the device name
  HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_3 );
	#endif

	#ifdef ZGP_AUTO_TT
  zgpTranslationTable_RegisterEP( &zclSampleThermostat_SimpleDesc );
	#endif

	UART_Init(HAL_UART_PORT_0);
	// cho nay khai bao gpio
	GPIO_init();
	
	// Set timer for first UART Reply CMD event
	osal_start_timerEx( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_UART_REPLY_CMD_EVT, UART_PARSE_RX_PACKAGE_EVT_PERIOD );

	// Set timer for first Check System event
	osal_start_timerEx( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_CHECK_SYSTEM_EVT, CHECK_SYSTEM_EVT_PERIOD );

	// Set timer for first Join Setup event
	if (NLME_GetShortAddr() != 0xFFFE)
	{
		osal_set_event( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_JOIN_SETUP_EVT );
	}
	
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
uint16 zclSampleThermostat_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

	/*--------------------------------------------------------------------------*/
  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleThermostat_TaskID )) )
    {
    	msg_RSSI = MSGpkt->rssi;
				
      switch ( MSGpkt->hdr.event )
      {
				#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSampleThermostat_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
					UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] ZDO_CB_MSG");
          break;
				#endif

        case MT_SYS_APP_MSG:
          // Message received from MT
          zclSampleThermostat_ProcessAppMsg( ((mtSysAppMsg_t *)MSGpkt)->endpoint,
                                          ((mtSysAppMsg_t *)MSGpkt)->appDataLen,
                                          ((mtSysAppMsg_t *)MSGpkt)->appData );
					UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] MT_SYS_APP_MSG");
          break;

        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleThermostat_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
					UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] ZCL_INCOMING_MSG");
          break;

        case KEY_CHANGE:
          zclSampleThermostat_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
					UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] KEY_CHANGE");
          break;

        case ZDO_STATE_CHANGE:
          zclSampleThermostat_NwkState = (devStates_t)(MSGpkt->hdr.status);


          // now on the network
          if ( ( zclSampleThermostat_NwkState == DEV_ZB_COORD ) ||
               ( zclSampleThermostat_NwkState == DEV_ROUTER )   ||
               ( zclSampleThermostat_NwkState == DEV_END_DEVICE ) )
          {
						#ifndef HOLD_AUTO_START
            // display main mode
            giThermostatScreenMode = THERMOSTAT_MAINMODE;
            zclSampleThermostat_LcdDisplayUpdate();
						#endif
						#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
						#endif  // ZCL_EZMODE
						
						UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] ZDO_STATE_CHANGE");
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

	/*--------------------------------------------------------------------------*/
  if ( events & SAMPLETHERMOSTAT_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSampleThermostat_IdentifyTime > 0 )
    {
      zclSampleThermostat_IdentifyTime--;
    }
    zclSampleThermostat_ProcessIdentifyTimeChange();
		
		UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] SAMPLETHERMOSTAT_IDENTIFY_TIMEOUT_EVT");

    return ( events ^ SAMPLETHERMOSTAT_IDENTIFY_TIMEOUT_EVT );
  }

	/*--------------------------------------------------------------------------*/
  if ( events & SAMPLETHERMOSTAT_MAIN_SCREEN_EVT )
  {
    giThermostatScreenMode = THERMOSTAT_MAINMODE;
    zclSampleThermostat_LcdDisplayUpdate();

		if (FLAG_BINDING)
		{
			FLAG_BINDING = FALSE;		
			if (FLAG_HARD_BINDING)
			{
				FLAG_HARD_BINDING = FALSE;
			}
		}
		UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] SAMPLETHERMOSTAT_MAIN_SCREEN_EVT");

    return ( events ^ SAMPLETHERMOSTAT_MAIN_SCREEN_EVT );
  }

	/*--------------------------------------------------------------------------*/
	#ifdef ZCL_EZMODE
  // going on to next state
  if ( events & SAMPLETHERMOSTAT_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state

		UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] SAMPLETHERMOSTAT_EZMODE_NEXTSTATE_EVT");
		
    return ( events ^ SAMPLETHERMOSTAT_EZMODE_NEXTSTATE_EVT );
  }

	/*--------------------------------------------------------------------------*/
  // the overall EZMode timer expired, so we timed out
  if ( events & SAMPLETHERMOSTAT_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out

		UART_DebugPrint(HAL_UART_PORT_0, "[EVENT] SAMPLETHERMOSTAT_EZMODE_TIMEOUT_EVT");
		
    return ( events ^ SAMPLETHERMOSTAT_EZMODE_TIMEOUT_EVT );
  }
	#endif // ZLC_EZMODE

	/*--------------------------------------------------------------------------*/
	if ( events & SAMPLETHERMOSTAT_JOIN_SETUP_EVT )
  {
		if (FLAG_JOIN_CONFIRM == FALSE && FLAG_BINDING == FALSE)
		{
			zclSampleThermostat_BindingProcess();
			// Stop binding after 100ms (time for process...)
			osal_start_timerEx( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_JOIN_SETUP_EVT, 5000 );
		}
		else if (FLAG_JOIN_CONFIRM == TRUE && FLAG_BINDING == TRUE)
		{
			zclSampleThermostat_BindingProcess();
		}
		
		UART_DebugPrint(HAL_UART_PORT_0, "SAMPLETHERMOSTAT_JOIN_SETUP_EVT");

    return ( events ^ SAMPLETHERMOSTAT_JOIN_SETUP_EVT );
  }

	/*--------------------------------------------------------------------------*/
	if ( events & SAMPLETHERMOSTAT_UART_REPLY_CMD_EVT )
  {
		// My apps
		if (UART_ParseRxPackage(HAL_UART_PORT_0))
		{
			ZCMD_ReplyCMD();
		}
		
		// Restart timer
    if ( UART_PARSE_RX_PACKAGE_EVT_PERIOD )
    {
      osal_start_timerEx( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_UART_REPLY_CMD_EVT, UART_PARSE_RX_PACKAGE_EVT_PERIOD );
    }
		
    return (events ^ SAMPLETHERMOSTAT_UART_REPLY_CMD_EVT);
  }

	/*--------------------------------------------------------------------------*/
	if ( events & SAMPLETHERMOSTAT_CHECK_SYSTEM_EVT )
  {
		// My apps
		#ifndef COORDINATOR
		zclSampleThermostat_RollCall();
		#endif
		
		if ( (FLAG_JOIN_CONFIRM == FALSE) || (FLAG_JOIN_CONFIRM == TRUE && FLAG_BINDING == TRUE && FLAG_HARD_BINDING == FALSE) )
		{
			osal_set_event( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_JOIN_SETUP_EVT );
		}

    // Restart timer
    if ( CHECK_SYSTEM_EVT_PERIOD )
    {
      osal_start_timerEx( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_CHECK_SYSTEM_EVT, CHECK_SYSTEM_EVT_PERIOD );
    }
			 
    return (events ^ SAMPLETHERMOSTAT_CHECK_SYSTEM_EVT);
  }
	
	/*--------------------------------------------------------------------------*/

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleThermostat_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclSampleThermostat_HandleKeys( byte shift, byte keys )
{
	if ( keys & HAL_KEY_SW_6 ) //~ SW1 on board
	{
		UART_DebugPrint(HAL_UART_PORT_0, "SW6");

		uint16 maxEntries;
		uint16 usedEntries;
		BindingEntry_t* current_BindingTableEntry;

		bindCapacity(&maxEntries, &usedEntries);
		UART_ZCmdPrintString(HAL_UART_PORT_0, "usedEntries: ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, usedEntries);
		UART_ZCmdPrint			(HAL_UART_PORT_0, "");

		current_BindingTableEntry = GetBindingTableEntry( 0 );
		UART_ZCmdPrintString(HAL_UART_PORT_0, "Entry 0: ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, current_BindingTableEntry[0].srcEP);
		UART_ZCmdPrintString(HAL_UART_PORT_0, " | ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, current_BindingTableEntry[0].dstEP);
		UART_ZCmdPrint			(HAL_UART_PORT_0, "");

		current_BindingTableEntry = GetBindingTableEntry( 1 );
		UART_ZCmdPrintString(HAL_UART_PORT_0, "Entry 1: ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, current_BindingTableEntry[1].srcEP);
		UART_ZCmdPrintString(HAL_UART_PORT_0, " | ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, current_BindingTableEntry[1].dstEP);
		UART_ZCmdPrint			(HAL_UART_PORT_0, "");

		current_BindingTableEntry = GetBindingTableEntry( 2 );
		UART_ZCmdPrintString(HAL_UART_PORT_0, "Entry 2: ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, current_BindingTableEntry[2].srcEP);
		UART_ZCmdPrintString(HAL_UART_PORT_0, " | ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, current_BindingTableEntry[2].dstEP);
		UART_ZCmdPrint			(HAL_UART_PORT_0, "");

		current_BindingTableEntry = GetBindingTableEntry( 3 );
		UART_ZCmdPrintString(HAL_UART_PORT_0, "Entry 3: ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, current_BindingTableEntry[3].srcEP);
		UART_ZCmdPrintString(HAL_UART_PORT_0, " | ");
		UART_ZCmdPrintNum		(HAL_UART_PORT_0, current_BindingTableEntry[3].dstEP);
		UART_ZCmdPrint			(HAL_UART_PORT_0, "");
		
	}
	
  if ( keys & HAL_KEY_SW_8 ) //~ SW3 on board
  {
  	UART_DebugPrint(HAL_UART_PORT_0, "SW8");
    if ( ( giThermostatScreenMode == THERMOSTAT_MAINMODE ) ||
         ( giThermostatScreenMode == THERMOSTAT_HELPMODE ) )
    {
      giThermostatScreenMode = THERMOSTAT_MAINMODE;

			FLAG_BINDING = !FLAG_BINDING;
			FLAG_HARD_BINDING = !FLAG_HARD_BINDING;

			#ifdef ZCL_EZMODE
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_HVAC_THERMOSTAT };   // only bind on the Thermostat cluster

      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLETHERMOSTAT_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( ( zclSampleThermostat_NwkState == DEV_ZB_COORD ) ||
           ( zclSampleThermostat_NwkState == DEV_ROUTER )   ||
           ( zclSampleThermostat_NwkState == DEV_END_DEVICE ) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;        // Thermostat is an initiator
      ezModeData.numActiveInClusters = 0;
      ezModeData.pActiveInClusterIDs = NULL;
      ezModeData.numActiveOutClusters = 1;   // active output cluster
      ezModeData.pActiveOutClusterIDs = clusterIDs;
      zcl_InvokeEZMode( &ezModeData );

			#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
			#endif

      // NOT ZCL_EZMODE, use EndDeviceBind
			#else
      zAddrType_t dstAddr;
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request, this bind request will
      // only use a cluster list that is important to binding.
      dstAddr.addrMode = afAddr16Bit;
      dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            SAMPLETHERMOSTAT_ENDPOINT,
                            ZCL_HA_PROFILE_ID,
                            ZCLSAMPLETHERMOSTAT_BINDINGLIST_IN, bindingInClusters,
                            ZCLSAMPLETHERMOSTAT_BINDINGLIST_OUT, bindingOutClusters,
                            TRUE );
			#endif // ZCL_EZMODE
    }
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    giThermostatScreenMode = THERMOSTAT_MAINMODE;

    if ( ( zclSampleThermostat_NwkState == DEV_ZB_COORD ) ||
         ( zclSampleThermostat_NwkState == DEV_ROUTER ) )
    {
      zAddrType_t tmpAddr;

      tmpAddr.addrMode = Addr16Bit;
      tmpAddr.addr.shortAddr = NLME_GetShortAddr();

      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;

      // Trust Center significance is always true
      ZDP_MgmtPermitJoinReq( &tmpAddr, gPermitDuration, TRUE, FALSE );
    }
  }

  if ( shift && ( keys & HAL_KEY_SW_5 ) )
  {
    zclSampleThermostat_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    if ( keys & HAL_KEY_SW_5 )
    {
      // in heating or cooling setpoint mode
      if ( giThermostatScreenMode == THERMOSTAT_HEATMODE )
      {
				#ifdef LCD_SUPPORTED
        // save current heat setpoint temperature
        HalLcdWriteString( (char *)sStoreHeatTemp, HAL_LCD_LINE_2 );
				#endif
        giThermostatScreenMode = THERMOSTAT_MAINMODE;
      }
      else if ( giThermostatScreenMode == THERMOSTAT_COOLMODE )
      {
				#ifdef LCD_SUPPORTED
        // save current cool setpoint temperature
        HalLcdWriteString( (char *)sStoreCoolTemp, HAL_LCD_LINE_2 );
				#endif
        giThermostatScreenMode = THERMOSTAT_MAINMODE;
      }
      else if ( giThermostatScreenMode == THERMOSTAT_MAINMODE )
      {
        giThermostatScreenMode = THERMOSTAT_HELPMODE;
      }
      else if ( giThermostatScreenMode == THERMOSTAT_HELPMODE )
      {
				#ifdef LCD_SUPPORTED
        HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
				#endif
        giThermostatScreenMode = THERMOSTAT_MAINMODE;
      }
    }
  }

  // update display
  zclSampleThermostat_LcdDisplayUpdate();
}

/*********************************************************************
 * @fn      zclSampleThermostat_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleThermostat_LcdDisplayUpdate( void )
{
  // use LEDs to show heating or cooling cycles based off local temperature
  if ( zclSampleThermostat_LocalTemperature != NULL )
  {
    if ( zclSampleThermostat_LocalTemperature <= zclSampleThermostat_OccupiedHeatingSetpoint )
    {
      // turn on heating
      zclSampleThermostat_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_HEAT;
//      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
//      HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
    }
    else if ( zclSampleThermostat_LocalTemperature >= zclSampleThermostat_OccupiedCoolingSetpoint )
    {
      // turn on cooling
      zclSampleThermostat_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_COOL;
//      HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
//      HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
    }
    else
    {
      // turn off heating/cooling
      zclSampleThermostat_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_OFF;
//      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
//      HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
    }
  }

  if ( giThermostatScreenMode == THERMOSTAT_HEATMODE )
  {
    zclSampleThermostat_LcdDisplayHeatMode();
  }
  else if ( giThermostatScreenMode == THERMOSTAT_COOLMODE )
  {
    zclSampleThermostat_LcdDisplayCoolMode();
  }
  else if ( giThermostatScreenMode == THERMOSTAT_HELPMODE )
  {
    zclSampleThermostat_LcdDisplayHelpMode();
  }
  else
  {
    zclSampleThermostat_LcdDisplayMainMode();
  }
}

/*********************************************************************
 * @fn      zclSampleThermostat_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleThermostat_LcdDisplayMainMode( void )
{
  char sDisplayTemp[16];

  if ( zclSampleThermostat_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( 0 );
  }
  else if ( zclSampleThermostat_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( 1 );
  }
  else if ( zclSampleThermostat_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( 2 );
  }

  osal_memcpy( sDisplayTemp, "TEMP: ", 6 );

  // if local temperature has not been set, make note on display
  if ( zclSampleThermostat_LocalTemperature == NULL )
  {
    osal_memcpy( &sDisplayTemp[6], "N/A", 4 );
  }
  else
  {
    _ltoa( ( zclSampleThermostat_LocalTemperature / 100 ), (void *)(&sDisplayTemp[6]), 10 ); // only use whole number
    osal_memcpy( &sDisplayTemp[8], "C", 2 );
  }
#ifdef LCD_SUPPORTED
  // display current temperature
  HalLcdWriteString( (char *)sDisplayTemp, HAL_LCD_LINE_2 );
#endif

#ifdef LCD_SUPPORTED
  if ( ( zclSampleThermostat_NwkState == DEV_ZB_COORD ) ||
       ( zclSampleThermostat_NwkState == DEV_ROUTER ) )
  {
    // display help key with permit join status
    if ( gPermitDuration )
    {
      HalLcdWriteString( "SW5: Help      *", HAL_LCD_LINE_3 );
    }
    else
    {
      HalLcdWriteString( "SW5: Help       ", HAL_LCD_LINE_3 );
    }
  }
  else
  {
    // display help key
    HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3);
  }
#endif
}

/*********************************************************************
 * @fn      zclSampleThermostat_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleThermostat_LcdDisplayHelpMode( void )
{
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sSwHeatSet, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwCoolSet, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSampleThermostat_LcdDisplayHeatMode
 *
 * @brief   Called to display the heating setpoint temperature on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleThermostat_LcdDisplayHeatMode( void )
{
#ifdef LCD_SUPPORTED
  char sDisplayTemp[16];

  osal_memcpy( sDisplayTemp, "HEAT TEMP: ", 11 );
  _ltoa( ( zclSampleThermostat_OccupiedHeatingSetpoint / 100 ), (void *)(&sDisplayTemp[11]), 10 ); // only use whole number
  osal_memcpy( &sDisplayTemp[13], "C", 2 );

  HalLcdWriteString( (char *)sDisplayTemp, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sTempLine2, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sTempLine3, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSampleThermostat_LcdDisplayCoolMode
 *
 * @brief   Called to display the cooling setpoint temperature on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleThermostat_LcdDisplayCoolMode( void )
{
#ifdef LCD_SUPPORTED
  char sDisplayTemp[16];

  osal_memcpy(sDisplayTemp, "COOL TEMP: ", 11);
  _ltoa( ( zclSampleThermostat_OccupiedCoolingSetpoint / 100 ), (void *)(&sDisplayTemp[11]), 10 ); // only use whole number
  osal_memcpy( &sDisplayTemp[13], "C", 2 );

  HalLcdWriteString( (char *)sDisplayTemp, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sTempLine2, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sTempLine3, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSampleThermostat_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleThermostat_ProcessIdentifyTimeChange( void )
{
  if ( zclSampleThermostat_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleThermostat_OnOff )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    }
    else
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    }

    osal_stop_timerEx( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleThermostat_ProcessAppMsg
 *
 * @brief   Process DoorLock messages
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   len - number of bytes
 * @param   msg - pointer to message
 *          0 - lo byte destination address
 *          1 - hi byte destination address
 *          2 - destination endpoint
 *          3 - lo byte cluster ID
 *          4 - hi byte cluster ID
 *          5 - message length
 *          6 - destination address mode (first byte of data)
 *          7 - zcl command frame
 *
 * @return  none
 */
static void zclSampleThermostat_ProcessAppMsg( uint8 srcEP, uint8 len, uint8 *msg )
{
  afAddrType_t dstAddr;
  uint16 clusterID;
  zclFrameHdr_t hdr;
  uint8 *pData;
  uint8 dataLen;

  dstAddr.addr.shortAddr = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  dstAddr.endPoint = *msg++;
  clusterID = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  dataLen = *msg++; // Length of message (Z-Tool can support up to 255 octets)
  dstAddr.addrMode = (afAddrMode_t)(*msg++);
  dataLen--; // Length of ZCL frame

  // Begining of ZCL frame
  pData = zclParseHdr( &hdr, msg );
  dataLen -= (uint8)( pData - msg );

  // Is this a foundation type message?
  if ( zcl_ProfileCmd( hdr.fc.type ) )
  {
    if ( hdr.fc.manuSpecific )
    {
      // We don't support any manufacturer specific command -- just forward it.
      zcl_SendCommand( srcEP, &dstAddr, clusterID, hdr.commandID, FALSE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr.fc.disableDefaultRsp, hdr.manuCode, hdr.transSeqNum, dataLen, pData );
    }
    else
    {
      zclParseCmd_t cmd;

      cmd.endpoint = srcEP;
      cmd.dataLen = dataLen;
      cmd.pData = pData;

      zclSampleThermostat_ProcessFoundationMsg( &dstAddr, clusterID, &hdr, &cmd );
    }
  }
  else
  {
    // Nope, must be specific to the cluster ID
    if ( hdr.fc.manuSpecific )
    {
      // We don't support any manufacturer specific command -- just forward it.
      zcl_SendCommand( srcEP, &dstAddr, clusterID, hdr.commandID, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr.fc.disableDefaultRsp, hdr.manuCode, hdr.transSeqNum, dataLen, pData );
    }
  }
}

/*********************************************************************
 * @fn      zclSampleThermostat_ProcessFoundationMsg
 *
 * @brief   Process Foundation message
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   dstAddr - where to send the request
 * @param   clusterID - real cluster ID
 * @param   hdr - pointer to the message header
 * @param   len - length of the received message
 * @param   data - received message
 *
 * @return  none
 */
static void zclSampleThermostat_ProcessFoundationMsg( afAddrType_t *dstAddr, uint16 clusterID,
                                                zclFrameHdr_t *hdr, zclParseCmd_t *pParseCmd )
{
#if defined(ZCL_READ) || defined(ZCL_WRITE) || defined(ZCL_REPORT) || defined(ZCL_DISCOVER)
  void *cmd;
#endif

  switch ( hdr->commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ:
      cmd = zclParseInReadCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendRead( SAMPLETHERMOSTAT_ENDPOINT, dstAddr, clusterID, (zclReadCmd_t *)cmd,
                      ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_READ

#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWrite( SAMPLETHERMOSTAT_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                       ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_WRITE_UNDIVIDED:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWriteUndivided( SAMPLETHERMOSTAT_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                                ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_WRITE_NO_RSP:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWriteNoRsp( SAMPLETHERMOSTAT_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                            ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_WRITE

#ifdef ZCL_REPORT
    case ZCL_CMD_CONFIG_REPORT:
      cmd = zclParseInConfigReportCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendConfigReportCmd( SAMPLETHERMOSTAT_ENDPOINT, dstAddr,  clusterID, (zclCfgReportCmd_t *)cmd,
                                 ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      cmd = zclParseInReadReportCfgCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendReadReportCfgCmd( SAMPLETHERMOSTAT_ENDPOINT, dstAddr, clusterID, (zclReadReportCfgCmd_t *)cmd,
                                  ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_REPORT:
      cmd = zclParseInReportCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendReportCmd( SAMPLETHERMOSTAT_ENDPOINT, dstAddr, clusterID, (zclReportCmd_t *)cmd,
                           ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_REPORT
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_ATTRS:
      cmd = zclParseInDiscAttrsCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendDiscoverAttrsCmd( SAMPLETHERMOSTAT_ENDPOINT, dstAddr, clusterID, (zclDiscoverAttrsCmd_t *)cmd,
                                  ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_DISCOVER

    default:
      // Unsupported command -- just forward it.
      zcl_SendCommand( pParseCmd->endpoint, dstAddr, clusterID, hdr->commandID, FALSE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr->fc.disableDefaultRsp, 0, hdr->transSeqNum, pParseCmd->dataLen, pParseCmd->pData );
      break;
  }
}

/*********************************************************************
 * @fn      zclSampleThermostat_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleThermostat_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network

  // restart device
  MT_SysCommandProcessing( aProcessCmd );
}

/*********************************************************************
 * @fn      zclSampleThermostat_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSampleThermostat_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSampleThermostat_IdentifyTime = pCmd->identifyTime;
  zclSampleThermostat_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSampleThermostat_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSampleThermostat_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
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
 * @fn      zclSampleThermostat_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleThermostat_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleThermostat_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleThermostat_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleThermostat_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleThermostat_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleThermostat_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleThermostat_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      zclSampleThermostat_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleThermostat_ProcessInDefaultRspCmd( pInMsg );
      break;

    default:
      break;
  }

  if ( pInMsg->attrCmd )
  {
    osal_mem_free( pInMsg->attrCmd );
  }
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleThermostat_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleThermostat_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleThermostat_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleThermostat_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

#ifdef ZCL_REPORT
/*********************************************************************
 * @fn      zclSampleThermostat_ProcessInReportCmd
 *
 * @brief   Process the "Profile" Report Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static void zclSampleThermostat_ProcessInReportCmd( zclIncomingMsg_t *pInMsg )
{
	uint8 src_EndPoint;
	
  zclReportCmd_t *pInReportCmd;
	
  zclReportCmd_t *pOutDemandReport;
  uint8 outDemandBuffer[sizeof( zclReportCmd_t ) + ( 3 * sizeof( zclReport_t ) )];
  bool send = TRUE;

  pInReportCmd = (zclReportCmd_t *)pInMsg->attrCmd;

	/*- Router: Data report ----------------------------------------------------*/
	#ifdef COORDINATOR
	if ( pInReportCmd->attrList[0].attrID == ATTRID_REPORT_DATA_COORD)
	{
		HalLedBlink ( HAL_LED_2, 1, 50, 500 );
		
		UART_ZCmdPrintString(HAL_UART_PORT_0, "@ZBR:");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, msg_RSSI);																			// RSSI
		UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[2].attrID);							// shortAddr
		UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[2].attrData[0]);						// endPoint
		UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[0].attrData[0]);					// temperature
		UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[0].attrData[1]);					// humidity
		UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[1].attrData[0]);					// heating
		UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[1].attrData[1]);					// cooling
		UART_ZCmdPrint		(HAL_UART_PORT_0, "!");
		return;
	}
	#endif
	
	/*- endDev Engine: Feedback ------------------------------------------------*/
	if ( pInReportCmd->attrList[0].attrID == ATTRID_HVAC_THERMOSTAT_RUNNING_STATE )
	{
		HalLedBlink ( HAL_LED_2, 1, 50, 500 );

		if (FLAG_ECHO_RDATA)
		{
			UART_ZCmdPrintString(HAL_UART_PORT_0, "@ZBF:");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, msg_RSSI);																		// RSSI
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInMsg->srcAddr.addr.shortAddr);							// shortAddr
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInMsg->srcAddr.endPoint);										// endPoint
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[0].attrData[0]); 			// Feedback Heating
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[0].attrData[1]); 			// Feedback Cooling
			UART_ZCmdPrint			(HAL_UART_PORT_0, "!");
		}
		
		return;
	}

	/*- endDev Sensor: Value Process -------------------------------------------*/
  if ( pInReportCmd->attrList[0].attrID == ATTRID_MS_TEMPERATURE_MEASURED_VALUE )
  {		
	
			UART_ZCmdPrintString(HAL_UART_PORT_0, "@ZBS:");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, msg_RSSI);																	
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInMsg->srcAddr.addr.shortAddr);
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[0].attrData[1]);
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[0].attrData[0]);	
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[1].attrData[1]);
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[1].attrData[0]);
			UART_ZCmdPrint		(HAL_UART_PORT_0, "!");
  }
	
	if (pInReportCmd->attrList[0].attrID == ATTRID_SENDSTATE)
	{	
		int i;
		uint8 add[16];
		UART_ZCmdPrintString(HAL_UART_PORT_0, "@ZBR:");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, msg_RSSI);																	
		UART_ZCmdPrintString(HAL_UART_PORT_0, ";"); 
		for(i = 0;i<16;i++)
		{
			add[i] = pInReportCmd->attrList[0].attrData[i];
		}
		add[16] == '\0';
		HalUARTWrite(HAL_UART_PORT_0, add,16); 
		UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[1].attrData[0]);
		UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[1].attrData[1]);
		UART_ZCmdPrint		(HAL_UART_PORT_0, "!");
		return;
	}

	// freedata
	if ( pInReportCmd->attrList[0].attrID == ATTRID_FREE_DATA )
	{
		uint8 i;
		uint8 tmp_Data[20];
		
		HalLedBlink ( HAL_LED_2, 1, 50, 500 );
		
		if (FLAG_ECHO_RDATA)
		{
			UART_ZCmdPrintString(HAL_UART_PORT_0, "@ZBR:");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, msg_RSSI);																// RSSI
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInMsg->srcAddr.addr.shortAddr);					// shortAddr
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInMsg->srcAddr.endPoint);								// endPoint
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[1].attrID); 				// coordShortAddr
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";"); 
			UART_ZCmdPrintNum 	(HAL_UART_PORT_0, pInReportCmd->attrList[1].attrData[0]); 	// data size
			UART_ZCmdPrintString(HAL_UART_PORT_0, ";");
			
			// eliminate DUMMY byte index[0]
			for (i = 1; i <= pInReportCmd->attrList[1].attrData[0]; i++)
			{
				tmp_Data[i-1] = pInReportCmd->attrList[0].attrData[i];
			}
			UART_ZCmdPrintBuffer(HAL_UART_PORT_0, tmp_Data, pInReportCmd->attrList[1].attrData[0]); 	// Free Data
			UART_ZCmdPrint			(HAL_UART_PORT_0, "!");
		}
		return;
	}


}
#endif  // ZCL_REPORT

/*********************************************************************
 * @fn      zclSampleThermostat_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleThermostat_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_EZMODE
/*********************************************************************
 * @fn      zclSampleThermostat_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleThermostat_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{
  zclEZMode_ActionData_t data;
  ZDO_MatchDescRsp_t *pMatchDescRsp;

  // Let EZ-Mode know of the Match Descriptor Response
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    pMatchDescRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pMatchDescRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free( pMatchDescRsp );
  }
}

/*********************************************************************
 * @fn      zclSampleThermostat_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - EZ-Mode state
 *          pData - data appropriate to state
 *
 * @return  none
 */
static void zclSampleThermostat_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    zclSampleThermostat_IdentifyTime = (EZMODE_TIME / 1000);  // convert to seconds
    zclSampleThermostat_ProcessIdentifyTimeChange();
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
    if ( pStr )
    {
      if ( giThermostatScreenMode == THERMOSTAT_MAINMODE )
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if ( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclSampleThermostat_IdentifyTime = 0;
    zclSampleThermostat_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if ( err == EZMODE_ERR_SUCCESS )
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
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    else
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giThermostatScreenMode == THERMOSTAT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif  // LCD_SUPPORTED

    // show main UI screen 3 seconds after completing EZ-Mode
    osal_start_timerEx( zclSampleThermostat_TaskID, SAMPLETHERMOSTAT_MAIN_SCREEN_EVT, 3000 );
  }
}
#endif // ZCL_EZMODE

/*******************************************************************************
********************************************************************************/
void zclSampleThermostat_BindingProcess(void)
{
	if ( ( giThermostatScreenMode == THERMOSTAT_MAINMODE ) ||
			 ( giThermostatScreenMode == THERMOSTAT_HELPMODE ) )
	{
		giThermostatScreenMode = THERMOSTAT_MAINMODE;

		FLAG_BINDING = !FLAG_BINDING;
	
	#ifdef ZCL_EZMODE
		zclEZMode_InvokeData_t ezModeData;
		static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_HVAC_THERMOSTAT };	 // only bind on the Thermostat cluster
	
		// Invoke EZ-Mode
		ezModeData.endpoint = SAMPLETHERMOSTAT_ENDPOINT; // endpoint on which to invoke EZ-Mode
		if ( ( zclSampleThermostat_NwkState == DEV_ZB_COORD ) ||
				 ( zclSampleThermostat_NwkState == DEV_ROUTER ) 	||
				 ( zclSampleThermostat_NwkState == DEV_END_DEVICE ) )
		{
			ezModeData.onNetwork = TRUE;			// node is already on the network
		}
		else
		{
			ezModeData.onNetwork = FALSE; 		// node is not yet on the network
		}
		ezModeData.initiator = TRUE;				// Thermostat is an initiator
		ezModeData.numActiveInClusters = 0;
		ezModeData.pActiveInClusterIDs = NULL;
		ezModeData.numActiveOutClusters = 1;	 // active output cluster
		ezModeData.pActiveOutClusterIDs = clusterIDs;
		zcl_InvokeEZMode( &ezModeData );
	
		#ifdef LCD_SUPPORTED
			HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
		#endif
	
		// NOT ZCL_EZMODE, use EndDeviceBind
	#else
		zAddrType_t dstAddr;
		HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
	
		// Initiate an End Device Bind Request, this bind request will
		// only use a cluster list that is important to binding.
		dstAddr.addrMode = afAddr16Bit;
		dstAddr.addr.shortAddr = 0; 	// Coordinator makes the match
		ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
													SAMPLETHERMOSTAT_ENDPOINT,
													ZCL_HA_PROFILE_ID,
													ZCLSAMPLETHERMOSTAT_BINDINGLIST_IN, bindingInClusters,
													ZCLSAMPLETHERMOSTAT_BINDINGLIST_OUT, bindingOutClusters,
													TRUE );
	#endif // ZCL_EZMODE
	}
	
}

#ifndef COORDINATOR
static void zclSampleThermostat_RollCall(void)
{

	afAddrType_t RollCall_DstAddr;
	// Set destination address to ZB Coordinator
  RollCall_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  RollCall_DstAddr.endPoint = 8;
  RollCall_DstAddr.addr.shortAddr = 0;
	
	zclReportCmd_t *pReportCmd;
	uint16 src_coordShortAddr = NLME_GetCoordShortAddr();
	uint8 device_type = 0;

	switch (zclSampleThermostat_NwkState)
	{
		case DEV_ZB_COORD:
			device_type = 1;
			break;

		case DEV_ROUTER:
			device_type = 2;
			break;

		case DEV_END_DEVICE:
			device_type = 3;
			break;

		default:
			device_type = 0;
			break;
	}
	
	pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + 2 * sizeof(zclReport_t) );
	if ( pReportCmd != NULL )
	{
	  pReportCmd->numAttr = 2;
		// Data
	  pReportCmd->attrList[0].attrID 		= ATTRID_ROLL_CALL;
	  pReportCmd->attrList[0].dataType 	= ZCL_DATATYPE_UINT16;
	  pReportCmd->attrList[0].attrData 	= (void *)(&src_coordShortAddr);
	  pReportCmd->attrList[1].attrID 		= ATTRID_ROLL_CALL;
	  pReportCmd->attrList[1].dataType 	= ZCL_DATATYPE_UINT8;
	  pReportCmd->attrList[1].attrData 	= (void *)(&device_type);

	  zcl_SendReportCmd( SAMPLETHERMOSTAT_ENDPOINT, &RollCall_DstAddr,
	                     ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
	                     pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSampleThermostatSeqNum++ );
	}

	osal_mem_free( pReportCmd );
}
#endif

void zclSampleThermostat_SendFreeData(void)
{
	#ifdef ZCL_REPORT
	// Can't send via indirect ??? (system reset)
	afAddrType_t DstAddr;
  DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  DstAddr.endPoint = 0xFF;
  DstAddr.addr.shortAddr = 0xFFFF;
	
  zclReportCmd_t *pReportCmd;
  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + 2 * sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 2;
		
		// Data
    pReportCmd->attrList[0].attrID 		= ATTRID_FREE_DATA;
    pReportCmd->attrList[0].dataType 	= ZCL_DATATYPE_CHAR_STR;
    pReportCmd->attrList[0].attrData 	= (void *)(Free_Data);
		// Endpoint and coordShortAddr
	pReportCmd->attrList[1].attrID 		= NLME_GetCoordShortAddr();
    pReportCmd->attrList[1].dataType 	= ZCL_DATATYPE_UINT8;
    pReportCmd->attrList[1].attrData 	= (void *)(&Free_Data_Size);

    zcl_SendReportCmd( SAMPLETHERMOSTAT_ENDPOINT, &DstAddr,
                       ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSampleThermostatSeqNum++ );
  }

  osal_mem_free( pReportCmd );
	#endif  // ZCL_REPORT
}
void zclSampleThermostat_SendC(void)
{
	char add[17];
	for(uint8 i = 1;i<=16;i++)
	{
		add[i-1] =  Free_Data[i];
	}
	for(uint8 i = 0; i < 16;i++) 
	{
	   	if( add[i] >= 'A' )
	  		add[i] = add[i] - '7';
	   	else 
	      	add[i] = add[i] - '0';
		if ( i%2 != 0 ) 
			add[i-1] = (add[i-1]<<4) + add[i];
 	}	
	#ifdef ZCL_REPORT
	// Can't send via indirect ??? (system reset)
	afAddrType_t DstAddr;
  DstAddr.addrMode = (afAddrMode_t)afAddr64Bit;
  DstAddr.endPoint = 0xFF;
  DstAddr.addr.shortAddr = 0xFFFF;
  DstAddr.addr.extAddr[0] = add[14];
  DstAddr.addr.extAddr[1] = add[12];
  DstAddr.addr.extAddr[2] = add[10];
  DstAddr.addr.extAddr[3] = add[8];
  DstAddr.addr.extAddr[4] = add[6];
  DstAddr.addr.extAddr[5] = add[4];
  DstAddr.addr.extAddr[6] = add[2];
  DstAddr.addr.extAddr[7] = add[0];
	
  zclReportCmd_t *pReportCmd;
  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + 2 * sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 2;
		
		// Data
    pReportCmd->attrList[0].attrID 		= ATTRID_CONTROL_S;
    pReportCmd->attrList[0].dataType 	= ZCL_DATATYPE_CHAR_STR;
    pReportCmd->attrList[0].attrData 	= (void *)(Free_Data);
		// Endpoint and coordShortAddr
	pReportCmd->attrList[1].attrID 		= NLME_GetCoordShortAddr();
    pReportCmd->attrList[1].dataType 	= ZCL_DATATYPE_UINT8;
    pReportCmd->attrList[1].attrData 	= (void *)(&Free_Data_Size);

    zcl_SendReportCmd( SAMPLETHERMOSTAT_ENDPOINT, &DstAddr,
                       ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSampleThermostatSeqNum++ );
  }

  osal_mem_free( pReportCmd );
	#endif  // ZCL_REPORT
}