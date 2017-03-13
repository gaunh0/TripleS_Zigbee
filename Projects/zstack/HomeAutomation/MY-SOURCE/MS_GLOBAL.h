#ifndef MS_GLOBAL_H
#define MS_GLOBAL_H

#ifdef __cplusplus
extern "C"
{
#endif
/*******************************************************************************
 *                                            INCLUDES
 *******************************************************************************/
#include "OnBoard.h"
#include "hal_board.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_timer.h"
#include "hal_uart.h"

/*******************************************************************************
 *                                             MACROS
 *******************************************************************************/

/*******************************************************************************
 *                                            CONSTANTS
 *******************************************************************************/
/*- Attribute ----------------------------------------------------------------*/
#define 		 	ATTRID_ENDPOINT 				0xA001
#define 			ATTRID_ROLL_CALL 				0xA002
#define 		 	ATTRID_FREE_DATA 				0xA003
#define 		 	ATTRID_REPORT_DATA_COORD 	    0xA004
#define				ATTRID_SENDSTATE				0xA006
#define 			ATTRID_CONTROL_DATA             0xA007
#define 			ATTRID_CONTROL_S                0xA008

/*- Period Event -------------------------------------------------------------*/
#define				CHECK_SYSTEM_EVT_PERIOD 			 	10000

/*- Global Endpoint ----------------------------------------------------------*/
#define				APP_END_DEVICE_ENDPOINT			1
#define				APP_ROUTER_ENDPOINT					1
#define				APP_COORDINATOR_ENDPOINT		8

/*******************************************************************************
 *                                             TYPEDEFS
 *******************************************************************************/

/*******************************************************************************
 *                                         GLOBAL VARIABLES
 *******************************************************************************/
extern bool FLAG_JOIN_CONFIRM;
extern bool FLAG_BINDING;
extern bool FLAG_HARD_BINDING;

extern uint8 COUNT_TO_ROLL_CALL;

extern int8 msg_RSSI;

/*******************************************************************************
 *                                          FUNCTIONS - API
 *******************************************************************************/

/*******************************************************************************
*******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif

