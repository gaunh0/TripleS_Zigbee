#ifndef MS_DHT11_H
#define MS_DHT11_H

#ifdef __cplusplus
extern "C"
{
#endif
/***************************************************************************************************
 *                                            INCLUDES
 ***************************************************************************************************/
#include "OnBoard.h"
#include "hal_board.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_timer.h"


/***************************************************************************************************
 *                                             MACROS
 ***************************************************************************************************/

/***************************************************************************************************
 *                                            CONSTANTS
 ***************************************************************************************************/
#define DHT11_Data         						P0_7
#define DHT11_Data_BIT         				BV(7)

#define DHTLIB_OK                   	0
#define DHTLIB_ERROR_CHECKSUM       	1
#define DHTLIB_ERROR_TIMEOUT        	2
#define DHTLIB_ERROR_CONNECT        	3
#define DHTLIB_ERROR_ACK_L          	4
#define DHTLIB_ERROR_ACK_H          	5

#define DHTLIB_DHT11_WAKEUP         	18
#define DHTLIB_DHT_WAKEUP           	1
#define DHTLIB_DHT11_LEADING_ZEROS  	1
#define DHTLIB_DHT_LEADING_ZEROS    	6

#define DHTLIB_TIMEOUT 								100

/***************************************************************************************************
 *                                             TYPEDEFS
 ***************************************************************************************************/

/***************************************************************************************************
 *                                         GLOBAL VARIABLES
 ***************************************************************************************************/

/***************************************************************************************************
 *                                          FUNCTIONS - API
 ***************************************************************************************************/

extern void  DHT11_Init(void);
extern uint8 DHT11_ReadSensor(void);
extern uint8 DHT11_GetValue(void);
extern uint8 DHT11_GetHumidityValue(void);
extern uint8 DHT11_GetTempValue(void);
extern int16 DHT11_GetPackageValue(void);

extern void DelayUs(uint16 microSecs);
extern void DelayMs(uint16 miliSecs);


/***************************************************************************************************
***************************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif

