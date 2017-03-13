/*******************************************************************************
 *                                            INCLUDES
 *******************************************************************************/
#include "MS_UART_CMD.h"
#include "MS_UART.h"
#include "string.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_hvac.h"
#include "zcl_ms.h"

#if 	(defined COORDINATOR) || (defined ROUTER)
	#include "zcl_samplethermostat.h" 
#elif (defined END_DEVICE_SENSOR)
	#include "zcl_sampletemperaturesensor.h"
#elif (defined END_DEVICE_ENGINE)
	#include "zcl_sampleheatingcoolingunit.h"
#endif

/*******************************************************************************
 *                                             MACROS
 *******************************************************************************/

/*******************************************************************************
 *                                            CONSTANTS
 *******************************************************************************/
#define UART_CMD_BUF_SIZE_128           			128

#define CMD_CHECK_ALIVE												0
#define CMD_BINDING_START											1
#define CMD_BINDING_STOP											2
#define CMD_RETURN_SHORT_ADDRESS							3
#define CMD_RETURN_COORD_SHORT_ADDRESS				4
#define CMD_ENABLE_ECHO_SDATA									5
#define CMD_DISABLE_ECHO_SDATA								6
#define CMD_ENABLE_ECHO_RDATA									7
#define CMD_DISABLE_ECHO_RDATA								8
#define CMD_SEND_FREE_DATA										10
#define CMD_SEND_CONTROL                              11


/*******************************************************************************
 *                                             TYPEDEFS
 *******************************************************************************/

/*******************************************************************************
 *                                         GLOBAL VARIABLES
 *******************************************************************************/
static uint16 packageLength = 0;
char* pcmdData;

bool FLAG_ECHO_SDATA 	= FALSE;
bool FLAG_ECHO_RDATA 	= FALSE;

char		Free_Data[FREE_DATA_BFR_SIZE + 1];
uint8 	Free_Data_Size;

/*******************************************************************************
 *                                          FUNCTIONS - External
 *******************************************************************************/

/*******************************************************************************
 *                                          FUNCTIONS - Local
 *******************************************************************************/
static char* ZCMD_MatchCMD(char* buf, char* str);
static uint8 ZCMD_FindChrStr(char* buf, uint8 chr);
static void ZCMD_ProcessCMD(uint8 CMD);
/*******************************************************************************
 *                                          FUNCTIONS - API
 *******************************************************************************/
void ZCMD_ReplyCMD(void)
{
	char Rx0_tmpBuffer[UART_CMD_BUF_SIZE_128];
	packageLength = UART_ParseLength(HAL_UART_PORT_0);
	
	UART_GetData(HAL_UART_PORT_0, (uint8*)Rx0_tmpBuffer, packageLength);
	UART_DebugPrintNum(HAL_UART_PORT_0, packageLength);
	UART_DebugPrint(HAL_UART_PORT_0, " ");

	if ( Rx0_tmpBuffer[0] == '@' && Rx0_tmpBuffer[1] == 'Z' &&
		 	 Rx0_tmpBuffer[2] == 'B' && Rx0_tmpBuffer[packageLength-1] == '!' &&
		 	 ZCMD_FindChrStr(Rx0_tmpBuffer, '!') )
	{
		// Check alive
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB!"))
		{
			ZCMD_ProcessCMD(CMD_CHECK_ALIVE);
			return;
		}
		
		// Binding STOP
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+BIND=0!"))
		{
			ZCMD_ProcessCMD(CMD_BINDING_STOP);
			return;
		}
		
		// Binding START
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+BIND=1!"))
		{
			ZCMD_ProcessCMD(CMD_BINDING_START);
			return;
		}
		
		// Get Short Address
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+SHORTADDR!"))
		{
			ZCMD_ProcessCMD(CMD_RETURN_SHORT_ADDRESS);
			return;
		}

		// Get Coord Short Address
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+COORDSHORTADDR!"))
		{
			ZCMD_ProcessCMD(CMD_RETURN_COORD_SHORT_ADDRESS);
			return;
		}

		// Disable Echo SEND Data
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+ECHOSDATA=0!"))
		{
			ZCMD_ProcessCMD(CMD_DISABLE_ECHO_SDATA);
			return;
		}

		// Enable Echo SEND Data
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+ECHOSDATA=1!"))
		{		
			ZCMD_ProcessCMD(CMD_ENABLE_ECHO_SDATA);
			return;
		}

		// Disable Echo RECEIVE Data
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+ECHORDATA=0!"))
		{
			ZCMD_ProcessCMD(CMD_DISABLE_ECHO_RDATA);
			return;
		}

		// Enable Echo RECEIVE Data
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+ECHORDATA=1!"))
		{		
			ZCMD_ProcessCMD(CMD_ENABLE_ECHO_RDATA);
			return;
		}

		// Send FREE DATA - Only for ZB Coordinator
		if (ZCMD_MatchCMD(Rx0_tmpBuffer, "@ZB+DATA="))
		{		
			ZCMD_ProcessCMD(CMD_SEND_FREE_DATA);
			return;
		}
		// control led -  only for ZBC
		if (ZCMD_MatchCMD(Rx0_tmpBuffer,"@ZB+CONTROL="))
		{
			ZCMD_ProcessCMD(CMD_SEND_CONTROL);
			return;
		}
		
	}
	UART_ZCmdPrint(HAL_UART_PORT_0, "ERROR");		
}

void ZCMD_ProcessCMD(uint8 CMD)
{
	switch (CMD)
	{
		case CMD_CHECK_ALIVE:
			UART_ZCmdPrint(HAL_UART_PORT_0, "OK");
			break;
			
		case CMD_BINDING_STOP:
			if (FLAG_BINDING)
			{
				#if 	(defined COORDINATOR) || (defined ROUTER)
					zclSampleThermostat_BindingProcess();
				#elif (defined END_DEVICE_SENSOR)
					zclSampleTemperatureSensor_BindingProcess();
				#elif (defined END_DEVICE_ENGINE)
					zclSampleHeatingCoolingUnit_BindingProcess();
				#endif
				FLAG_HARD_BINDING = FALSE;
				UART_ZCmdPrint(HAL_UART_PORT_0, "BINDING STOPPED");
			}
			else
			{
				UART_ZCmdPrint(HAL_UART_PORT_0, "BINDING STOP ALREADY");
			}
			break;

		case CMD_BINDING_START:
			if (!FLAG_BINDING)
			{
				#if 	(defined COORDINATOR) || (defined ROUTER)
					zclSampleThermostat_BindingProcess();
				#elif (defined END_DEVICE_SENSOR)
					zclSampleTemperatureSensor_BindingProcess();
				#elif (defined END_DEVICE_ENGINE)
					zclSampleHeatingCoolingUnit_BindingProcess();
				#endif
				FLAG_HARD_BINDING = TRUE;
				UART_ZCmdPrint(HAL_UART_PORT_0, "BINDING STARTED");
			}
			else
			{					
				UART_ZCmdPrint(HAL_UART_PORT_0, "BINDING START ALREADY");
			}
			
			break;

		case CMD_RETURN_SHORT_ADDRESS:
			UART_ZCmdPrintNum(HAL_UART_PORT_0, NLME_GetShortAddr());
			UART_ZCmdPrintString(HAL_UART_PORT_0, "\r\n");
			break;

		case CMD_RETURN_COORD_SHORT_ADDRESS:
			#ifndef COORDINATOR
				UART_ZCmdPrintNum(HAL_UART_PORT_0, NLME_GetCoordShortAddr());
				UART_ZCmdPrintString(HAL_UART_PORT_0, "\r\n");
			#else
				UART_ZCmdPrint(HAL_UART_PORT_0, "COORDINATOR!");
			#endif		

			break;
			
		case CMD_ENABLE_ECHO_SDATA:
			FLAG_ECHO_SDATA = TRUE;
			UART_ZCmdPrint(HAL_UART_PORT_0, "OK");			
			break;

		case CMD_DISABLE_ECHO_SDATA:
			FLAG_ECHO_SDATA = FALSE;
			UART_ZCmdPrint(HAL_UART_PORT_0, "OK");		
			break;

		case CMD_ENABLE_ECHO_RDATA:
			FLAG_ECHO_RDATA = TRUE;
			UART_ZCmdPrint(HAL_UART_PORT_0, "OK");
			break;

		case CMD_DISABLE_ECHO_RDATA:
			FLAG_ECHO_RDATA = FALSE;
			UART_ZCmdPrint(HAL_UART_PORT_0, "OK");
			break;

		case CMD_SEND_FREE_DATA:
			{
				#ifdef COORDINATOR
				uint8 i = 0;
				uint8 parseStr_len = osal_strlen("@ZB+DATA=");
				Free_Data_Size = packageLength - (parseStr_len + 1); 			// +  "!" ->  + 1
				Free_Data[0] = '0'; 																			// DUMMY Byte

				if ( Free_Data_Size <= FREE_DATA_BFR_SIZE )
				{	
					
					for (i = 1; i <= FREE_DATA_BFR_SIZE; i++)
					{
						if (i <= Free_Data_Size)
						{
							Free_Data[i] = *(pcmdData + parseStr_len + i - 1);
						}
						else
						{
							Free_Data[i] = ' ';
						}
					}
					zclSampleThermostat_SendFreeData();
					
					UART_ZCmdPrint(HAL_UART_PORT_0, "OK");
				}
				else
				{
					UART_ZCmdPrint(HAL_UART_PORT_0, "ERROR");
				}
				#else
					UART_ZCmdPrint(HAL_UART_PORT_0, "NOT SUPPORT");
				#endif
			}
			break;
		case CMD_SEND_CONTROL:
			{
				#ifdef COORDINATOR
				uint8 i = 0;
				uint8 parseStr_len = osal_strlen("@ZB+CONTROL=");
				Free_Data_Size = packageLength - (parseStr_len + 1); 			// +  "!" ->  + 1
				Free_Data[0] = '0'; 																			// DUMMY Byte

				if ( Free_Data_Size <= FREE_DATA_BFR_SIZE )
				{	
					
					for (i = 1; i <= FREE_DATA_BFR_SIZE; i++)
					{
						if (i <= Free_Data_Size)
						{
							Free_Data[i] = *(pcmdData + parseStr_len + i - 1);
						}
						else
						{
							Free_Data[i] = ' ';
						}
					}
					zclSampleThermostat_SendControlData();
					
					UART_ZCmdPrint(HAL_UART_PORT_0, "DONE");
				}
				else
				{
					UART_ZCmdPrint(HAL_UART_PORT_0, "ERROR");
				}
				#else
					UART_ZCmdPrint(HAL_UART_PORT_0, "ONLY ZBC");
				#endif
			}
			break;		

		default:
			break;
	}
}

char* ZCMD_MatchCMD(char* buf, char* str)
{
	pcmdData = strstr(buf, str);
	return pcmdData;
}

uint8 ZCMD_FindChrStr(char* buf, uint8 chr)
{
	uint8 i, num;

	num = 0;
	for (i = 0; i < packageLength; i++)
	{
		if (buf[i] == chr)
		{
			num++;
		}
	}

	if (num == 1)
	{
		return 1;
	}

	return 0;
}

/*******************************************************************************
********************************************************************************/
