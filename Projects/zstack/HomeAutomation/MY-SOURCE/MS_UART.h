#ifndef MS_UART_H
#define MS_UART_H

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
#define RX_BUFFER_SIZE											128
#define UART_PARSE_RX_PACKAGE_EVT_PERIOD		10
/*******************************************************************************
 *                                             TYPEDEFS
 *******************************************************************************/
// UART Rx Data Format
typedef struct
{
	uint16			idxRead;
	uint16			idxWrite;
	uint16			DataAvailable;
	uint8				ParseLength;
	uint8 			CircularBuffer[RX_BUFFER_SIZE];
} UartRxData_t;

/*******************************************************************************
 *                                         GLOBAL VARIABLES
 *******************************************************************************/

/*******************************************************************************
 *                                          FUNCTIONS - API
 *******************************************************************************/
extern void UART_Init(uint8 port);
extern void UART_SendString(uint8 port, uint8 *buf);
extern void UART_SendNum(uint8 port, long num);
extern void UART_DebugPrint(uint8 port, uint8 *buf);
extern void UART_DebugPrintNum(uint8 port, long num);
extern void UART_DebugPrintLCD(uint8 port, uint8 Row, uint8 Col, uint8 *buf);
extern void UART_DebugPrintLCDNum(uint8 port, uint8 Row, uint8 Col, long Num);

extern void UART_ZCmdPrint(uint8 port, uint8 *buf);
extern void UART_ZCmdPrintNum(uint8 port, long num);
extern void UART_ZCmdPrintBuffer(uint8 port, uint8 *buf, uint8 length);
extern void UART_ZCmdPrintString(uint8 port, uint8 *buf);

extern uint8* UART_GetData(uint8 port, uint8* buffer, uint8 length);
extern uint8 UART_DataAvailable(uint8 port);

extern uint8 UART_ParseRxPackage(uint8 port);
extern uint8 UART_ParseLength(uint8 port);

/*******************************************************************************
*******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif

