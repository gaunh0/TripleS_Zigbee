/*******************************************************************************
 *                                            INCLUDES
 *******************************************************************************/
#include "MS_UART.h"

/*******************************************************************************
 *                                             MACROS
 *******************************************************************************/

/*******************************************************************************
 *                                            CONSTANTS
 *******************************************************************************/
#define UART0_MAX_TX_BUF_SIZE_128           128
#define UART0_MAX_TX_BUF_SIZE_256           256
#define UART0_MAX_RX_BUF_SIZE_128           128
#define UART0_MAX_RX_BUF_SIZE_256           256

#define UART1_MAX_TX_BUF_SIZE_128           128
#define UART1_MAX_TX_BUF_SIZE_256           256
#define UART1_MAX_RX_BUF_SIZE_128           128
#define UART1_MAX_RX_BUF_SIZE_256           256

/*******************************************************************************
 *                                             TYPEDEFS
 *******************************************************************************/

/*******************************************************************************
 *                                         GLOBAL VARIABLES
 *******************************************************************************/
static UartRxData_t Rx0_Data;

/*******************************************************************************
 *                                          FUNCTIONS - External
 *******************************************************************************/

/*******************************************************************************
 *                                          FUNCTIONS - Local
 *******************************************************************************/
static void UART0_RxProcessCB(uint8 port, uint8 event);
static void UART1_RxProcessCB(uint8 port, uint8 event);

static void UART0_WriteDataToRxBuffer(uint8* buffer, uint8 length);

static long uart_pow_of(uint8 A, uint8 n);

/*******************************************************************************
 *                                          FUNCTIONS - API
 *******************************************************************************/
void UART_Init(uint8 port)
{
	halUARTCfg_t uartConfig;

	if (port == HAL_UART_PORT_0)
	{
		uartConfig.configured = TRUE;
		uartConfig.baudRate = HAL_UART_BR_115200;
		uartConfig.flowControl = HAL_UART_FLOW_OFF;
		uartConfig.flowControlThreshold = 0;
		uartConfig.rx.maxBufSize = UART0_MAX_RX_BUF_SIZE_128;
		uartConfig.tx.maxBufSize = UART0_MAX_TX_BUF_SIZE_128;
		uartConfig.idleTimeout = 1; 	
		uartConfig.intEnable = TRUE;	
		uartConfig.callBackFunc = UART0_RxProcessCB;
	}
	else if (port == HAL_UART_PORT_1)
	{
		uartConfig.configured = TRUE;
		uartConfig.baudRate = HAL_UART_BR_115200;
		uartConfig.flowControl = HAL_UART_FLOW_OFF;
		uartConfig.flowControlThreshold = 0;
		uartConfig.rx.maxBufSize = UART1_MAX_RX_BUF_SIZE_256;
		uartConfig.tx.maxBufSize = UART1_MAX_TX_BUF_SIZE_128;
		uartConfig.idleTimeout = 1; 	
		uartConfig.intEnable = TRUE;	
		uartConfig.callBackFunc = UART1_RxProcessCB;
	}
	HalUARTOpen (port, &uartConfig);
}

void UART_SendString(uint8 port, uint8 *buf)
{
	HalUARTWrite(port, buf, osal_strlen((char*)buf));
}

void UART_SendNum(uint8 port, long num)
{
	uint8 i;
	uint8 num_flag = 0;
	uint8 tmp_chr[1];
	
	if (num == 0)
	{
		HalUARTWrite(port, "0", 1);
		return;
	}
	if (num < 0)
	{
		HalUARTWrite(port, "-", 1);
		num *= -1;
	}

	for ( i = 9; i > 0; i--)
	{
		if ((num / uart_pow_of(10, i-1)) != 0)
		{
			num_flag = 1;
			tmp_chr[0] = num/uart_pow_of(10, i-1) + '0';
			HalUARTWrite(port, tmp_chr, 1);
		}
		else
		{
			if (num_flag != 0)
			{
				HalUARTWrite(port, "0", 1);
			}
		}
		
		num %= uart_pow_of(10, i-1);
	}
}

long uart_pow_of(uint8 A, uint8 n)
{
	uint8 i;
	uint32 temp = 1;
	
	for (i = 0; i < n; i++)
	{
		temp *= A;
	}
		
	return temp;
}

void UART_DebugPrint(uint8 port, uint8* buf)
{
	#if (defined UART_DEBUG_TERMINAL) && (UART_DEBUG_TERMINAL == TRUE)
	HalUARTWrite(port, buf, osal_strlen(buf));
	HalUARTWrite(port, "\r\n", 2);	
	#endif
}

void UART_DebugPrintNum(uint8 port, long num)
{
	#if (defined UART_DEBUG_TERMINAL) && (UART_DEBUG_TERMINAL == TRUE)
	UART_SendNum(port, num);
	#endif
}

void UART_ZCmdPrint(uint8 port, uint8 *buf)
{
	#if (defined UART_ZCMD) && (UART_ZCMD == TRUE)
	HalUARTWrite(port, buf, osal_strlen((char*)buf));
	HalUARTWrite(port, "\r\n", 2);	
	#endif
}

void UART_ZCmdPrintBuffer(uint8 port, uint8 *buf, uint8 length)
{
	#if (defined UART_ZCMD) && (UART_ZCMD == TRUE)
	HalUARTWrite(port, buf, length);
	#endif
}

void UART_ZCmdPrintNum(uint8 port, long num)
{
	#if (defined UART_ZCMD) && (UART_ZCMD == TRUE)
	UART_SendNum(port, num);
	#endif
}

void UART_ZCmdPrintString(uint8 port, uint8 *buf)
{
	#if (defined UART_ZCMD) && (UART_ZCMD == TRUE)
	HalUARTWrite(port, buf, osal_strlen((char*)buf));	
	#endif
}

void UART_DebugPrintLCD(uint8 port, uint8 Row, uint8 Col, uint8 *buf)
{
	#if (defined UART_DEBUG_LCD) && (UART_DEBUG_LCD == TRUE)
	uint8 Length = osal_strlen((char*)buf);
	
	UART_SendString(port, "@LCD1602");
	
	if (Row < 10)
		UART_SendString(port, "0");
	UART_SendNum(port, Row);
	if (Col < 10)
		UART_SendString(port, "0");
	UART_SendNum(port, Col);

	if (Length < 10)
		UART_SendString(port, "0");
	UART_SendNum(port, Length);
	
	HalUARTWrite(port, buf, Length);
	
	UART_SendString(port, "@");
	
	#endif
}

void UART_DebugPrintLCDNum(uint8 port, uint8 Row, uint8 Col, long Num)
{
	#if (defined UART_DEBUG_LCD) && (UART_DEBUG_LCD == TRUE)
	uint8 numNum = 0;
	uint16 tmpNum = Num;
	
	UART_SendString(port, "@LCD1602");
	
	if (Row < 10)
		UART_SendString(port, "0");
	UART_SendNum(port, Row);
	if (Col < 10)
		UART_SendString(port, "0");
	UART_SendNum(port, Col);

	while (tmpNum != 0)
	{
		numNum++;
		tmpNum = tmpNum/10;
	}
	
	if (numNum < 10)
		UART_SendString(port, "0");
	if (Num != 0)
		UART_SendNum(port, numNum);
	else
		UART_SendNum(port, 1);

	UART_SendNum(port, Num);
	UART_SendString(port, "@");
	
	#endif
}

void UART0_WriteDataToRxBuffer(uint8 *buffer, uint8 length)
{
	uint8 tmp_Length = 0;

	while (tmp_Length < length)
	{
		Rx0_Data.CircularBuffer[Rx0_Data.idxWrite] = *(buffer + tmp_Length);
		Rx0_Data.idxWrite = (Rx0_Data.idxWrite + 1) % RX_BUFFER_SIZE;
		if (++Rx0_Data.DataAvailable == RX_BUFFER_SIZE)
		{
			Rx0_Data.DataAvailable = 0;
		}
		tmp_Length++;
	}
}

uint8 UART_DataAvailable(uint8 port)
{
	if ( port == HAL_UART_PORT_0 )
	{
		return Rx0_Data.DataAvailable;
	}
	else if ( port == HAL_UART_PORT_1 )
	{

	}
	return 0;
}

uint8* UART_GetData(uint8 port, uint8* buffer, uint8 length)
{
	uint8 tmp_Length = 0;
	
	while (tmp_Length < length)
	{
		*(buffer + tmp_Length) = Rx0_Data.CircularBuffer[Rx0_Data.idxRead];
		Rx0_Data.DataAvailable--;
		Rx0_Data.idxRead = (Rx0_Data.idxRead + 1) % RX_BUFFER_SIZE;
		if (Rx0_Data.idxRead == Rx0_Data.idxWrite)
		{
			Rx0_Data.DataAvailable = 0;
			break;
		}
		tmp_Length++;
	}
	return buffer;
}

static void UART0_RxProcessCB(uint8 port, uint8 event)
{
	uint16 Rx0_tmpBufLen;
	uint8 Rx0_tmpBuffer[UART0_MAX_RX_BUF_SIZE_128];
	
	switch (event)
	{
		case HAL_UART_RX_FULL:
			break;
		case HAL_UART_RX_ABOUT_FULL:
			break;
		case HAL_UART_RX_TIMEOUT:
			Rx0_tmpBufLen = Hal_UART_RxBufLen(port);
			HalUARTRead(HAL_UART_PORT_0, Rx0_tmpBuffer, Rx0_tmpBufLen);
			UART0_WriteDataToRxBuffer(Rx0_tmpBuffer, Rx0_tmpBufLen);			
			break;			
	}
}

static void UART1_RxProcessCB(uint8 port, uint8 event)
{
	switch (event)
	{
		case HAL_UART_RX_FULL:
			break;
		case HAL_UART_RX_ABOUT_FULL:
			break;
		case HAL_UART_RX_TIMEOUT:
			break;
	}
}

uint8 UART_ParseRxPackage(uint8 port)
{
	if (port == HAL_UART_PORT_0)
	{
		uint8 i = 0;
		uint8 j = 0;
		uint16 dataAvailable = UART_DataAvailable(HAL_UART_PORT_0);
		
		if (dataAvailable)
		{
			for (i = 0; i < dataAvailable; i++)
			{
				if ( Rx0_Data.CircularBuffer[(Rx0_Data.idxRead + i) % RX_BUFFER_SIZE] == '@' )
				{
					for (j = i + 1; j < dataAvailable; j++)
					{
						if ( Rx0_Data.CircularBuffer[(Rx0_Data.idxRead + j) % RX_BUFFER_SIZE] == '!' )
						{
							Rx0_Data.ParseLength = j + 1;
							return 1;
						}
					}
				}
				else
				{
					Rx0_Data.idxRead = (Rx0_Data.idxRead + 1) % RX_BUFFER_SIZE;
					Rx0_Data.DataAvailable--;
					if ( Rx0_Data.idxRead == Rx0_Data.idxWrite )
					{
						Rx0_Data.DataAvailable = 0;
					}
				}
			}
		}
		Rx0_Data.ParseLength = 0;
	}
	else if (port == HAL_UART_PORT_1)
	{
    
	}
  return 0;
}

uint8 UART_ParseLength(uint8 port)
{
	uint8 tmp_ParseLength;

	if ( port == HAL_UART_PORT_0 )
	{
		tmp_ParseLength = Rx0_Data.ParseLength;
		Rx0_Data.ParseLength = 0;
		return tmp_ParseLength;
	}
	else if ( port == HAL_UART_PORT_1 )
	{
	
	}
	return 0;
}
/*******************************************************************************
********************************************************************************/