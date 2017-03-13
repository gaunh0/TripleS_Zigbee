/***************************************************************************************************
 *                                            INCLUDES
 ***************************************************************************************************/
#include "MS_DHT11.h"
#include "hal_timer.h"
/***************************************************************************************************
 *                                             MACROS
 ***************************************************************************************************/

/***************************************************************************************************
 *                                            CONSTANTS
 ***************************************************************************************************/
 
/***************************************************************************************************
 *                                             TYPEDEFS
 ***************************************************************************************************/

/***************************************************************************************************
 *                                         GLOBAL VARIABLES
 ***************************************************************************************************/
uint8 OneWireDataBuffer[5] = {0,0,0,0,0};
uint8 humidityValue =0;
uint8 tempValue = 1;

uint8 bitmask = 7;
uint8 byteIndex = 0;
uint8 loopCount = 0;
uint8 _index = 0;

/***************************************************************************************************
 *                                          FUNCTIONS - External
 ***************************************************************************************************/

/***************************************************************************************************
 *                                          FUNCTIONS - Local
 ***************************************************************************************************/

void DHT11_SetDataPinInput(void);
void DHT11_SetDataPinOutput(void);

void DHT11_SetDataPinInput(void)
{
	/* Select GPIO direction : input*/
	P0DIR &= ~(DHT11_Data_BIT);
}
void DHT11_SetDataPinOutput(void)
{
	/* Select GPIO direction : output */
	P0DIR |= DHT11_Data_BIT;
}

/***************************************************************************************************
 *                                          FUNCTIONS - API
 ***************************************************************************************************/
void DHT11_Init(void)
{
	/* Select general purpose on I/O pins. */
	P0SEL &= ~(DHT11_Data_BIT);

	/* Select GPIO direction : output */
	DHT11_SetDataPinOutput();

	DHT11_Data = 1;
}

uint8 DHT11_ReadSensor(void)
{
	DHT11_SetDataPinOutput();
	DHT11_Data = 0;
	DelayMs(DHTLIB_DHT11_WAKEUP);
	DHT11_Data = 1;
	//disableInterrupts();
	DelayUs(10);
	DHT11_SetDataPinInput();
	
	loopCount = 0;
	while (DHT11_Data == 0)
	{
		loopCount ++;
		if (loopCount > DHTLIB_TIMEOUT)
		{
      //enableInterrupts();
      return DHTLIB_ERROR_ACK_L;
		}
	}
	
	loopCount = 0;
	while (DHT11_Data != 0)
	{
		loopCount ++;
		if (loopCount > DHTLIB_TIMEOUT)
		{
	    //enableInterrupts();
	    return DHTLIB_ERROR_ACK_H;
		}	
	}
	
	while (_index < 40)
	{
		loopCount = 0;
		while (DHT11_Data == 0)
		{
			asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
			loopCount ++;
			if (loopCount > DHTLIB_TIMEOUT)
			{
        //enableInterrupts();
        return DHTLIB_ERROR_TIMEOUT;
			}
		}
		
		loopCount = 0;
		while (DHT11_Data != 0)
		{
			asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
			loopCount ++;
			if (loopCount > DHTLIB_TIMEOUT)
			{
        //enableInterrupts();
        return DHTLIB_ERROR_TIMEOUT;
			}
		}
		
		if (loopCount > 20)
		{
			OneWireDataBuffer[byteIndex] |= (1 << bitmask);		
		}
		if (bitmask == 0)   // next byte			
		{
			bitmask = 7;
			byteIndex++;
		}
		else
		{
			bitmask = bitmask - 1;
		}
		_index = _index + 1;
	}
	
	DHT11_SetDataPinOutput();
	DHT11_Data = 1;
	//enableInterrupts();
	byteIndex = 0;
	_index = 0;
	
	return DHTLIB_OK;
}

uint8 DHT11_GetValue(void)
{
	uint8 checksum = 0;
	uint8 ret_val;
	ret_val = DHT11_ReadSensor();
	
	if (ret_val == DHTLIB_OK)
	{	
		if (OneWireDataBuffer[4] == ((OneWireDataBuffer[0]+ OneWireDataBuffer[2])&0xFF))
		{
			humidityValue = OneWireDataBuffer[0] & 0x7F;
			tempValue = OneWireDataBuffer[2] & 0x7F;
      checksum = 1;
		}
	}
	OneWireDataBuffer[0] = 0;
	OneWireDataBuffer[2] = 0;
	OneWireDataBuffer[4] = 0;
  return checksum;
}

uint8 DHT11_GetHumidityValue(void)
{
	return humidityValue;
}
uint8 DHT11_GetTempValue(void)
{
	return tempValue;
}

int16 DHT11_GetPackageValue(void)
{
	uint8 tmp_temperature;
	uint8 tmp_humidity;
	int16 tmp_package;

//	DHT11_GetValue();
	tmp_temperature = DHT11_GetTempValue();
	tmp_humidity		= DHT11_GetHumidityValue();
	tmp_package = BUILD_UINT16(tmp_temperature, tmp_humidity);
	return tmp_package;
}


/***************************************************************************************************
***************************************************************************************************/
void DelayUs(uint16 microSecs)
{
	while(microSecs--)
	{
		/* 32 NOPs == 1 usecs */
		asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
		asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
//		asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
//		asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
//		asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
//		asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
//		asm("nop"); asm("nop");
	}
}

void DelayMs(uint16 miliSecs)
{
	uint16 i = 0;
	
	for (i = 0; i < 1000; i++)
	{
		DelayUs(miliSecs);
	}
}
	

