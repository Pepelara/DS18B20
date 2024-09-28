# DS18B20

`DS18B20` is an C library for controlling DS18B20 temperature sensors.
It can be easily compiled for every micro-controller supported by `gcc`.

This library is based on https://github.com/Jacajack/avr-ds18b20

## Usage example

```c
#include "main.h"
#include "ds18b20.h"
//#include "stm32g0xx_hal.h"

int main( )
{
	
	onewire_lib_t myLib;
    myLib.user_pinMode = mypinMode;
    myLib.user_writePin = HAL_GPIO_WritePin;
    myLib.user_readPin = HAL_GPIO_ReadPin;
    myLib.user_portDelay = mybitDelay;
    myLib.disable_irq = __disable_irq;
    myLib.enable_irq = __enable_irq;
    onewire_lib_init(&myLib);
    
    onewire_bus_t mybus;
    mybus.GPIOx = GPIOA;
    mybus.pin = GPIO_PIN_4;
    
    ds18_dev_t myds18[2] = {0};
    
    ds18_search(&mybus, &myds18[0], &count, 2);
    
    
	int32_t temp1, temp2;
	
	while ( 1 )
	{
		//Start conversion (without ROM matching)
		ds18_convert(&mybus, NULL); //This sends convert to all

		//Delay (sensor needs time to perform conversion)
		HAL_Delay( 1000 );

		//Read temperature (without ROM matching)
		ds18_read(&mybus, &myds18[0], &temp1);
		ds18_read(&mybus, &myds18[1], &temp2);

		//Somehow use data stored in `temp` variable
		printf("temp1: %d\n", temp1);
		printf("temp2: %d\n\n", temp2);
	}

	return 0;
}


void mypinMode(GPIO_TypeDef *GPIOx, int pin, int mode)
{
    
    //This is stm32 example. Do your own low level init.
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    //HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode == 0 ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT_PP; //INPUT is often 0
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//This works with timer running at 8 MHz
void mybitDelay(int c)
{
	uint32_t duration = c * 8;
    TIM3->CNT = 0;
    while ((TIM3->CNT) < duration);
}


```
