# DS18B20

`DS18B20` is an C library for controlling DS18B20 temperature sensors.
It can be easily compiled for every micro-controller supported by `gcc`.

## Usage example

```c
#include "main.h"
#include "ds18b20.h"
//#include "stm32g0xx_hal.h"

int main( )
{
	int temp;
	ds18_lib_t myLib;
    myLib.user_pinMode = mypinMode; //arduino-like pinMode implementation.
    myLib.user_writePin = HAL_GPIO_WritePin;
    myLib.user_readPin = HAL_GPIO_ReadPin;
    myLib.user_portDelay = mybitDelay;	//your delay_us() function
    
    ds18_lib_init(&myLib);
    
    uint8_t myrom[10];
    uint8_t mysp[10];
    
    ds18_dev_t myds18;
    myds18.GPIOx = GPIOA;
    myds18.pin = GPIO_PIN_4;
    myds18.rom = myrom;
    myds18.sp = mysp;
    
    
	int32_t temp;
	
	while ( 1 )
	{
		//Start conversion (without ROM matching)
		ds18_convert(&myds18);

		//Delay (sensor needs time to perform conversion)
		HAL_Delay( 1000 );

		//Read temperature (without ROM matching)
		ds18_read(&myds18, &temp);

		//Somehow use data stored in `temp` variable
		printf("temp: %d\n", temp);
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
    uint32_t start = TIM3->CNT;
    uint32_t duration = c * 8; //8 is your freq in MHz
    while ((TIM3->CNT - start) < duration);
}


```

For more information visit [wiki](https://github.com/Jacajack/avr-dallas1820/wiki).
