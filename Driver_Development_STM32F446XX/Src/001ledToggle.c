/*
 * 001ledToggle.c
 *
 *  Created on: Sep 22, 2024
 *      Author: Kunal Sharma
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void Delay(void)
{
	for(uint32_t i = 0; i<500000/2 ; i++);
}




int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig_t.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);
	while(1)
	{

		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_5);
		Delay();

	}
	return 0;
}

