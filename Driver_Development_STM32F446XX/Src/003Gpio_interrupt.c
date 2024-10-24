/*
 * 002LedButton.c
 *
 *  Created on: Sep 22, 2024
 *      Author: Kunal Sharma
 *
 */
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void Delay(void)
{
	for(uint32_t i = 0; i<500000/2 ; i++);
}




int main(void)
{
	GPIO_Handle_t GpioLed , GpioBtn;
	//Setting up the GPIO Led
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig_t.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	//Setting Up the GPIO Button
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_13;
	GpioBtn.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;


	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioBtn);


	//we have to do the irq configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);
	return 0;
}
void EXTI15_10_IRQHandler(void)
	{
		GPIO_IRQHandling(GPIO_PIN_13);
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
	}
