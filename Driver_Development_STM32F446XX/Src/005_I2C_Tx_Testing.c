/*
 * 005_I2C_Tx_Testing.c
 *
 *  Created on: Sep 28, 2024
 *      Author: hp
 */

#include "stdio.h"
#include "stm32f446xx.h"
#include "string.h"


//pb6: i2c1 sclk
//pb7: i2c1_sda
I2C_Handle_t  I2C1Handle;
uint8_t some_data[] = "hello";
void Delay(void)
{
	for(uint32_t i = 0; i<500000/2 ; i++);
}
void btn_gpio_inits(void)
{
    GPIO_Handle_t GpioBtn;
    GpioBtn.pGPIOx = GPIOC;
  	GpioBtn.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_IN;
  	GpioBtn.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_13;
  	GpioBtn.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_SPEED_FAST;
  	GpioBtn.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
//	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioBtn);

}

void I2C1_GPIO_Inits(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFUN;
	I2CPins.GPIO_PinConfig_t.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig_t.GPIO_PinOPType  = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//sclk
	I2CPins.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	//SDA

	I2CPins.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);
}
void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0X61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1Handle);

}
int main(void)
{
	btn_gpio_inits();
    I2C1_GPIO_Inits();
    I2C1_Inits();
    I2C_PeriClockControl(I2C1, ENABLE);
    while(1){
    while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_13));

    Delay();
    I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data),0x61,DISABLE);

    }


}
