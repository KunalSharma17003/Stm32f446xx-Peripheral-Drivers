/*
 * 004spi_SendData.c
 *
 *  Created on: Sep 25, 2024
 *      Author: Kunal Sharma
 */
//pb12-->spi2_nss
//pb13-->spi2_sclk
//pb14-->spi2_miso
//pb15-->spi2_mosi
//with alternate functionality mode as 5
#include"stm32f446xx.h"
#include"string.h"
#include "stm32f446xx_gpio_driver.h"
#include"stm32f446xx_spi_driver.h"
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
void SPI2_GPIOInits(void)
{
  GPIO_Handle_t SPIPins;
  SPIPins.pGPIOx = GPIOB;
  SPIPins.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFUN;
  SPIPins.GPIO_PinConfig_t.GPIO_PinAltFunMode = 5;
  SPIPins.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  SPIPins.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
  SPIPins.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_SPEED_FAST;


  //SCLK
  SPIPins.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_13;
  GPIO_Init(&SPIPins);

  //MOSI
  SPIPins.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_15;
  GPIO_Init(&SPIPins);

  //MISO
//  SPIPins.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_14;
//  GPIO_Init(&SPIPins);

  //NSS
//  SPIPins.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_12;
//  GPIO_Init(&SPIPins);
}
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_CPOL= SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI_Init(&SPI2Handle);

}
int main(void)
{
  char usr_data[] = "hello world";
  btn_gpio_inits();
  SPI2_GPIOInits();
  SPI2_Inits();
  SSI_Config(SPI2,ENABLE);
  /*******************************************
   * Making SSOE 1 does NSS Output enable
   * the nss pin is automatically managed by the hardware
   * i.e. when when spe =1 , nss will be pulled to low
   * and nss pin will be high when spe =0*/
  SPI_SSOE_Config(SPI2, ENABLE);
  while(1)
  {
  if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_13)==RESET)
  {


  SPI_Peripheral_Control(SPI2,ENABLE);

  //ENABLE THE SPI PERIPHERALS

  SPI_SendData(SPI2,(uint8_t*)usr_data, strlen(usr_data));

  //check whether spi is busy or not
  while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

  SPI_Peripheral_Control(SPI2,DISABLE);
  }
  }



  return 0;
}
