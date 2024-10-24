/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Sep 25, 2024
 *      Author: Kunal Sharma
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include"stm32f446xx.h"
/*
 * Configuration Structure for SPIx peripheral
*/
typedef struct
{
   uint8_t SPI_DeviceMode;
   uint8_t SPI_BusConfig;
   uint8_t SPI_SClkSpeed;
   uint8_t SPI_DFF;
   uint8_t SPI_CPHA;
   uint8_t SPI_CPOL;
   uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * Handle Structure for SPIx Peripherals
*/
typedef struct
{
   SPI_RegDef_t *pSPIx;         /*!<this holds the base address of the spi peripherals SPIx(x:1,2,3,4)>*/
   SPI_Config_t  SPI_Config;
   uint8_t      *pTxBuffer;
   uint8_t      *pRxBuffer;
   uint32_t      TxLen;
   uint32_t      RxLen;
   uint8_t       TxState;
   uint8_t       RxState;
}SPI_Handle_t;


/**********************************************
 * Defining Macros for the SPI
**********************************************/
/*
 * @SPI_DeviceMode
*/
#define SPI_DEVICE_MODE_MASTER         1
#define SPI_DEVICE_MODE_SLAVE          0

/*
 * @SPI_BUSConfig
*/

#define SPI_BUS_CONFIG_FD              1
#define SPI_BUS_CONFIG_HD              2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY  3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY  4

/*
 * @SPI_SclkSpeed
*/
#define SPI_SCLK_SPEED_DIV2            0
#define SPI_SCLK_SPEED_DIV4            1
#define SPI_SCLK_SPEED_DIV8            2
#define SPI_SCLK_SPEED_DIV16           3
#define SPI_SCLK_SPEED_DIV32           4
#define SPI_SCLK_SPEED_DIV64           5
#define SPI_SCLK_SPEED_DIV128          6
#define SPI_SCLK_SPEED_DIV256          7


/*
 * @SPI_DFF
*/
#define SPI_DFF_8BITS                  0
#define SPI_DFF_16BITS                 1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW                   0
#define SPI_CPOL_HIGH                  1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW                   0
#define SPI_CPHA_HIGH                  1

/*
 * @SPI_SSM
*/
#define SPI_SSM_EN                     1
#define SPI_SSM_DI                     0

/**
 * SPI application states*/
#define SPI_READY                      0
#define SPI_BUSY_IN_RX                 1
#define SPI_BUSY_IN_TX                 2


/*
 * SPI APPLICATION EVENTS
*/
#define SPI_EVENT_TX_CMPLT             1
#define SPI_EVENT_RX_CMPLT             2
#define SPI_EVENT_OVR_ERR              3
#define SPI_EVENT_CRC_ERR              4

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG           ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG          ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG          ( 1 << SPI_SR_BSY)
/*********************************************************
 *      API Supported by this driver
 *      For more information about the api's check the respective function definitions
**********************************************************/

/*
 * Peripheral Clock Setup
*/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx ,uint8_t State);        /*!<State --> SET/Enable as 1 and RESET/DISABLE as 0  >*/




/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);



/*
 * Data Send and Receive
*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len );

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len );

/*
 * IRQ Configuration and ISR Handling
*/


void SPI_IRQConfig(uint8_t IRQNumber, uint8_t State );
void SPI_IRQHandling(SPI_Handle_t* pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Other Peripheral control API'S
*/

void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t State);
void SSI_Config(SPI_RegDef_t *pSPIx, uint8_t State);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t State);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
*/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t App_Ev);


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
