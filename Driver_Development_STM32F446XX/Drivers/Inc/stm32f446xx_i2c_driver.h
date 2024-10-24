/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Sep 27, 2024
 *      Author: hp
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"
/*
 * Configuration structure for I2Cx Peripheral
 */
typedef struct
{
  uint32_t  I2C_SCLSpeed;
  uint8_t   I2C_DeviceAddress;
  uint8_t   I2C_ACKControl;
  uint16_t  I2C_FMDutyCycle;
}I2C_Config_t;


/*
 * Handle Structure for I2Cx Peripheral
*/
typedef struct
{
  I2C_RegDef_t *pI2Cx;
  I2C_Config_t  I2C_Config;
  uint8_t       *pTxBuffer;
  uint8_t       *pRxBuffer;
  uint32_t       TxLen;
  uint32_t       RxLen;
  uint8_t        TxRxState;
  uint8_t        DevAddr;
  uint32_t       RxSize;
  uint8_t        Sr;
}I2C_Handle_t;



/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2


/*
 * @I2C_SCLSpeed
*/
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM4K      400000
#define I2C_SCL_SPEED_FM2K      200000

/*
 * @DEVICE_ADDRESS
*/
#define I2C_ACK_ENABLE           1
#define I2C_ACK_DISABLE          0

/*
 * @DutyCycle
 */
#define I2C_FM_DUTY_2            0
#define I2C_FM_DUTY_16_9         1


/*
 * I2C Application event macro
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9



/*********************************************************
 *      API Supported by this driver
 *      For more information about the api's check the respective function definitions
**********************************************************/

/*
 * Peripheral Clock Setup
*/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx ,uint8_t State);        /*!<State --> SET/Enable as 1 and RESET/DISABLE as 0  >*/




/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);



/*
 * Data Send and Receive
*/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR Handling
*/


void I2C_IRQConfig(uint8_t IRQNumber, uint8_t State );
void I2C_IRQHandling(I2C_Handle_t* pHandle);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle);                     //Event IRQ Handling
void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle);                     //error IRQ Handling

/*
 * Other Peripheral control API'S
*/
void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t State);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t State);
void I2C_SlaveEnableDisableCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t State);
/*
 * Application Callback
*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t App_Ev);



/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE           (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE          (1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB            (1 << I2C_SR1_SB)
#define I2C_FLAG_OVR           (1 << I2C_SR1_OVR)
#define I2C_FLAG_AF            (1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO          (1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR          (1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF         (1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10         (1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF           (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR          (1 << I2C_SR1_ADDR)
#define I2C_FLAG_IMEOUT        (1 << I2C_SR1_TIMEOUT)





#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */