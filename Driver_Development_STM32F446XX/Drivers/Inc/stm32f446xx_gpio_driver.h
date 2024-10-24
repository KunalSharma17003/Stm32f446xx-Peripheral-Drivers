/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Sep 22, 2024
 *      Author: hp
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"


/*
 * This is a configuration structure for a gpio pin
*/
typedef struct
{
	uint8_t GPIO_PinNumber;            /*!<Possible values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;              /*!<Possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;             /*!<Possible values from @GPIO_PIN_SPEEDS>*/
	uint8_t GPIO_PinPuPdControl;       /*!<Possible values from @GPIO_PIN_PUPD>*/
	uint8_t GPIO_PinOPType;            /*!<Possible values from @GPIO_PIN_OP_TYPES>*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a handle structure for a gpio pin
*/
typedef struct
{
	GPIO_RegDef_t *pGPIOx;                 /*!<this holds the base address of the gpio port to which the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConfig_t;    /*<this holds GPIO Pin Configuration Settings>*/
}GPIO_Handle_t;


/**************************************************************************
 *          GPIO Init Structure Macros to configure the gpio
 *
 **************************************************************************/
/*
 * @GPIO_PIN_NUMBERS
 * GPIO PIN NUMBERS
*/
#define GPIO_PIN_0    0
#define GPIO_PIN_1    1
#define GPIO_PIN_2    3
#define GPIO_PIN_3    3
#define GPIO_PIN_4    4
#define GPIO_PIN_5    5
#define GPIO_PIN_6    6
#define GPIO_PIN_7    7
#define GPIO_PIN_8    8
#define GPIO_PIN_9    9
#define GPIO_PIN_10   10
#define GPIO_PIN_11   11
#define GPIO_PIN_12   12
#define GPIO_PIN_13   13
#define GPIO_PIN_14   14
#define GPIO_PIN_15   15






/*
 *   @GPIO_PIN_MODES
 *   GPIO Pin Possible Input Mode
*/
#define GPIO_MODE_IN             0
#define GPIO_MODE_OUT            1
#define GPIO_MODE_ALTFUN         2
#define GPIO_MODE_ANALOG         3
#define GPIO_MODE_IT_FT          4
#define GPIO_MODE_IT_RT          5
#define GPIO_MODE_IT_RFT         6


/*
 * @GPIO_PIN_OP_TYPES
 * GPIO Pin Possible Outputs
*/

#define GPIO_OP_TYPE_PP          0
#define GPIO_OP_TYPE_OD          1

/*
 * @GPIO_PIN_SPEEDS
 * GPIO port output speed register
 */
#define GPIO_SPEED_LOW           0
#define GPIO_SPEED_MEDIUM        1
#define GPIO_SPEED_FAST          2
#define GPIO_SPEED_HIGH          3

/*
 * @GPIO_PIN_PUPD
 * GPIO port pull-up/pull-down register (GPIOx_PUPDR)
*/
#define GPIO_PIN_NO_PUPD               0
#define GPIO_PIN_PU                    1
#define GPIO_PIN_PD                    2


/************************************************************************************
 *            API'S Supported by this driver
 *            For more info. about the api's check the function definitions
 *
 * **********************************************************************************/

/*
 * Peripheral Clock Setup
*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx ,uint8_t State);        /*!<State --> SET/Enable as 1 and RESET/DISABLE as 0  >*/

/*
 * Peripheral Initialization and Deinitialization
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Peripheral read and write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);                                  /*!<uint16_t Because the port is 16 bit long>*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber ,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);

/*
 * Interrupt Handling for a peripheral
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t State);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority);





















#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
