/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Oct 1, 2024
 *      Author: hp
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"
uint32_t RCC_GetOutputClock(void);
uint32_t RCC_GetPclk1Value(void);
uint32_t RCC_GetPclk2Value(void);


#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
