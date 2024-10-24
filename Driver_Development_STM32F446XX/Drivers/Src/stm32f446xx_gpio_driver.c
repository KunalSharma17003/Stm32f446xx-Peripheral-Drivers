/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Sep 22, 2024
 *      Author: Kunal Sharma
 */


#include "stm32f446xx_gpio_driver.h"





/****************************************************************************************
 *                        Function Definitions
 *                                &
 *                            Description
*****************************************************************************************/


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx ,uint8_t State)
{
	if(State == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
				GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
		    	GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
		    	GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
				GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
				GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
				GPIOF_PCLK_EN();

		}
		else if(pGPIOx == GPIOG)
		{
				GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
				GPIOH_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA)
				{
						GPIOA_PCLK_DI();
				}
				else if(pGPIOx == GPIOB)
				{
				    	GPIOB_PCLK_DI();
				}
				else if(pGPIOx == GPIOC)
				{
				    	GPIOC_PCLK_DI();
				}
				else if(pGPIOx == GPIOD)
				{
						GPIOD_PCLK_DI();
				}
				else if(pGPIOx == GPIOE)
				{
						GPIOE_PCLK_DI();
				}
				else if(pGPIOx == GPIOF)
				{
						GPIOF_PCLK_DI();

				}
				else if(pGPIOx == GPIOG)
				{
						GPIOG_PCLK_DI();
				}
				else if(pGPIOx == GPIOH)
				{
						GPIOH_PCLK_DI();
				}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function is used to initialize the gpio peripheral
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp =0;
	//1. Configure the mode of GPIO pin
	//ENABLING THE CLOCK SO AS THE USER USES THIS FUNCTION HE NEED NOT TO ENABLE THE CLOCK EXPLICILTLY
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	if(pGPIOHandle->GPIO_PinConfig_t.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_PinConfig_t.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);                    //clearing the bit
		pGPIOHandle->pGPIOx->MODER |=temp;                                                                    // setting the bit


	}
	else
	{
		//this part is for the case of interrupt HANDLING
		if(pGPIOHandle->GPIO_PinConfig_t.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber); //Clearing the corresponding rtsr bit
		}
		else if(pGPIOHandle->GPIO_PinConfig_t.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber); //Clearing the corresponding rtsr bit

		}
		else if(pGPIOHandle->GPIO_PinConfig_t.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber); //Clearing the corresponding rtsr bit

		}

		//2.configure the gpio port selection in syscfg_exticr
		 uint8_t temp1 = pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber/4;
		 uint8_t temp2 = pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber%4;
		 uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		 SYSCFG_PCLK_EN();
		 SYSCFG->EXTICR[temp1] = PortCode<<(4*temp2);

		//3.enable the exti interrupt delivery using IMR
         EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
	}
	temp=0;


	//2.configure the speed
	temp = pGPIOHandle->GPIO_PinConfig_t.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	temp = 0;


//	3.configure the pull u  and pull down settings
	temp = pGPIOHandle->GPIO_PinConfig_t.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp =0;

//	4.configure the output type
	temp = pGPIOHandle->GPIO_PinConfig_t.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;

//	5. configure the gpio in alternate functionality mode
	if(pGPIOHandle->GPIO_PinConfig_t.GPIO_PinMode == GPIO_MODE_ALTFUN){
		//configure the alternate function registers.
		uint8_t temp1 , temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig_t.GPIO_PinNumber%8;
		if(temp1 ==1)
		{
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF<<(4*temp2));
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig_t.GPIO_PinAltFunMode<<(4*temp2));
		}
		else
		{
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF<<(4*temp2));
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig_t.GPIO_PinAltFunMode<<(4*temp2));
		}

	}


}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
			if(pGPIOx == GPIOA)
			{
					GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
			    	GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
			{
			    	GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD)
			{
					GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE)
			{
					GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOF)
			{
					GPIOF_REG_RESET();

			}
			else if(pGPIOx == GPIOG)
			{
					GPIOG_REG_RESET();
			}
			else if(pGPIOx == GPIOH)
			{
					GPIOH_REG_RESET();
			}
}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - 0 or 1
 *
 * @Note              -

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR  >> PinNumber)& 0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber ,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value)
{
     pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<< PinNumber);
}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t State )
{
     //To write this you can use the cortex-m4 general user guide
	if(State == ENABLE)
	{
		if(IRQNumber <=31)
		{
			//program iser0 register
			*NVIC_ISER0 |=(1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber<64)
		{
		   //program iser0 register
			*NVIC_ISER1 |=(1<<IRQNumber%32);
		}
		else if(IRQNumber >=64 && IRQNumber<96)
		{
			//program the iser2 register
			*NVIC_ISER2 |=(1<<IRQNumber%64);
		}
	}
	else
	{
		if(IRQNumber <=31)
		{
		     //program icer0 register
			*NVIC_ICER0 |=(1<<IRQNumber);
		}
		if(IRQNumber >31 && IRQNumber<64)
		{
			//program icer1 register
			*NVIC_ICER1 |=(1<<IRQNumber%32);
		}
		else if(IRQNumber >=64 && IRQNumber<96)
		{
			//program icer2 register
			*NVIC_ICER2 |=(1<<IRQNumber%64);
		}
	}
}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber ,uint32_t IRQPriority)
{
  //1. Find out the IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_ammount = (8* iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + (iprx)) |=(IRQPriority<<(shift_ammount));

}
/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
  //Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR &(1<<PinNumber))
	{
		//Clear the pr register
		EXTI->PR |=(1<<PinNumber);
	}

}
