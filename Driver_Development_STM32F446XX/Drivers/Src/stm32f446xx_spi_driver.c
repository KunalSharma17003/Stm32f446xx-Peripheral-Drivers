/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Sep 25, 2024
 *      Author: hp
 */

#include "stm32f446xx_spi_driver.h"
#include"stm32f446xx.h"
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle); //make these functions private
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);
/*
 * Peripheral Clock Setup
*/
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

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx ,uint8_t State)
{
	if(State == ENABLE)      /*!<State --> SET/Enable as 1 and RESET/DISABLE as 0  >*/
	{
		if(pSPIx == SPI1)
		{
           SPI1_PCLK_EN();
		}
		else if(pSPIx ==SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx ==SPI3)
		{
					SPI3_PCLK_EN();
		}
		else if(pSPIx ==SPI4)
		{
					SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
		   SPI1_PCLK_DI();
		}
		else if(pSPIx ==SPI2)
		{
		   SPI2_PCLK_DI();
		}
		else if(pSPIx ==SPI3)
		{
		    SPI3_PCLK_DI();
		}
		else if(pSPIx ==SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}




/*
 * Init and De-Init
 */
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Enabling the clock so user need not to do it explicitly
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	uint32_t tempreg = 0;
  //1.CONFIGURE THE DEVICE MODE
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode<<2;

  //2. Configure the Bus Config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Bi-Di Mode Should be cleared
		tempreg &= ~(1<<15);

	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig ==SPI_BUS_CONFIG_HD)
	{
		//Bi-Di Mode should be enabled
		tempreg |= (1<<15);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig ==SPI_BUS_CONFIG_SIMPLEX_TXONLY)
	{

	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_TXONLY)
		{
			//Bi-Di Mode must be cleared
			tempreg &= ~(1<<15);
			//Bi-Di_OE(Output enable) Bit must be set
			tempreg |= (1<<14);
		}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//Bi-Di Mode must be cleared
		tempreg &= ~(1<<15);
		//RxOnly Bit must be set
		tempreg |= (1<<10);
	}

//	3.configure the spi serial clock speed(baud rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SClkSpeed<<3;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF<<11;

	//4.Configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL<<1;

	//5.Configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA<<0;


	//Saving the value of tempreg into the cr1 register
	pSPIHandle->pSPIx->CR1 = tempreg;
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
  SPI_PeriClockControl(pSPIx, DISABLE);
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data Send and Receive
*/
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
 * @Note              - this is a blocking call as it will hold until all the bytes have been transmitted

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	//1. Check if length is greater than zero or not
	while(Len>0)
	{
		//check for txe flag
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );

		//check the dff bit in cr1
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16 bit dff
			//load data to dr
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
		   //8 bit dff
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len )
{
	while(Len>0)
		{
			//check for txe flag
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET)
		{

		}

			//check the dff bit in cr1
			if((pSPIx->CR1 & (1<< SPI_CR1_DFF)))
			{
				//16 bit dff
				//load data FROM dr to RxBuffer address
				*((uint16_t *)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else
			{
			   //8 bit dff
				*(pRxBuffer) = pSPIx->DR;
				Len--;
				pRxBuffer++;
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
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
    //1. save the tx buffer address and len information in some global variable
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;



	//2. mark the spi state as busy in transmission so that
	//no other code can take over same spi peripheral until transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. enable the txie control bit to get interrupt whenever txe flag is set in sr
	pSPIHandle->pSPIx->CR2 |=(1<<SPI_CR2_TXEIE);



	//4. data transmission will be handled by the isr code

	}
	return state;
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
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len )
{

	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
	    //1. save the tx buffer address and len information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;



		//2. mark the spi state as busy in transmission so that
		//no other code can take over same spi peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. enable the txie control bit to get interrupt whenever txe flag is set in sr
		pSPIHandle->pSPIx->CR2 |=(1<<SPI_CR2_RXNEIE);



		//4. data transmission will be handled by the isr code

		}
		return state;
}


/*
 * IRQ Configuration and ISR Handling
*/
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

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t State )
{

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
void SPI_IRQHandling(SPI_Handle_t* pHandle)
{
   //Fisrt let's check where the interrupt is occured
	//let's check for txe
	uint8_t temp1,temp2;
	temp1 = pHandle->pSPIx->SR &(1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SR &(1<<SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle txe
		spi_txe_interrupt_handle(pHandle);
	}

	//let's check for rxe

		temp1 = pHandle->pSPIx->SR &(1<<SPI_SR_RXNE);
		temp2 = pHandle->pSPIx->SR &(1<<SPI_CR2_RXNEIE);

		if(temp1 && temp2)
		{
			//handle txe
			spi_rxne_interrupt_handle(pHandle);
		}

		//let's check for other interrupts
			temp1 = pHandle->pSPIx->SR &(1<<SPI_SR_OVR);
			temp2 = pHandle->pSPIx->SR &(1<<SPI_CR2_ERRIE);

			if(temp1 && temp2)
			{
				//handle txe
				spi_ovr_interrupt_handle(pHandle);
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/*
 * Other Peripheral control API'S
*/
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t State)
{
	if(State == ENABLE)
	{
		pSPIx->CR1 |=(1<<SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &=~(1<<SPI_CR1_SPE);
	}
}

void SSI_Config(SPI_RegDef_t *pSPIx, uint8_t State)
{
	if(State == ENABLE){
  pSPIx->SR |= (1<<SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SR &= ~(1<<SPI_CR1_SSI);
	}
}


void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t State)
{
	if(State ==ENABLE)
	{
		pSPIx->CR2 |=(1<<SPI_CR2_SS_OE);
	}
	else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SS_OE);
	}
}





//some helper function implementations

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
			{
				//16 bit dff
				//load data to dr
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}
			else
			{
			   //8 bit dff
				pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
	if(!pSPIHandle->TxLen)
	{
		//tx len is zero , so close the spi communication and inform the application that Transmission is over
		pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState =SPI_READY;
		SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_TX_CMPLT);

	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    if(pSPIHandle->pSPIx->CR1 &(1<<11))
    {
    	//16 bit
    	*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
    	pSPIHandle->RxLen-=2;
    	pSPIHandle->pRxBuffer--;
    	pSPIHandle->pRxBuffer--;

    }
    else
    {
    	//8 bit
    	*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
    	pSPIHandle->RxLen--;
    	pSPIHandle->pRxBuffer--;
    }
    if(!pSPIHandle->RxLen)
    {
    	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
    	pSPIHandle->pRxBuffer = NULL;
    	pSPIHandle->RxLen = 0;
    	pSPIHandle->RxState = SPI_READY;
    	SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_RX_CMPLT);

    }
}
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
   //Clear the ovr flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
  //inform the application
	SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState =SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}




__attribute__((weak))  void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t App_Ev)
{
  //weak implementation. the application can override this function

}
