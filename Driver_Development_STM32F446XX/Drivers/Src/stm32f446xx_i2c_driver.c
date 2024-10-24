/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Sep 27, 2024
 *      Author: hp
 */

#include "stm32f446xx_i2c_driver.h"



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx ,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx ,uint8_t SlaveAddr);



/*
 * Peripheral Clock Setup
*/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx ,uint8_t State)
{
	if(State == ENABLE){
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx== I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
				{
					I2C1_PCLK_DI();
				}
				else if(pI2Cx== I2C2)
				{
					I2C2_PCLK_DI();
				}
				else if(pI2Cx == I2C3)
				{
					I2C3_PCLK_DI();
				}
	}
}




/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

   uint32_t tempreg =0;
   //Enable the clock
   I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
   //ack control bit for mainly enabling and disabling the ACK's
   tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl <<10;
   pI2CHandle->pI2Cx->CR1=tempreg;

   //configure the freq feild of cr2
   tempreg =0;
   tempreg |= RCC_GetPclk1Value()/1000000U;
   pI2CHandle->pI2Cx->CR2 = (tempreg&0x3F);

   //program the device slave address
   tempreg =0;
   tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;
   tempreg |= (1<<14);
   pI2CHandle->pI2Cx->OAR1 = tempreg;

   //CCR CALCULATION
   uint16_t  ccr_value;
   tempreg = 0;
   if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
   {
   //speed mode is standard mode
   ccr_value = RCC_GetPclk1Value()/(2* pI2CHandle->I2C_Config.I2C_SCLSpeed);
   tempreg |= (ccr_value & 0xFFF);
   }
   else
   {
	  //speed mode is fast mode
	   tempreg |= (1<<15);
	   tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
	   if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
	   {
		   ccr_value = RCC_GetPclk1Value()/(3* pI2CHandle->I2C_Config.I2C_SCLSpeed);

	   }
	   else
	   {
		   ccr_value = RCC_GetPclk1Value()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);

	   }
	   tempreg |= (ccr_value & 0xFFF);

   }
   pI2CHandle->pI2Cx->CCR = tempreg;

   //Trise calculations
   if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
   {
	   //mode is standard

	  tempreg = (RCC_GetPclk1Value()/1000000U)+1;
   }
   else
   {
	//mode is fast mode
	   tempreg = ((RCC_GetPclk1Value()*300)/1000000000U)+1;
   }
   pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx);



/*
 * Data Send and Receive
*/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//Steps to be followed
	//1.generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm the start generation is completed by checking the SB flag in SR1
	//Note: Until The SB bit is cleared the clock will be stretched
    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));
//	while(!pI2CHandle->pI2Cx->SR1 & (0x01) );

    //3.Send the address of the slave with r/w bit set to w(0) (total 8 bits)
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx ,SlaveAddr);

    //4. Confirm the address phase is completed by checking the addr flag in the sr1
    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    //5.clear the addr flag according to it's software sequence
    //Note: Until addr is cleared scl will be stretched (pulled to low)
    I2C_ClearADDRFlag(pI2CHandle);

    //6. send data until length becomes 0
    while(Len>0)
    {
    	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))//checking whether txe is set or not
    	{
    		pI2CHandle->pI2Cx->DR = *pTxBuffer;
    		pTxBuffer++;
    		Len--;
    	}
    }

       //7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
      // Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
     //when BTF=1 SCL will be stretched (pulled to LOW)
    while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

    while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

    //8.generate the stop condition and master need not to wait for the completion of stop condition
    //note: generating the stop , automatically clears the btf
    if(Sr == DISABLE)
    {
    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }

}


void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//Steps to be followed
		//1.generate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//2. confirm the start generation is completed by checking the SB flag in SR1
		//Note: Until The SB bit is cleared the clock will be stretched
	    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	    //3.Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx ,SlaveAddr);

	    //4. Confirm the address phase is completed by checking the addr flag in the sr1
	    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	    //procedure to read only 1 byte
	    if(Len == 1)
	    {
	    	//disable acking
	    	I2C_ManageAcking(pI2CHandle->pI2Cx , I2C_ACK_DISABLE);

	    	//clear the addr flag
	    	I2C_ClearADDRFlag(pI2CHandle);
	    	//wait until RXNE becomes 1
	    	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
	    	//generate Stop Condition
	        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	    	//read data in to buffer
	    	*pRxBuffer = pI2CHandle->pI2Cx->DR;
	    }

	    if(Len>1)
	    {
	    	//clear the addr flag
	    	I2C_ClearADDRFlag(pI2CHandle);
	    	//read the data until len becomes zero
	    	for(uint32_t i = Len ; i>0;i--){
	    	//wait until rxne becomes 1
	    	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
	    	//check if last 2 bytes are remaining
	    	if(i==2)
	    	{
	    	//clear the ack bit
	        I2C_ManageAcking(pI2CHandle->pI2Cx , I2C_ACK_DISABLE);
	    	//generate Stop Condition
	        if(Sr == DISABLE)
	        {
	    	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	        }


	    	}
	    	//read the data from data register into buffer
	    	*pRxBuffer = pI2CHandle->pI2Cx->DR;
	    	//increment the buffer address
	    	pRxBuffer++;
	    	}
	    }
	    //re-enabling acking
	    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	    {
	    I2C_ManageAcking(pI2CHandle->pI2Cx , I2C_ACK_ENABLE);
	    }
}

void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	//interrupt handling for both master and salve mode of the device
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_SB);

	//1.handle for interrupt generated by sb event
	//note: sb flag is only applicable in master mode
	if(temp1 && temp2 && temp3)
	{
	    //SB Flag is set
		//this block will not be executed in slave mode because for slave SB is always zero
		//in this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}


	//2.handle for interrupt generated by ADDR event
   //note: When master mode: address is sent
	//when slave mode : address is matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_ADDR);
	if(temp1 && temp2 && temp3)
	{
	  //ADDR Flag is set
	  I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. handle for interrupt generated by BTF(Byte transferr finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_BTF);
		if(temp1 && temp2 && temp3)
		{
		  //BTF Flag is set
		  if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		  {
			  //make sure that TXE is Set
			 if(pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TXE))
			 {
				 //BTF and TXE both are set
				 if(pI2CHandle->TxLen ==0){
				 //generate the stop condition
				 if(pI2CHandle->Sr == DISABLE)
				 {
				 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				 }
				 // reset all the member elements of the handle structure
				 I2C_CloseSendData(pI2CHandle);
				 //notify the application about the transmission complete
				 I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				 }
			 }

		  }
		  else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		  {
			  ;
		  }
		}


	//4. Handle for interrupt generated by STOPF Event
		temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_STOPF);
			if(temp1 && temp2 && temp3)
			{
			  //STOPF Flag is set
			  //Clear the STOPF Flag(i.e.)1. read the sr1 2. write to cr1
				pI2CHandle->pI2Cx->CR1 |= 0x0000;

			  // notify the application stop is generated by the master
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
			}

	//5. Handle for interrupt generated by TXE Event

        temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TXE);
        if(temp1 && temp2 && temp3)
        {
           //checking the device mode(master or slave ) and do all this is iff the device is in master mode
        	if(pI2CHandle->pI2Cx->SR2 & (1<< I2C_SR2_MSL))
        	{
           //TXE Flag is set
        	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        	{
        		if(pI2CHandle->TxLen>0)
        		{
        		//load data into the DR
        		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
        		//decrement the Tx Len
        		pI2CHandle->TxLen--;
        		//Increment the buffer address
        		pI2CHandle->pTxBuffer++;
        		}
        	}
        }
        	else
        	{
        		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
        		{
        		//slave
        		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
        		}
        	}
        }

	//6. Handle for interrupt generated by RXNE Event

		temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_RXNE);
        if(temp1 && temp2 && temp3)
         {
        	if(pI2CHandle->pI2Cx->SR2 &(1<<I2C_SR2_MSL)){
           //RXNE Flag is set
        	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        	{
        		//we have to do data reception
        		if(pI2CHandle->RxSize == 1)
        		{
        			 *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        			 pI2CHandle->RxLen--;

        		}
        		else if(pI2CHandle->RxSize > 1)
        		{
        			 if(pI2CHandle->RxLen == 2)
        			 {
        				 I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
        			 }
        			 //read DR
        			 *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        			 pI2CHandle->pRxBuffer++;
        			 pI2CHandle->RxLen--;
        		}
        		else if(pI2CHandle->RxSize == 0)
				{
        			//close the I2C Data reception and notify the application

        			//generate the stop condition
        			if(pI2CHandle->Sr == DISABLE)
        			{
        				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        			}
        			// close the i2c rx
        			I2C_CloseReceiveData(pI2CHandle);
        			//notify the application
        			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
        	}
          }
        	else
        	{
        		if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
        		{
        			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
        		}
        	}
         }
}
void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle)
{

	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}

}









void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t State)
{
  if(State == I2C_ACK_ENABLE)
  {
	//Enable the Macro
	pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
  }
  else
  {
	//Disable the Macro
	  pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
  }
}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	//generates the start
	pI2Cx->CR1 |= (1<<I2C_CR1_START);

}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx ,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr<<1;  // salve address is shifted by 1 position as it includes the read/ write bit
	SlaveAddr &= ~(1);         // slave address = slave address + r/w
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx ,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr<<1;  // salve address is shifted by 1 position as it includes the read/ write bit
	SlaveAddr |= 1;           // slave address = slave address + r/w
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
   //check for the device mode
	if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the addr flag(read sr1 , read sr2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;

			}
		}
		else
		{
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}
	}
	else
	{
		//device is in slave mode
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;

	}
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
  pI2Cx->CR1 |=(1<< I2C_CR1_STOP);
}




void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
   //implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//implement the code to disable ITEVFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen =0;
	//

}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);

	//implement the code to disable ITEVFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}
/*
 * IRQ Configuration and ISR Handling
*/


void I2C_IRQConfig(uint8_t IRQNumber, uint8_t State )
{
	if(State == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 6 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
			}
		}
}
void I2C_IRQHandling(I2C_Handle_t* pHandle);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASEADDR + iprx ) |=  ( IRQPriority << shift_amount );
}


/*
 * Other Peripheral control API'S
*/
void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t State)
{
	if(State == ENABLE)
		{
			pI2Cx->CR1 |= (1<<I2C_CR1_PE);
		}
		else
		{
			pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
		}

}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
   if(pI2Cx->SR1 & FlagName)
   {
	   return FLAG_SET;
   }
   return FLAG_RESET;
}




uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Len;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
		}

		return busystate;
}
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;

}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}





void I2C_SlaveEnableDisableCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t State)
{
	if(State == ENABLE)
	{
		pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1<<I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);

	}
	else
	{
		pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1<<I2C_CR2_ITERREN);

	}
}
