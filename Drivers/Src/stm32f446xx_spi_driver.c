/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Mar 28, 2020
 *      Author: Rahul
 */

#include "stm32f446xx_spi_driver.h"


/******************************************* HELPER FUNCTIONS **************************************************/

static void SPI_TxeITHandle();
static void SPI_RxeITHandle();
static void SPI_OvrITHandle();
static void SPI_ClrTxIT();
static void SPI_ClrRxIT();
static void SPI_ApplicationCallbackEvent(SPI_Handle_t *pSPIxHandle, uint8_t event);


/*********************************************** DRIVER API IMPLMENTATION ******************************************/

/*
 * Function:	SPI_PeriClockControl
 *
 * Brief: 		Enable and disable clock signal to SPI peripherals
 *
 * Params: 		struct SPI_RegDef_t* - SPI base address
 * 				uint8_t - Enable or disable value
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}


/*
 * Function:	SPI_PeripheralControl
 *
 * Brief: 		Enable or disable SPI peripheral communication
 *
 * Params: 		struct SPI_RegDef_t* - SPI base address
 * 				uint8_t EnOrDi - Enable or disable SPI communication (0 or 1)
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << 6);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << 6);
	}
}


/*
 * Function:	SPI_GetFlagStatus
 *
 * Brief: 		Check status flag of SPI to see if flag is set
 *
 * Params: 		struct SPI_RegDef_t* - SPI base address
 * 				uint8_t - Enable or disable value
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t SPI_FLAG)
{
	if ((pSPIx->SR & SPI_FLAG) > 0)
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}


/*
 * Function:	SPI_SSI_Control
 *
 * Brief: 		Set the SSI bit for software controlled NSS
 *
 * Params: 		struct SPI_RegDef_t* - SPI base address
 * 				uint8_t EnOrDi - SSI high or low (0 or 1)
 *
 */
void SPI_SSI_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == SET)
	{
		pSPIx->CR1 |= (1 << 8);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << 8);
	}
}


/*
 * Function:	SPI_SSOE_Control
 *
 * Brief: 		Set the SSOE bit for CS control
 *
 * Params: 		struct SPI_RegDef_t* - SPI base address
 * 				uint8_t EnOrDi - SSI high or low (0 or 1)
 *
 */
void SPI_SSOE_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == SET)
	{
		pSPIx->CR2 |= (1 << 2);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << 2);
	}
}


/*
 * Function:	SPI_Init
 *
 * Brief: 		Initialize the SPI peripheral
 *
 * Params: 		struct SPI_Handle_t* - SPI peripheral handle
 *
 */
void SPI_Init(SPI_Handle_t *pSPIxHandle)
{
	// Enable clock to SPI peripheral
	SPI_PeriClockControl(pSPIxHandle->pSPIx, ENABLE);

	// 1. Set clock phase and polarity
	pSPIxHandle->pSPIx->CR1 |= ((pSPIxHandle->SPI_Config.SPI_CPHA) << 0);
	pSPIxHandle->pSPIx->CR1 |= ((pSPIxHandle->SPI_Config.SPI_CPOL) << 1);

	// 2. Master selection
	pSPIxHandle->pSPIx->CR1 |= ((pSPIxHandle->SPI_Config.SPI_DeviceMode) << 2);

	// 3. Baud rate
	pSPIxHandle->pSPIx->CR1 |= ((pSPIxHandle->SPI_Config.SPI_SclkSpeed) << 3);

	// 4. CS software management
	pSPIxHandle->pSPIx->CR1 |= ((pSPIxHandle->SPI_Config.SPI_SSM) << 9);

	// 5. Data frame format
	pSPIxHandle->pSPIx->CR1 |= ((pSPIxHandle->SPI_Config.SPI_DFF) << 11);

	// 6. Set communication mode
	if ((pSPIxHandle->SPI_Config.SPI_BusConfig == SPI_FULL_DPLX) || (pSPIxHandle->SPI_Config.SPI_BusConfig == SPI_SMPLX_TX))
	{
		pSPIxHandle->pSPIx->CR1 &= ~(1 << 15);
	}
	else if (pSPIxHandle->SPI_Config.SPI_BusConfig == SPI_HALF_DPLX)
	{
		pSPIxHandle->pSPIx->CR1 |= (1 << 15);
	}
	else if (pSPIxHandle->SPI_Config.SPI_BusConfig == SPI_SMPLX_RX)
	{
		pSPIxHandle->pSPIx->CR1 &= ~(1 << 15);
		pSPIxHandle->pSPIx->CR1 |= (1 << 10);
	}
}


/*
 * Function:	SPI_DeInit
 *
 * Brief: 		Reset the SPI peripheral
 *
 * Params: 		struct SPI_RegDef_t* - SPI register pointer
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_RESET();
	}
}


/*
 * Function:	SPI_Write
 *
 * Brief: 		Write to SPI data register
 *
 * Params: 		struct SPI_RegDef_t* - SPI register pointer
 * 				uint8_t* pTxBuffer - pointer to Tx buffer data
 *				uint32_t len - length of data to be sent
 *
 */
void SPI_Write(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// 1. Wait till the Tx buffer is empty
		while ( !(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE)) );

		// 2. Check DFF
		if (pSPIx->CR1 & (1 << 11))
		{
			// 16 bit format
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)(pTxBuffer)++;
		}
		else
		{
			//8 bit format
			pSPIx->DR = *((uint8_t*)pTxBuffer);
			len--;
			(uint8_t*)(pTxBuffer)++;
		}
	}
}


/*
 * Function:	SPI_Read
 *
 * Brief: 		Read from SPI data register
 *
 * Params: 		struct SPI_RegDef_t* - SPI register pointer
 * 				uint8_t* pTxBuffer - pointer to Rx storage address
 *				uint32_t len - length of data to be read
 *
 */
void SPI_Read(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	// 1. Wait till RXNE flag set to indicate Rx buffer holds data
	while ( !(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE)) );

	while (len > 0)
	{
		if (pSPIx->CR1 & (1 << 11))
		{
			// 16 bit format
			*(uint16_t*)(pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			(uint16_t*)(pRxBuffer)++;
		}
		else
		{
			// 8 bit format
			*(uint8_t*)(pRxBuffer) = pSPIx->DR;
			len--;
			(uint8_t*)(pRxBuffer)++;
		}
	}
}


/*
 * Function:	SPI_Write_IT
 *
 * Brief: 		Enable data write to SPI to be handled by interrupt function
 *
 * Params: 		struct pSPIxHandle* - SPI handler
 * 				uint8_t* pTxBuffer - pointer to Tx buffer data
 *				uint32_t len - length of data to be sent
 *
 */
uint8_t SPI_Write_IT(SPI_Handle_t *pSPIxHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIxHandle->TxState;

	if (state != SPI_BUSY_TX)
	{
		// Save data and length information in handle
		pSPIxHandle->pTxBuffer = pTxBuffer;
		pSPIxHandle->TxLen = len;

		// TXE status busy
		pSPIxHandle->TxState = SPI_BUSY_TX;

		// Enable interrupt for TXEIE
		pSPIxHandle->pSPIx->CR2 |= (1 << 7);
	}

	return state;
}


/*
 * Function:	SPI_Read_IT
 *
 * Brief: 		Enable data reading from SPI to be handled by interrupt function
 *
 * Params: 		struct pSPIxHandle* - SPI handler
 * 				uint8_t* pTxBuffer - pointer to Rx storage address
 *				uint32_t len - length of data to be read
 *
 */
uint8_t SPI_Read_IT(SPI_Handle_t *pSPIxHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIxHandle->RxState;

	if (state != SPI_BUSY_TX)
	{
		// 1. Save data and length information in handle
		pSPIxHandle->pRxBuffer = pRxBuffer;
		pSPIxHandle->RxLen = len;

		// 2. TXE status busy
		pSPIxHandle->RxState = SPI_BUSY_RX;

		// 3. Enable interrupt for TXEIE
		pSPIxHandle->pSPIx->CR2 |= (1 << 6);
	}

	return state;

}


/*
 * Function:	SPI_IRQConfig
 *
 * Brief: 		Set or clear interrupt in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t Value - Enabling or disabling interrupt (1 or 0)
 *
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		NVIC_ISER->IER[IRQNumber / 32] = (1 << (IRQNumber % 32));
	}
	else if (EnOrDi == DISABLE)
	{
		NVIC_ICER->IER[IRQNumber / 32] = (1 << (IRQNumber % 32));
	}
}


/*
 * Function:	SPI_IRQPriorityConfig
 *
 * Brief: 		Set interrupt priority in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t IRQPriority - Priority value (0 - 255)
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t shift = 8 * (IRQNumber % 4) + NVIC_NONIMPL_LOW_BITS;
	NVIC_IPR->IPR[IRQNumber / 4] = (IRQPriority << shift);
}


/*
 * Function:	SPI_IRQHandling
 *
 * Brief: 		Determine source of interrupt and call the appropriate handler
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t IRQPriority - Priority value (0 - 255)
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIxHandle)
{
	uint8_t temp1, temp2;

	temp1 = pSPIxHandle->pSPIx->SR & (1 << 1);
	temp2 = pSPIxHandle->pSPIx->CR2 & (1 << 7);

	if (temp1 && temp2)
	{
		// Handle Tx interrupt
		SPI_TxeITHandle(pSPIxHandle);
	}

	temp1 = pSPIxHandle->pSPIx->SR & (1 << 0);
	temp2 = pSPIxHandle->pSPIx->CR2 & (1 << 6);

	if (temp1 && temp2)
	{
		// Handle Rx interrupt
		SPI_RxeITHandle(pSPIxHandle);
	}

	temp1 = pSPIxHandle->pSPIx->SR & (1 << 6);
	temp2 = pSPIxHandle->pSPIx->CR2 & (1 << 5);

	if (temp1 && temp2)
	{
		// Handle overrun interrupt
		SPI_OvrITHandle(pSPIxHandle);
	}
}


/*
 * Function:	SPI_ClrTxIT
 *
 * Brief: 		Helper function to clear the Tx interrupt
 *
 * Params: 		SPI_Handle_t* - SPI peripheral handler
 *
 */
static void SPI_ClrTxIT(SPI_Handle_t *pSPIxHandle)
{
	// Clear TxE flag and reset handle
	pSPIxHandle->TxLen = 0;
	pSPIxHandle->pTxBuffer = NULL;
	pSPIxHandle->pSPIx->CR2 &= ~(1 << 7);
	pSPIxHandle->TxState = SPI_RDY;

	SPI_ApplicationCallbackEvent(pSPIxHandle, SPI_EV_TX_CMPLT);
}


/*
 * Function:	SPI_ClrRxIT
 *
 * Brief: 		Helper function to clear the Rx interrupt
 *
 * Params: 		SPI_Handle_t* - SPI peripheral handler
 *
 */
static void SPI_ClrRxIT(SPI_Handle_t *pSPIxHandle)
{
	// Clear RxNE interrupt and reset handle
	pSPIxHandle->RxLen = 0;
	pSPIxHandle->pRxBuffer = NULL;
	pSPIxHandle->pSPIx->CR2 &= ~(1 << 6);
	pSPIxHandle->RxState = SPI_RDY;

	SPI_ApplicationCallbackEvent(pSPIxHandle, SPI_EV_RX_CMPLT);
}


/*
 * Function:	SPI_OvrITHandle
 *
 * Brief: 		Handle overrun interrupt by clearing it
 *
 * Params: 		SPI_Handle_t* - SPI peripheral handler
 *
 */
static void SPI_OvrITHandle(SPI_Handle_t *pSPIxHandle)
{
	// Clear the OVR flag
	uint8_t temp;
	temp = pSPIxHandle->pSPIx->DR;
	temp = pSPIxHandle->pSPIx->SR;
	(void)temp;

	SPI_ApplicationCallbackEvent(pSPIxHandle, SPI_ERROR_OVR);
}


/*
 * Function:	SPI_TxeITHandle
 *
 * Brief: 		Handle Tx interrupt trigger by sending data and clearing interrupt
 *
 * Params: 		SPI_Handle_t* - SPI peripheral handler
 *
 */
static void SPI_TxeITHandle(SPI_Handle_t *pSPIxHandle)
{
	// 1. Check DFF and send data
	if (pSPIxHandle->pSPIx->CR1 & (1 << 11))
	{
		// 16 bit format
		pSPIxHandle->pSPIx->DR = *((uint16_t*)pSPIxHandle->pTxBuffer);
		(pSPIxHandle->TxLen)--;
		(pSPIxHandle->TxLen)--;
		(uint16_t*)(pSPIxHandle->pTxBuffer)++;
	}
	else
	{
		//8 bit format
		pSPIxHandle->pSPIx->DR = *((uint8_t*)pSPIxHandle->pTxBuffer);
		(pSPIxHandle->TxLen)--;
		(uint8_t*)(pSPIxHandle->pTxBuffer)++;
	}

	// 2. Clear transmission interrupt
	if (! pSPIxHandle->TxLen)
	{
		SPI_ClrTxIT(pSPIxHandle);
	}
}


/*
 * Function:	SPI_RxeITHandle
 *
 * Brief: 		Handle Rx interrupt trigger by sending data and clearing interrupt
 *
 * Params: 		SPI_Handle_t* - SPI peripheral handler
 *
 */
static void SPI_RxeITHandle(SPI_Handle_t *pSPIxHandle)
{
	// 1. Check DFF and read data
	if (pSPIxHandle->pSPIx->CR1 & (1 << 11))
	{
		// 16 bit format
		*(uint16_t*)(pSPIxHandle->pRxBuffer) = pSPIxHandle->pSPIx->DR;
		(pSPIxHandle->RxLen)--;
		(pSPIxHandle->RxLen)--;
		(uint16_t*)(pSPIxHandle->pRxBuffer)++;
	}
	else
	{
		// 8 bit format
		*(uint8_t*)(pSPIxHandle->pRxBuffer) = pSPIxHandle->pSPIx->DR;
		(pSPIxHandle->RxLen)--;
		(uint8_t*)(pSPIxHandle->pRxBuffer)++;
	}

	// 2. Clear receive interrupt
	if (! pSPIxHandle->RxLen)
	{
		SPI_ClrRxIT(pSPIxHandle);
	}
}

static void SPI_ApplicationCallbackEvent(SPI_Handle_t *pSPIxHandle, uint8_t event)
{
	//TODO
}
