/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Apr. 18, 2020
 *      Author: Rahul
 */


#include "stm32f446xx_usart_driver.h"


/************************************************* HELPER FUNCTIONS *******************************************/

static void USART_Write_IT_Handle(USART_Handle_t *pUSARTxHandle);
static void USART_Read_IT_Handle(USART_Handle_t *pUSARTxHandle);


/*********************************************** DRIVER API IMPLMENTATION ******************************************/

/*
 * Function:	USART_PeriClockControl
 *
 * Brief: 		Enable and disable clock signal to USART/UART peripherals
 *
 * Params: 		struct USART_RegDef_t* *pUSARTx - USART/UART base address
 * 				uint8_t - Enable or disable value
 *
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}


/*
 * Function:	USART_PeripheralControl
 *
 * Brief: 		Enable or disable SPI peripheral communication
 *
 * Params: 		struct USART_RegDef_t *pUSARTx - USART/UART base address
 * 				uint8_t EnOrDi - Enable or disable communication (0 or 1)
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}
	else
	{
		pUSARTx->CR1 |= ~(1 << 13);
	}
}


/*
 * Function:	USART_GetFlagStatus
 *
 * Brief: 		Check status register of USART to see if flag is set
 *
 * Params: 		struct USART_RegDef_t *pUSARTx - USART base address
 * 				uint8_t USART_FLAG - flag being checked
 *
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t USART_FLAG)
{
	return (pUSARTx->SR & USART_FLAG);
}


/*
 * Function:	USART_ClearFlag
 *
 * Brief: 		Clears flag from status register of USART
 *
 * Params: 		struct USART_RegDef_t *pUSARTx - USART base address
 * 				uint8_t USART_FLAG - flag being cleared
 *
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t USART_FLAG)
{
	int dummyRead, dummyWrite = 0xFF;

	if (USART_FLAG == USART_FLAG_PE)
	{
		// Clear PE by waiting for RXNE, reading from SR, writing to DR
		while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_RXNE));
		dummyRead = pUSARTx->SR;
		pUSARTx->DR |= dummyWrite;
	}
	else if (USART_FLAG == USART_FLAG_FE)
	{
		// Clear FE by read from SR, then read from DR
		dummyRead = pUSARTx->SR;
		dummyRead = pUSARTx->DR;
	}
	else if (USART_FLAG == USART_FLAG_NF)
	{
		// Clear NF by read from SR, then read from DR
		dummyRead = pUSARTx->SR;
		dummyRead = pUSARTx->DR;
	}
	else if (USART_FLAG == USART_FLAG_ORE)
	{
		// Clear ORE by read from SR, then read from DR
		dummyRead = pUSARTx->SR;
		dummyRead = pUSARTx->DR;
	}
	else if (USART_FLAG == USART_FLAG_IDLE)
	{
		// Clear IDLE by read from SR, then read from DR
		dummyRead = pUSARTx->SR;
		dummyRead = pUSARTx->DR;
	}
	else if (USART_FLAG == USART_FLAG_RXNE)
	{
		// Clear RXNE by read from DR
		dummyRead = pUSARTx->DR;
	}
	else if (USART_FLAG == USART_FLAG_TC)
	{
		// Clear TC by read from SR, then write to DR
		dummyRead = pUSARTx->SR;
		pUSARTx->DR |= dummyWrite;
	}
	else if (USART_FLAG == USART_FLAG_TXE)
	{
		// Clear TXE flag by writing to DR
		pUSARTx->DR |= dummyWrite;
	}
	else if (USART_FLAG == USART_FLAG_LBD)
	{
		// Clear LBD by writing to 0
		pUSARTx->SR &= ~(1 << 8);
	}
	else if (USART_FLAG == USART_FLAG_CTS)
	{
		// Clear CTS by writing to 0
		pUSARTx->SR &= ~(1 << 9);
	}

	(void)dummyRead;
}


/*
 * Function:	USART_SetBaudRate
 *
 * Brief: 		Set baud rate of USART
 *
 * Params: 		struct USART_RegDef_t *pUSARTx - USART base address
 * 				uint16_t USART_BaudRate - Baud rate to be set (bps)
 *
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t USART_BaudRate)
{
	uint32_t ApbClk;

	// 1. Get the peripheral clock to USART/UART
	if (pUSARTx == USART1 || pUSARTx == USART6)
	{
		ApbClk = RCC_GetAPB2CLK();
	}
	else
	{
		ApbClk = RCC_GetAPB1CLK();
	}

	// 2. Calculate USARTDIV
	uint8_t over8 = (pUSARTx->CR1 & (1 << 15)) ? 1 : 0;
	float usartDiv = (ApbClk  * 100) / ((8 * (2 - over8)) * USART_BaudRate);   // Multiply hundred to get 2 decimal places

	// 3. Calculate mantissa and fraction (See USART 25.4.4 in datasheet for formulas)
	uint16_t mantissa = usartDiv / 100;
	uint16_t fraction = (usartDiv - mantissa * 100) * (8 * (2 - over8));

	// Round fraction
	fraction = (fraction + 50) / 100;

	// 4. Carry over
	if (over8)
	{
		// Fraction register is 3 bits
		if (fraction > 7)
		{
			mantissa++;
			fraction = 0;
		}

		pUSARTx->BRR |= (mantissa << 4);
		pUSARTx->BRR |= ((fraction & 0x07) << 0);
	}
	else
	{
		// Fraction register is 4 bits
		if (fraction > 15)
		{
			mantissa++;
			fraction = 0;
		}

		pUSARTx->BRR |= (mantissa << 4);
		pUSARTx->BRR |= ((fraction & 0x0F) << 0);
	}
}


/*
 * Function:	USART_Init
 *
 * Brief: 		Initialize the USART peripheral
 *
 * Params: 		struct USART_Handle_t *pUSARTxHandle - USART peripheral handle
 *
 */
void USART_Init(USART_Handle_t *pUSARTxHandle)
{
	// 1. Give clock signal to USART/UART bus
	USART_PeriClockControl(pUSARTxHandle->pUSARTx, ENABLE);

	// 2. Enable USART/UART peripheral
	USART_PeripheralControl(pUSARTxHandle->pUSARTx, ENABLE);

	// 3. Set the data length
	if (pUSARTxHandle->USART_Config.USART_WordLen == USART_9BIT_DATA)
	{
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 12);
	}

	// 4. Parity setup
	if (pUSARTxHandle->USART_Config.USART_ParitySet != USART_PARITY_DISABLE)
	{
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 10);

		if (pUSARTxHandle->USART_Config.USART_ParitySet == USART_EVEN_PARITY)
		{
			pUSARTxHandle->pUSARTx->CR1 &= ~(1 << 9);
		}
		else if (pUSARTxHandle->USART_Config.USART_ParitySet == USART_ODD_PARITY)
		{
			pUSARTxHandle->pUSARTx->CR1 |= (1 << 9);
		}
	}

	// 5. Set number of stop bits
	pUSARTxHandle->pUSARTx->CR2 &= ~(0x3 << 12);

	if (pUSARTxHandle->USART_Config.USART_StopBits == USART_HALF_STOP)
	{
		pUSARTxHandle->pUSARTx->CR2 |= (1 << 12);
	}
	else if (pUSARTxHandle->USART_Config.USART_StopBits == USART_2_STOP)
	{
		pUSARTxHandle->pUSARTx->CR2 |= (0x2 << 12);
	}
	else if (pUSARTxHandle->USART_Config.USART_StopBits == USART_1ANDHALF_STOP)
	{
		pUSARTxHandle->pUSARTx->CR2 |= (0x3 << 12);
	}

	// 6. Set over sampling mode
	if (pUSARTxHandle->USART_Config.USART_Oversampling == USART_OVERSMPL_8)
	{
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 15);
	}
	else
	{
		pUSARTxHandle->pUSARTx->CR1 &= ~(1 << 15);
	}

	// 7. Set baud rate
	USART_SetBaudRate(pUSARTxHandle->pUSARTx, pUSARTxHandle->USART_Config.USART_BaudRate);

	// 8. Set Tx/Rx mode
	if (pUSARTxHandle->USART_Config.USART_Mode == USART_RX_ONLY)
	{
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 2);
	}
	else if (pUSARTxHandle->USART_Config.USART_Mode == USART_TX_ONLY)
	{
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 3);
	}
	else if (pUSARTxHandle->USART_Config.USART_Mode == USART_RX_TX)
	{
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 3) | (1 << 2);
	}

	// 9. Hardware flow control setup
	pUSARTxHandle->pUSARTx->CR2 &= ~(0x3 << 8);

	if (pUSARTxHandle->USART_Config.USART_HWFlowCtrl == USART_RTS_ON)
	{
		pUSARTxHandle->pUSARTx->CR3 |= (1 << 8);
	}
	else if (pUSARTxHandle->USART_Config.USART_HWFlowCtrl == USART_CTS_ON)
	{
		pUSARTxHandle->pUSARTx->CR3 |= (1 << 9);
	}
	else if (pUSARTxHandle->USART_Config.USART_HWFlowCtrl == USART_RTS_CTS_ON)
	{
		pUSARTxHandle->pUSARTx->CR3 |= (0x3 << 8);
	}
}


/*
 * Function:	USART_DeInit
 *
 * Brief: 		Reset the USART/UART peripheral
 *
 * Params: 		struct USART_RegDef_t *pUSARTx - USART/UART register pointer
 *
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1)
	{
		USART1_RESET();
	}
	else if (pUSARTx == USART2)
	{
		USART2_RESET();
	}
	else if (pUSARTx == USART3)
	{
		USART3_RESET();
	}
	else if (pUSARTx == UART4)
	{
		UART4_RESET();
	}
	else if (pUSARTx == UART5)
	{
		UART5_RESET();
	}
	else if (pUSARTx == USART6)
	{
		USART6_RESET();
	}
}


/*
 * Function:	USART_Write
 *
 * Brief: 		Read data using USART/UART
 *
 * Params: 		struct USART_Handle_t *pUSARTxHandle - USART/UART handle
 * 				uint8_t* pTxBuffer - pointer to location of data to send
 * 				uint32_t len - number of 8-bit data packets to be sent
 *
 */
void USART_Write(USART_Handle_t *pUSARTxHandle, uint8_t *pTxBuffer, uint8_t len)
{
	while (len > 0)
	{
		// 1. Send start with TE
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 3);

		// 2. Wait for TXE flag
		while(!USART_GetFlagStatus(pUSARTxHandle->pUSARTx, USART_FLAG_TXE));

		// 3. Write data (how many bits depends on word length and parity)
		if (pUSARTxHandle->USART_Config.USART_WordLen == USART_9BIT_DATA)
		{
			// Send 9 bits of data
			pUSARTxHandle->pUSARTx->DR = (uint16_t)(*pTxBuffer) & (0x01FF);

			// If parity disabled, 9 bits of useful data sent - increment data buffer accordingly
			if (pUSARTxHandle->USART_Config.USART_ParitySet == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				len--;
			}
		}
		else
		{
			pUSARTxHandle->pUSARTx->DR = *pTxBuffer;
		}

		pTxBuffer++;
		len--;
	}

	// 4. Wait for TC flag to indicate transmission complete
	while(!USART_GetFlagStatus(pUSARTxHandle->pUSARTx, USART_FLAG_TC));

	// 5. Clear the TC bit
	USART_ClearFlag(pUSARTxHandle->pUSARTx, USART_FLAG_TC);

	// 6. End transmission by clearing TE
	pUSARTxHandle->pUSARTx->CR1 &= ~(1 << 3);
}


/*
 * Function:	USART_Read
 *
 * Brief: 		Read data using USART/UART
 *
 * Params: 		struct USART_Handle_t *pUSARTxHandle - USART/UART handle
 * 				uint8_t* pRxBuffer - pointer to location where read data is stored
 * 				uint32_t len - number of 8-bit data packets to be read
 *
 */
void USART_Read(USART_Handle_t *pUSARTxHandle, uint8_t *pRxBuffer, uint8_t len)
{
	while (len > 0)
	{
		// 1. Wait for RXNE to set
		while(!USART_GetFlagStatus(pUSARTxHandle->pUSARTx, USART_FLAG_RXNE));

		// 2. If 9-bit word length and no parity, read 9-bits of data
		if (pUSARTxHandle->USART_Config.USART_WordLen == USART_9BIT_DATA && pUSARTxHandle->USART_Config.USART_ParitySet == USART_PARITY_DISABLE)
		{
			 *((uint16_t*)(pRxBuffer)) = pUSARTxHandle->pUSARTx->DR & 0x01FF;
			 pRxBuffer++;
			 len--;
		}
		else
		{
			*pRxBuffer = pUSARTxHandle->pUSARTx->DR & 0x00FF;
		}

		// 3. Increment Rx buffer
		pRxBuffer++;
		len--;
	}
}



/************************************************ INTERRUPT CONFIG AND HANLDER APIS ****************************************/


/*
 * Function:	USART_IRQConfig
 *
 * Brief: 		Set or clear interrupt in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t Value - Enabling or disabling interrupt (1 or 0)
 *
 */
void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * Function:	USART_IRQPriorityConfig
 *
 * Brief: 		Set interrupt priority in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t IRQPriority - Priority value (0 - 255)
 *
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t shift = 8 * (IRQNumber % 4) + NVIC_NONIMPL_LOW_BITS;
	NVIC_IPR->IPR[IRQNumber / 4] = (IRQPriority << shift);
}


/*
 *
 */
void USART_EV_IRQHandling(USART_Handle_t *pUSARTxHandle)
{
	// 1. Check which interrupts have been set(RXNEIE, TCIE, TXEIE, CTSIE)
	uint8_t temp1, temp2, temp3;

	// 2. Is TC interrupt?
	temp1 = pUSARTxHandle->pUSARTx->SR & (1 << 6);
	temp2 = pUSARTxHandle->pUSARTx->CR1 & (1 << 6);
	if (temp1 && temp2)
	{
		// If TXE also raised, TxLen = 0 then this is end of transmission
		if (pUSARTxHandle->TxState == USART_BUSY_TX && pUSARTxHandle->TxLen == 0)
		{
			USART_CloseTx_IT(pUSARTxHandle);
		}
	}

	// 3. Is TXE interrupt?
	temp1 = pUSARTxHandle->pUSARTx->SR & (1 << 7);
	temp2 = pUSARTxHandle->pUSARTx->CR1 & (1 << 7);
	if (temp1 && temp2)
	{
		if (pUSARTxHandle->TxState == USART_BUSY_TX)
		{
			USART_Write_IT_Handle(pUSARTxHandle);
		}
	}

	// 4. Is RXNE interrupt?
	temp1 = pUSARTxHandle->pUSARTx->SR & (1 << 5);
	temp2 = pUSARTxHandle->pUSARTx->CR1 & (1 << 5);
	if (temp1 && temp2)
	{
		if (pUSARTxHandle->RxState == USART_BUSY_RX)
		{
			USART_Read_IT_Handle(pUSARTxHandle);
		}
	}

	// 6. Is CTS interrupt?
	temp1 = pUSARTxHandle->pUSARTx->SR & (1 << 9);
	temp2 = pUSARTxHandle->pUSARTx->CR3 & (1 << 9);
	temp3 = pUSARTxHandle->pUSARTx->CR3 & (1 << 10);
	if (temp1 && temp2 && temp3)
	{
		// TODO
	}

	// 7. Is ORE error interrupt
	temp1 = pUSARTxHandle->pUSARTx->SR & (1 << 3);
	temp2 = pUSARTxHandle->pUSARTx->CR3 & (1 << 9);
	if (temp1 && temp2)
	{
		// If RXNE also set, a data bit has been lost
		if (pUSARTxHandle->pUSARTx->SR & (1 << 5))
		{
			// Close reception and send callback
			USART_CloseRx_IT(pUSARTxHandle);
			USART_ApplicationCallbackEvent(pUSARTxHandle, USART_ORE_ERROR);
		}
	}
}


/*
 * Function:	USART_Write_IT
 *
 * Brief: 		Send data via USART/UART using non-blocking interrupts
 *
 * Params: 		USART_Handle_t *pUSARTxHandle - USART/UART handle
 * 				uint8_t* pTxBuffer - pointer to data to be sent
 * 				uint32_t len - number of 8-bit data packets to be sent
 *
 */
uint8_t USART_Write_IT(USART_Handle_t *pUSARTxHandle, uint8_t *pTxBuffer, uint8_t len)
{
	uint8_t state = pUSARTxHandle->TxState;

	if (state != USART_BUSY_TX)
	{
		// 1. Set to busy in Tx
		pUSARTxHandle->TxState = USART_BUSY_TX;

		// 2. Store parameters in handle
		pUSARTxHandle->TxLen = len;
		pUSARTxHandle->pTxBuffer = pTxBuffer;

		// 3. CTS interrupt only if in HW flow control
		if (pUSARTxHandle->USART_Config.USART_HWFlowCtrl == USART_CTS_ON || pUSARTxHandle->USART_Config.USART_HWFlowCtrl == USART_RTS_CTS_ON)
		{
			pUSARTxHandle->pUSARTx->CR3 |= (1 << 10);
		}

		// 4. Enable interrupt TXEIE, TCIE, CTSIE
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 7);
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 6);

		// 5. CTSIE only if HW flow control
		if (pUSARTxHandle->USART_Config.USART_HWFlowCtrl == USART_CTS_ON || pUSARTxHandle->USART_Config.USART_HWFlowCtrl == USART_RTS_CTS_ON)
		{
			pUSARTxHandle->pUSARTx->CR3 |= (1 << 10);
		}

		// 6. Set TE bit to send start
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 3);
	}

	return state;
}


/*
 * Function:	USART_Read_IT
 *
 * Brief: 		Read data via USART/UART using non-blocking interrupts
 *
 * Params: 		USART_Handle_t *pUSARTxHandle - USART/UART handle
 * 				uint8_t* pRxBuffer - pointer where received data will be written
 * 				uint32_t len - number of 8-bit data packets to be read
 *
 */
uint8_t USART_Read_IT(USART_Handle_t *pUSARTxHandle, uint8_t *pRxBuffer, uint8_t len)
{
	uint8_t state = pUSARTxHandle->RxState;

	if (state != USART_BUSY_RX)
	{
		// 1. Set RE bit to listen for start
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 2);

		// 2. Set to busy in Rx
		pUSARTxHandle->RxState = USART_BUSY_RX;

		// 3. Store parameters in handle
		pUSARTxHandle->RxLen = len;
		pUSARTxHandle->pRxBuffer = pRxBuffer;

		// 4. Enable interrupts for RXNEIE
		pUSARTxHandle->pUSARTx->CR1 |= (1 << 5);
	}

	return state;
}


/*
 * Function:	USART_Write_IT_Handle
 *
 * Brief: 		Handle data writing to DR when TXE set in interrupt mode
 *
 * Params: 		USART_Handle_t *pUSARTxHandle - USART/UART handle
 *
 */
void USART_Write_IT_Handle(USART_Handle_t *pUSARTxHandle)
{
	// Is there data to be written
	if (pUSARTxHandle->TxLen > 0)
	{
		// Is 9 bit word length?
		if (pUSARTxHandle->USART_Config.USART_WordLen == USART_9BIT_DATA)
		{
			// If parity is disabled, 9 bits of data will be sent - increment buffer accordingly
			if (pUSARTxHandle->USART_Config.USART_ParitySet == USART_PARITY_DISABLE)
			{
				pUSARTxHandle->pUSARTx->DR = (uint16_t)*(pUSARTxHandle->pTxBuffer) & (0x01FF);
				(pUSARTxHandle->pTxBuffer)++;
				(pUSARTxHandle->TxLen)--;
			}
		}
		else
		{
			if (pUSARTxHandle->USART_Config.USART_ParitySet == USART_PARITY_DISABLE)
			{
				pUSARTxHandle->pUSARTx->DR = *(pUSARTxHandle->pTxBuffer);
			}
		}

		(pUSARTxHandle->pTxBuffer)++;
		(pUSARTxHandle->TxLen)--;
	}
}


/*
 * Function:	USART_Read_IT_Handle
 *
 * Brief: 		Handle data reading from DR when RXNE set in interrupt mode
 *
 * Params: 		USART_Handle_t *pUSARTxHandle - USART/UART handle
 *
 */
void USART_Read_IT_Handle(USART_Handle_t *pUSARTxHandle)
{
	if (pUSARTxHandle->RxLen > 0)
	{
		// Check if 9-bit or 8-bit data to read
		if (pUSARTxHandle->USART_Config.USART_WordLen == USART_9BIT_DATA )
		{
			 if (pUSARTxHandle->USART_Config.USART_ParitySet == USART_PARITY_DISABLE)
			 {
				 // 9 bits of data sent, so increment buffer once here and again later
				 *((uint16_t*)(pUSARTxHandle->pRxBuffer)) = pUSARTxHandle->pUSARTx->DR & 0x01FF;
				 (pUSARTxHandle->pRxBuffer)++;
				 (pUSARTxHandle->RxLen)--;
			 }
		}
		else
		{
			 if (pUSARTxHandle->USART_Config.USART_ParitySet == USART_PARITY_DISABLE)
			 {
				 *(pUSARTxHandle->pRxBuffer) = pUSARTxHandle->pUSARTx->DR & 0x00FF;
			 }
			 else
			 {
				 // If 8 bit data with parity, ignore the MSB
				 *(pUSARTxHandle->pRxBuffer) = pUSARTxHandle->pUSARTx->DR & 0x007F;
			 }
		}

		 (pUSARTxHandle->pRxBuffer)++;
		 (pUSARTxHandle->RxLen)--;
	}
	else
	{
		// If length 0, close Rx and fire callback
		USART_CloseRx_IT(pUSARTxHandle);
	}
}


void USART_CloseTx_IT(USART_Handle_t *pUSARTxHandle)
{
	// 1. Reset TXEIE, TCEIE, CTSIE interrupts
	pUSARTxHandle->pUSARTx->CR1 &= ~(0x3 << 6);
	pUSARTxHandle->pUSARTx->CR3 &= ~(1 << 10);

	// 2. Reset handle fields
	pUSARTxHandle->TxLen = 0;
	pUSARTxHandle->pTxBuffer = NULL;
	pUSARTxHandle->TxState = USART_RDY;

	// 3. Send callback
	USART_ApplicationCallbackEvent(pUSARTxHandle, USART_TX_CMPLT);
}


void USART_CloseRx_IT(USART_Handle_t *pUSARTxHandle)
{
	// 1. Reset interrupts
	pUSARTxHandle->pUSARTx->CR1 &= ~(1 << 5);

	// 2. Reset handle fields
	pUSARTxHandle->RxLen = 0;
	pUSARTxHandle->pRxBuffer = NULL;
	pUSARTxHandle->RxState = USART_RDY;

	// 4. Send callback
	USART_ApplicationCallbackEvent(pUSARTxHandle, USART_RX_CMPLT);
}

