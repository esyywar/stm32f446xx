/*
 * stm32f446xx_dma_driver.c
 *
 *  Created on: Jun. 11, 2020
 *      Author: Rahul
 */


#include "stm32f446xx.h"


/***************************** SHIFT VALUES FOR INTERRUPT FLAGS IN DMA SR (LISR AND HISR) ***************************/

uint8_t FEIF_Flag[4] = {0, 6, 16, 22};
uint8_t DMEIF_Flag[4] = {2, 8, 18, 24};
uint8_t TEIF_Flag[4] = {3, 9, 19, 25};
uint8_t HTIF_Flag[4] = {4, 10, 20, 26};
uint8_t TCIF_Flag[4] = {5, 11, 21, 27};


/****************************** LOCAL FUNCTIONS *******************************/

static void DMA_StartStop(DMA_Handle_t *pDMAxHandle, uint8_t EnOrDi);
static void DMA_Stream_IRQ_Enable(DMA_Handle_t *pDMAxHandle);
static void DMA_Trans_Cmplt_Handle_IT(DMA_Handle_t *pDMAxHandle);


/*************************************************** DRIVER API IMPLMENTATION ***********************************************/


/*
 * Function:	DMA_PeriClockControl
 *
 * Brief: 		Enable and disable clock signal to DMA peripherals
 *
 * Params: 		struct DMA_RegDef_t *pDMAx - DMA base address
 * 				uint8_t EnOrDi - Enable or disable value
 *
 */
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pDMAx == DMA1)
		{
			DMA1_PCLK_EN();
		}
		else if (pDMAx == DMA2)
		{
			DMA2_PCLK_EN();
		}
	}
	else
	{
		if (pDMAx == DMA1)
		{
			DMA1_PCLK_DI();
		}
		else if (pDMAx == DMA2)
		{
			DMA2_PCLK_DI();
		}
	}
}


static void DMA_StartStop(DMA_Handle_t *pDMAxHandle, uint8_t EnOrDi)
{
	uint8_t streamNum = pDMAxHandle->DMA_Stream;

	if (EnOrDi == ENABLE)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 0);
	}
	else
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR &= ~(1 << 0);
	}
}


/*
 * Function:	DMA_Init
 *
 * Brief: 		Initialize the DMA peripherals
 *
 * Params: 		struct DMA_Handle_t *pDMAxHandle - DMA handle address
 *
 */
void DMA_Init(DMA_Handle_t *pDMAxHandle)
{
	// 1. Turn on peripheral clock
	DMA_PeriClockControl(pDMAxHandle->pDMAx, ENABLE);

	uint8_t streamNum = pDMAxHandle->DMA_Stream;

	// 2. Set the channel number
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_Channel << 25);

	// 3. Stream priority
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_Priority << 16);

	// 4. FIFO mode (direct mode)
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR |= (pDMAxHandle->DMA_Config.DMA_FIFOMode << 2);

	// 5. FIFO threshold
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR &= ~(0x3 << 0);
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR |= (pDMAxHandle->DMA_Config.DMA_FIFOThresh << 0);

	// 6. Set transfer direction
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_Dir << 6);

	// 7. Peripheral increment
	if (pDMAxHandle->DMA_Config.DMA_PeriphInc == DMA_PERIPH_INC)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 9);
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR &= ~(1 << 15);
	}

	// 8. Memory increment
	if (pDMAxHandle->DMA_Config.DMA_MemInc == DMA_MEM_INC)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 10);
	}

	// 9. Peripheral burst
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_PBurst << 21);

	// 10. Memory burst
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_MBurst << 23);

	// 11. Set peripheral and memory data sizes
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_PeriphDataSize << 11);
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_MemDataSize << 13);

	// 12. Set  normal, circle or peripheral control mode
	if (pDMAxHandle->DMA_Config.DMA_Mode == DMA_MODE_PFCCTRL)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 5);
	}
	else if (pDMAxHandle->DMA_Config.DMA_Mode == DMA_MODE_CIRC)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 8);
	}
}


/*
 * Function:	DMA_DeInit
 *
 * Brief: 		Reset the DMA peripheral
 *
 * Params: 		struct DMA_RegDef_t *pDMAx - DMA register pointer
 *
 */
void DMA_DeInit(DMA_RegDef_t *pDMAx)
{
	if (pDMAx == DMA1)
	{
		DMA1_RESET();
	}
	else if (pDMAx == DMA2)
	{
		DMA2_RESET();
	}
}


/*
 * Function:	DMA_GetFlagStatus
 *
 * Brief: 		Check status register of DMA to see if flag is set
 *
 * Params: 		struct ADC_RegDef_t *pADCx - ADC base address
 * 				uint8_t ADC_FLAG - flag being checked
 *
 */
uint8_t DMA_GetFlagStatus(DMA_Handle_t *pDMAxHandle, uint8_t DMA_FLAG)
{
	uint8_t streamNum = pDMAxHandle->DMA_Stream;
	uint8_t temp = streamNum % 4;
	uint32_t statusReg = (streamNum < 4) ? (pDMAxHandle->pDMAx->LISR) : (pDMAxHandle->pDMAx->HISR);

	switch (DMA_FLAG)
	{
		case DMA_FLAG_FEIF:
		{
			return (statusReg & (1 << FEIF_Flag[temp]));
		}
		case DMA_FLAG_DMEIF:
		{
			return (statusReg & (1 << DMEIF_Flag[temp]));
		}
		case DMA_FLAG_TEIF:
		{
			return (statusReg & (1 << TEIF_Flag[temp]));
		}
		case DMA_FLAG_HTIF:
		{
			return (statusReg & (1 << HTIF_Flag[temp]));
		}
		case DMA_FLAG_TCIF:
		{
			return (statusReg & (1 << TCIF_Flag[temp]));
		}
		default:
		{
			return 0;
		}
	}
}


/*
 * Function:	DMA_Start
 *
 * Brief: 		Blocking function to start DMA transfer.
 *
 * Params: 		DMA_Handle_t *pDMAxHandle - DMA handle address
 * 				uint32_t *pSrcAddr - source address
 * 				uint32_t *pDestArrd - destination address
 * 				uint16_t dataLen - length of data to transfer
 *
 */
void DMA_Start(DMA_Handle_t *pDMAxHandle, uint32_t *pSrcAddr, uint32_t *pDestArrd, uint16_t dataLen)
{
	uint8_t streamNum = pDMAxHandle->DMA_Stream;

	// 1. Set the source and destination address depending on direction
	if (pDMAxHandle->DMA_Config.DMA_Dir == DMA_MEM_TO_PERIPH)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SM0AR = (uint32_t)pSrcAddr;
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SPAR = (uint32_t)pDestArrd;
	}
	else
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SPAR = (uint32_t)pSrcAddr;
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SM0AR = (uint32_t)pDestArrd;
	}

	// 2. Set number of data items to transfer
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SNDTR = (uint32_t)(dataLen & 0xFF);

	// 3. Start the transfer
	DMA_StartStop(pDMAxHandle, ENABLE);

	// 4. Wait till transfer complete - wait for transfer complete flag
	DMA_GetFlagStatus(pDMAxHandle, DMA_FLAG_TCIF);
}



/************************************************ INTERRUPT CONFIG AND HANLDER APIS ****************************************/


/*
 * Function:	DMA_IRQConfig
 *
 * Brief: 		Set or clear interrupt in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t Value - Enabling or disabling interrupt (1 or 0)
 *
 */
void DMA_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		NVIC_ISER->ISER[IRQNumber / 32] |= (1 << (IRQNumber % 32));
	}
	else if (EnOrDi == DISABLE)
	{
		NVIC_ICER->ICER[IRQNumber / 32] |= (1 << (IRQNumber % 32));
	}
}


/*
 * Function:	DMA_IRQPriorityConfig
 *
 * Brief: 		Set interrupt priority in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t IRQPriority - Priority value (0 - 255)
 *
 */
void DMA_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t shift = 8 * (IRQNumber % 4) + NVIC_NONIMPL_LOW_BITS;
	NVIC_IPR->IPR[IRQNumber / 4] |= (IRQPriority << shift);
}


/*
 * Function:	DMA_EV_IRQHandling
 *
 * Brief: 		Handle interrupt events from DMAs
 *
 * Params: 		struct DMA_Handle_t *pDMAxHandle - DMA handle address
 *
 */
void DMA_EV_IRQHandling(DMA_Handle_t *pDMAxHandle)
{
	// Is transfer complete interrupt?
	if (DMA_GetFlagStatus(pDMAxHandle, DMA_FLAG_TCIF))
	{
		DMA_Trans_Cmplt_Handle_IT(pDMAxHandle);
	}
}


/*
 * Function:	DMA_Stream_IRQ_Enable
 *
 * Brief: 		Set the required IRQ interrupt in NVIC for the DMA register and stream number
 *
 * Params: 		DMA_Handle_t *pDMAxHandle - DMA handle address
 *
 */
static void DMA_Stream_IRQ_Enable(DMA_Handle_t *pDMAxHandle)
{
	if (pDMAxHandle->pDMAx == DMA1)
	{
		switch (pDMAxHandle->DMA_Stream)
		{
			case (DMA_STREAM_0):
			{
				DMA_IRQConfig(IRQ_DMA1_STREAM0, ENABLE);
			} break;
			case (DMA_STREAM_1):
			{
				DMA_IRQConfig(IRQ_DMA1_STREAM1, ENABLE);
			} break;
			case (DMA_STREAM_2):
			{
				DMA_IRQConfig(IRQ_DMA1_STREAM2, ENABLE);
			} break;
			case (DMA_STREAM_3):
			{
				DMA_IRQConfig(IRQ_DMA1_STREAM3, ENABLE);
			} break;
			case (DMA_STREAM_4):
			{
				DMA_IRQConfig(IRQ_DMA1_STREAM4, ENABLE);
			} break;
			case (DMA_STREAM_5):
			{
				DMA_IRQConfig(IRQ_DMA1_STREAM5, ENABLE);
			} break;
			case (DMA_STREAM_6):
			{
				DMA_IRQConfig(IRQ_DMA1_STREAM6, ENABLE);
			} break;
			case (DMA_STREAM_7):
			{
				DMA_IRQConfig(IRQ_DMA1_STREAM7, ENABLE);
			}
		}
	}
	else if (pDMAxHandle->pDMAx == DMA2)
	{
		switch (pDMAxHandle->DMA_Stream)
		{
			case (DMA_STREAM_0):
			{
				DMA_IRQConfig(IRQ_DMA2_STREAM0, ENABLE);
			} break;
			case (DMA_STREAM_1):
			{
				DMA_IRQConfig(IRQ_DMA2_STREAM1, ENABLE);
			} break;
			case (DMA_STREAM_2):
			{
				DMA_IRQConfig(IRQ_DMA2_STREAM2, ENABLE);
			} break;
			case (DMA_STREAM_3):
			{
				DMA_IRQConfig(IRQ_DMA2_STREAM3, ENABLE);
			} break;
			case (DMA_STREAM_4):
			{
				DMA_IRQConfig(IRQ_DMA2_STREAM4, ENABLE);
			} break;
			case (DMA_STREAM_5):
			{
				DMA_IRQConfig(IRQ_DMA2_STREAM5, ENABLE);
			} break;
			case (DMA_STREAM_6):
			{
				DMA_IRQConfig(IRQ_DMA2_STREAM6, ENABLE);
			} break;
			case (DMA_STREAM_7):
			{
				DMA_IRQConfig(IRQ_DMA2_STREAM7, ENABLE);
			}
		}
	}
}


/*
 * Function:	DMA_Start_IT
 *
 * Brief: 		Enable DMA transfer with interrupts.
 *
 * Params: 		DMA_Handle_t *pDMAxHandle - DMA handle address
 * 				uint32_t *pSrcAddr - source address
 * 				uint32_t *pDestArrd - destination address
 * 				uint16_t dataLen - length of data to transfer
 *
 */
uint8_t DMA_Start_IT(DMA_Handle_t *pDMAxHandle, uint32_t *pSrcAddr, uint32_t *pDestArrd, uint16_t dataLen)
{
	uint8_t state = pDMAxHandle->state;

	if (state == DMA_READY)
	{
		pDMAxHandle->state = DMA_BUSY;

		uint8_t streamNum = pDMAxHandle->DMA_Stream;

		// 1. Set the source and destination address depending on direction
		if (pDMAxHandle->DMA_Config.DMA_Dir == DMA_MEM_TO_PERIPH)
		{
			pDMAxHandle->pDMAx->DMA_Strm[streamNum].SM0AR = (uint32_t)pSrcAddr;
			pDMAxHandle->pDMAx->DMA_Strm[streamNum].SPAR = (uint32_t)pDestArrd;
		}
		else
		{
			pDMAxHandle->pDMAx->DMA_Strm[streamNum].SPAR = (uint32_t)pSrcAddr;
			pDMAxHandle->pDMAx->DMA_Strm[streamNum].SM0AR = (uint32_t)pDestArrd;
		}

		// 2. Set number of data items to transfer
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SNDTR = (uint32_t)(dataLen & 0xFF);

		// 3. Set transfer complete interrupt
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 4);
		DMA_Stream_IRQ_Enable(pDMAxHandle);

		// 4. Start the transfer
		DMA_StartStop(pDMAxHandle, ENABLE);
	}

	return state;
}


/*
 * Function:	DMA_Trans_Cmplt_Handle_IT
 *
 * Brief: 		Handle transfer complete events from DMAs
 *
 * Params: 		struct DMA_Handle_t *pDMAxHandle - DMA handle address
 *
 */
static void DMA_Trans_Cmplt_Handle_IT(DMA_Handle_t *pDMAxHandle)
{
	// 1. If not in circular mode and data to be transfered is 0, close transfer
	if (pDMAxHandle->DMA_Config.DMA_Mode != DMA_MODE_CIRC && pDMAxHandle->pDMAx->DMA_Strm[pDMAxHandle->DMA_Stream].SNDTR == 0)
	{
		DMA_Close_IT(pDMAxHandle);
	}
	else
	{
		DMA_ApplicationCallbackEvent(pDMAxHandle, DMA_TRANSFER_CMPLT);
	}
}


/*
 * Function:	DMA_Close_IT
 *
 * Brief: 		Closes the DMA transfers and resets state
 *
 * Params: 		struct DMA_Handle_t *pDMAxHandle - DMA handle address
 *
 */
void DMA_Close_IT(DMA_Handle_t *pDMAxHandle) {
	uint8_t streamNum = pDMAxHandle->DMA_Stream;

	// 1. Reset transfer complete interrupt
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR &= ~(1 << 4);

	// 2. Reset handle fields
	pDMAxHandle->state = DMA_READY;

	// 3. Send callback
	DMA_ApplicationCallbackEvent(pDMAxHandle, DMA_TRANSFER_CMPLT);
}



