/*
 * stm32f446xx_dma_driver.c
 *
 *  Created on: Jun. 11, 2020
 *      Author: Rahul
 */


#include "stm32f446xx.h"


/***************************** SHIFT VALUES FOR INTERRUPT FLAGS IN DMA SR (LISR AND HISR) ***************************/

uint8_t TCIF_Flag[4] = {5, 11, 21, 27};
uint8_t HTIF_Flag[4] = {4, 10, 20, 26};
uint8_t TEIF_Flag[4] = {3, 9, 19, 25};
uint8_t DMEIF_Flag[4] = {2, 8, 18, 24};
uint8_t FEIF_Flag[4] = {0, 6, 16, 22};


/****************************** LOCAL FUNCTIONS *******************************/

static void DMA_StartStop(DMA_Handle_t *pDMAxHandle, uint8_t EnOrDi);



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
	uint8_t streamNum = pDMAxHandle->DMA_Stream;

	// 1. Set the channel number
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_Channel << 25);

	// 2. Stream priority
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_Priority << 16);

	// 3. FIFO mode (direct mode)
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR |= (pDMAxHandle->DMA_Config.DMA_FIFOMode << 2);

	// 4. FIFO threshold
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR &= ~(0x3 << 0);
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR |= (pDMAxHandle->DMA_Config.DMA_FIFOThresh << 0);

	// 5. Set transfer direction
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_Dir << 6);

	// 6. Peripheral increment
	if (pDMAxHandle->DMA_Config.DMA_PeriphInc == DMA_PERIPH_INC)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 9);
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR &= ~(1 << 15);
	}

	// 7. Memory increment
	if (pDMAxHandle->DMA_Config.DMA_MemInc == DMA_MEM_INC)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 10);
	}

	// 8. Peripheral burst
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_PBurst << 21);

	// 9. Memory burst
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_MBurst << 23);

	// 10. Set peripheral and memory data sizes
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_PeriphDataSize << 11);
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_MemDataSize << 13);

	// 11. Circle mode
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_CircMode << 8);
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
	if (streamNum > 3)
	{
		// GET FLAG
	}
	else
	{
		// GET FLAG
	}
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


