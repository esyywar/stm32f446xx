/*
 * stm32f446xx_dma_driver.c
 *
 *  Created on: Jun. 11, 2020
 *      Author: Rahul
 */


#include "stm32f446xx.h"


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

	// 2. Set transfer direction
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_Dir << 6);

	// 3. Set peripheral and memory data sizes
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_PeriphDataSize << 11);
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_MemDataSize << 13);


	// 4. Peripheral increment
	if (pDMAxHandle->DMA_Config.DMA_PeriphInc == DMA_PERIPH_INC)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 9);
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR &= ~(1 << 15);
	}

	// 4. Memory increment
	if (pDMAxHandle->DMA_Config.DMA_MemInc == DMA_MEM_INC)
	{
		pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (1 << 10);
	}

	// 5. Peripheral burst
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_PBurst << 21);

	// 6. Memory burst
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_MBurst << 23);

	// 5. Circle mode
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_CircMode << 8);

	// 6. Stream priority
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SCR |= (pDMAxHandle->DMA_Config.DMA_Priority << 16);

	// 7. FIFO mode (direct mode)
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR |= (pDMAxHandle->DMA_Config.DMA_FIFOMode << 2);

	// 8. FIFO threshold
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR &= ~(0x3 << 0);
	pDMAxHandle->pDMAx->DMA_Strm[streamNum].SFCR |= (pDMAxHandle->DMA_Config.DMA_FIFOThresh << 0);
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




