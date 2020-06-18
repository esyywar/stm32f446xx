/*
 * stm32f446xx_adc_driver.c
 *
 *  Created on: Jun. 10, 2020
 *      Author: Rahul
 */

#include "stm32f446xx_adc_driver.h"


/****************************** LOCAL FUNCTIONS *******************************/

static void ADC_OnOff(ADC_RegDef_t *pADCx, uint8_t EnOrDi);
static void ADC_Read_IT_Handle(ADC_Handle_t *pADCxHandle);


/*********************************************** DRIVER API IMPLMENTATION ******************************************/

/*
 * Function:	ADC_PeriClockControl
 *
 * Brief: 		Enable and disable clock signal to ADC peripherals
 *
 * Params: 		struct ADC_RegDef_t* *pADCx - ADC base address
 * 				uint8_t EnOrDi - Enable or disable value
 *
 */
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pADCx == ADC1)
		{
			ADC1_PCLK_EN();
		}
		else if (pADCx == ADC2)
		{
			ADC2_PCLK_EN();
		}
		else if (pADCx == ADC3)
		{
			ADC3_PCLK_EN();
		}
	}
	else
	{
		if (pADCx == ADC1)
		{
			ADC1_PCLK_DI();
		}
		else if (pADCx == ADC2)
		{
			ADC2_PCLK_DI();
		}
		else if (pADCx == ADC3)
		{
			ADC3_PCLK_DI();
		}
	}
}


/*
 * Function:	ADC_OnOff
 *
 * Brief: 		Turn the ADC on or put in low power mode
 *
 * Params:		ADC_RegDef_t *pADCx - ADC base address
 * 				uint8_t EnOrDi - On or Off
 *
 */
static void ADC_OnOff(ADC_RegDef_t *pADCx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pADCx->CR2 |= (1 << 0);
	}
	else
	{
		pADCx->CR2 &= ~(1 << 0);
	}
}


/*
 * Function:	ADC_Init
 *
 * Brief: 		Initialize the ADC peripherals
 *
 * Params: 		struct ADC_Handle_t* *pADCx - ADC handle address
 *
 */
void ADC_Init(ADC_Handle_t *pADCxHandle)
{
	// 1. Enable clock signal to the ADC
	ADC_PeriClockControl(pADCxHandle->pADCx, ENABLE);

	// 2. Set the ADC clock prescaler
	ADC_COMM->CCR |= (uint32_t)(pADCxHandle->ADC_Config.ADC_PreSc << 16);

	// 3. Set ADC resolution
	pADCxHandle->pADCx->CR1 |= (uint32_t)(pADCxHandle->ADC_Config.ADC_Res << 24);

	// 4. Continuous or single read mode
	if (pADCxHandle->ADC_Config.ADC_Mode == ADC_SINGLE_READ)
	{
		pADCxHandle->pADCx->CR2 &= ~(1 << 1);
	}
	else if (pADCxHandle->ADC_Config.ADC_Mode == ADC_CONT_READ)
	{
		pADCxHandle->pADCx->CR2 |= (1 << 1);
	}
}


/*
 * Function:	ADC_DeInit
 *
 * Brief: 		Deinitialize the ADC peripherals
 *
 */
void ADC_DeInit()
{
	ADC_RESET();
}


/*
 * Function:	ADC_GetFlagStatus
 *
 * Brief: 		Check status register of ADC to see if flag is set
 *
 * Params: 		struct ADC_RegDef_t *pADCx - ADC base address
 * 				uint8_t ADC_FLAG - flag being checked
 *
 */
uint8_t ADC_GetFlagStatus(ADC_RegDef_t *pADCx, uint8_t ADC_FLAG)
{
	return (pADCx->SR & ADC_FLAG);
}


/*
 * Function:	ADC_Read_Reg
 *
 * Brief: 		Returns a single conversion from a regular ADC channel.
 *
 * Params: 		struct ADC_Handle_t* *pADCx - ADC handle address
 * 				uint8_t ADC_CHAN - ADC channel designation
 * 				uint8_t ADC_SMP_CYC - Number of cycle to take reading
 * 				uint8_t ADC_DAQ_MODE - Take single reading or continuous mode
 *
 */
uint16_t ADC_Read_Reg(ADC_Handle_t *pADCxHandle, uint8_t ADC_CHAN, uint8_t ADC_SMP_CYC)
{
	// 1. Set the ADC On (Note: several steps b/w this and 'START' to allow stabilization time
	ADC_OnOff(pADCxHandle->pADCx, ENABLE);

	// 2. Disable scan mode
	pADCxHandle->pADCx->CR1 &= ~(1 << 8);

	// 3. Disable continuous mode
	pADCxHandle->pADCx->CR2 &= ~(1 << 1);

	// 4. Set number of channels in sequence to 1
	pADCxHandle->pADCx->SQR[0] &= ~(0xF << 20);

	// 5. Load the channel to be read
	pADCxHandle->pADCx->SQR[2] = (ADC_CHAN << 0);

	// 6. Number of sampling cycles
	uint8_t temp1 = 1 - ADC_CHAN / 10, temp2 = ADC_CHAN % 9;
	pADCxHandle->pADCx->SMPR[temp1] = (ADC_SMP_CYC << temp2);

	// 7. Begin conversion
	pADCxHandle->pADCx->CR2 |= (1 << 30);

	// 8. Hang till conversion complete
	while(!ADC_GetFlagStatus(pADCxHandle->pADCx, ADC_FLAG_EOC));

	// 9. Get reading from DR
	uint16_t data = pADCxHandle->pADCx->DR;

	// 10. Return ADC to low power mode
	ADC_OnOff(pADCxHandle->pADCx, DISABLE);

	return data;
}


/*
 * Function:	ADC_Scan_Reg
 *
 * Brief: 		Read a scan of the given ADC channels.
 *
 * Params: 		struct ADC_Handle_t* *pADCx - ADC handle address
 * 				uint8_t NUM_CHAN - number of channels to scan
 * 				uint8_t *pSeqBuffer - pointer to buffer of channel sequence
 * 				uint8_t *pSmpBuffer - pointer to buffer of sample cycles for each channel
 * 				uint16_t *pDataBuffer - pointer to buffer to store ADC data
 *
 */
void ADC_Scan_Reg(ADC_Handle_t *pADCxHandle, uint8_t NUM_CHAN, uint8_t *pSeqBuffer, uint8_t *pSmpBuffer, uint16_t *pDataBuffer)
{
	// 1. Set the ADC On (Note: several steps b/w this and 'START' to allow stabilization time
	ADC_OnOff(pADCxHandle->pADCx, ENABLE);

	// 2. Set scan mode
	pADCxHandle->pADCx->CR1 |= (1 << 8);

	// 3. Can take only single reading without non-blocking interrupt
	pADCxHandle->pADCx->CR2 &= ~(1 << 1);

	// 4. Set number of channels in the sequence
	pADCxHandle->pADCx->SQR[0] |= (NUM_CHAN << 20);

	// 4. Load the sequence of channels to be read and sample cycles for each channel
	for (int i = 0; i < NUM_CHAN; i++)
	{
		uint8_t temp1 = i / 7, temp2 = i % 6, temp3 = i / 10, temp4 = i % 9;

		// Set channel in sequence
		pADCxHandle->pADCx->SQR[2 - temp1] |= (uint32_t)(*pSeqBuffer << (temp2 * 5));
		pSeqBuffer++;

		// Set sampling cycle for channel
		pADCxHandle->pADCx->SMPR[1 - temp3] |= (uint32_t)(*pSmpBuffer << (temp4 * 3));
		pSmpBuffer++;
	}

	// 5. Enable end of conversion flag on each reading
	pADCxHandle->pADCx->CR2 |= (1 << 10);

	// 6. Take readings from conversion
	for (int i = 0; i < NUM_CHAN; i++)
	{
		while(!ADC_GetFlagStatus(pADCxHandle->pADCx, ADC_FLAG_EOC));
		*pDataBuffer = (uint16_t)(pADCxHandle->pADCx->DR & 0xFF);
		pDataBuffer++;
	}

	// 7. Return ADC to low power state
	ADC_OnOff(pADCxHandle->pADCx, DISABLE);
}


/*
 * Function:	ADC_Read_Close
 *
 * Brief: 		End continuous read operation on an ADC channel.
 *
 * Params: 		struct ADC_Handle_t* *pADCx - ADC handle address
 *
 */
void ADC_Cont_Close(ADC_Handle_t *pADCxHandle)
{
	ADC_OnOff(pADCxHandle->pADCx, DISABLE);
	pADCxHandle->state = ADC_READY;
}



/************************************************ INTERRUPT CONFIG AND HANLDER APIS ****************************************/


/*
 * Function:	ADC_IRQConfig
 *
 * Brief: 		Set or clear interrupt in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t Value - Enabling or disabling interrupt (1 or 0)
 *
 */
void ADC_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * Function:	ADC_IRQPriorityConfig
 *
 * Brief: 		Set interrupt priority in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t IRQPriority - Priority value (0 - 255)
 *
 */
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t shift = 8 * (IRQNumber % 4) + NVIC_NONIMPL_LOW_BITS;
	NVIC_IPR->IPR[IRQNumber / 4] |= (IRQPriority << shift);
}


/*
 * Function:	ADC_Read_Reg_Channel
 *
 * Brief: 		Configures regular ADC channel for single or continuous reading. Interrupt set on EOC to read value.
 *
 * Params: 		struct ADC_Handle_t* *pADCx - ADC handle address
 * 				uint8_t ADC_CHAN - ADC channel designation
 * 				uint8_t ADC_SMP_CYC - Number of cycle to take reading
 * 				uint8_t ADC_DAQ_MODE - Take single reading or continuous mode
 *
 */
uint8_t ADC_Read_Reg_IT(ADC_Handle_t *pADCxHandle, uint8_t ADC_CHAN, uint8_t ADC_SMP_CYC)
{
	uint8_t state = pADCxHandle->state;

	if (state == ADC_READY)
	{
		pADCxHandle->state = ADC_BUSY_READ;

		// 1. Set the ADC On (Note: several steps b/w this and 'START' to allow stabilization time
		ADC_OnOff(pADCxHandle->pADCx, ENABLE);

		// 2. Disable scan mode
		pADCxHandle->pADCx->CR1 &= ~(1 << 8);

		// 3. Set number of channels in sequence to 1
		pADCxHandle->pADCx->SQR[0] &= ~(0xF << 20);

		// 4. Load the channel to be read
		pADCxHandle->pADCx->SQR[2] = (ADC_CHAN << 0);

		// 5. Number of sampling cycles
		uint8_t temp1 = 1 - ADC_CHAN / 10, temp2 = ADC_CHAN % 9;
		pADCxHandle->pADCx->SMPR[temp1] = (ADC_SMP_CYC << temp2);

		// 6. Enable end of conversion interrupt
		pADCxHandle->pADCx->CR1 |= (1 << 5);
		ADC_IRQConfig(IRQ_POS_ADC, ENABLE);

		// 7. Set DMA transfer if requested
		pADCxHandle->pADCx->CR2 |= (pADCxHandle->ADC_Config.ADC_DMA_En << 8);
		pADCxHandle->pADCx->CR2 |= (pADCxHandle->ADC_Config.ADC_DMA_Cont << 9);

		// 8. Either set trigger or begin conversion
		if (pADCxHandle->ADC_Config.ADC_Trig_Pol == ADC_EXTEN_DI)
		{
			// Start conversion of regular channels
			pADCxHandle->pADCx->CR2 |= (1 << 30);
		}
		else
		{
			// Configure external trigger and trigger source
			pADCxHandle->pADCx->CR2 |= (uint32_t)(pADCxHandle->ADC_Config.ADC_Trig_Pol << 28) | (uint32_t)(pADCxHandle->ADC_Config.ADC_Trig_Src << 24);
		}
	}

	return state;
}


/*
 * Function:	ADC_EV_IRQHandling
 *
 * Brief: 		Handle interrupt events from ADCs
 *
 * Params: 		struct ADC_Handle_t* *pADCx - ADC handle address
 *
 */
void ADC_EV_IRQHandling(ADC_Handle_t *pADCxHandle)
{
	// 1. Is analog watchdog interrupt?
	if (ADC_GetFlagStatus(pADCxHandle->pADCx, ADC_FLAG_AWD))
	{
		pADCxHandle->pADCx->SR &= ~(1 << 5);
	}

	// 2. Is end of conversion interrupt?
	if (ADC_GetFlagStatus(pADCxHandle->pADCx, ADC_FLAG_EOC))
	{
		ADC_Read_IT_Handle(pADCxHandle);
	}

	// 3. Is injected channel end of conversion interrupt?
	if (ADC_GetFlagStatus(pADCxHandle->pADCx, ADC_FLAG_JEOC))
	{
		pADCxHandle->pADCx->SR &= ~(1 << 3);
	}

	// 4. Is injected channel start interrupt?
	if (ADC_GetFlagStatus(pADCxHandle->pADCx, ADC_FLAG_JSTRT))
	{
		pADCxHandle->pADCx->SR &= ~(1 << 2);
	}

	// 5. Is regular channel start interrupt?
	if (ADC_GetFlagStatus(pADCxHandle->pADCx, ADC_FLAG_STRT))
	{
		pADCxHandle->pADCx->SR &= ~(1 << 1);
	}

	// 5. Is overrun interrupt?
	if (ADC_GetFlagStatus(pADCxHandle->pADCx, ADC_FLAG_OVR))
	{
		pADCxHandle->pADCx->SR &= ~(1 << 0);
	}
}


/*
 * Function:	ADC_Read_IT_Handle
 *
 * Brief: 		Called by interrupt event handler when EOC is raised. Reads data from DR.
 *
 * Params: 		struct ADC_Handle_t* *pADCx - ADC handle address
 *
 */
void ADC_Read_IT_Handle(ADC_Handle_t *pADCxHandle)
{
	// Read value from DR into buffer
	*(pADCxHandle->pDataBuffer) = pADCxHandle->pADCx->DR & 0xFFFF;

	if (pADCxHandle->ADC_Config.ADC_Mode == ADC_CONT_READ)
	{
		// If in continuous mode, send callback on each read
		ADC_ApplicationCallbackEvent(pADCxHandle, ADC_READ_CMPLT);
	}
	else if (pADCxHandle->ADC_Config.ADC_Mode == ADC_SINGLE_READ)
	{
		// If not in continuous, adjust data buffers and length
		(pADCxHandle->pDataBuffer)++;
		pADCxHandle->dataLen--;

		// If reached end of data sequence, low power mode and send callback
		if (pADCxHandle->dataLen == 0)
		{
			ADC_OnOff(pADCxHandle->pADCx, DISABLE);
			ADC_ApplicationCallbackEvent(pADCxHandle, ADC_READ_CMPLT);
		}
	}
}

