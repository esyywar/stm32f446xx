/*
 * stm32f446xx_adc_driver.c
 *
 *  Created on: Jun. 10, 2020
 *      Author: Rahul
 */

#include "stm32f446xx_adc_driver.h"


/****************************** LOCAL FUNCTIONS *******************************/

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

	// 4. Set external trigger
	pADCxHandle->pADCx->CR2 |= (uint32_t)(pADCxHandle->ADC_Config.ADC_Ext_Trig << 28);
}


/*
 * Function:	ADC_DeInit
 *
 * Brief: 		Deinitialize the ADC peripherals
 *
 *
 */
void ADC_DeInit()
{
	ADC_RESET();
}


/*
 * Function:	ADC_SingleChannel
 *
 * Brief: 		Take ADC reading(s) from a single channel
 *
 * Params: 		struct ADC_Handle_t* *pADCx - ADC handle address
 * 				uint8_t adcChan - ADC channel designation
 * 				uint8_t sampleCycles - Number of cycle to take reading
 * 				uint8_t adcReadMode - Take single reading or continuous mode
 *
 */
void ADC_Read_Channel(ADC_Handle_t *pADCxHandle, uint8_t ADC_CHAN, uint8_t ADC_SMP_CYC, uint8_t ADC_DAQ_MODE)
{
	// 1. Set the ADC On (Note: several steps b/w this and 'START' to allow stabilization time
	pADCxHandle->pADCx->CR2 |= (1 << 0);

	// 2. Reset ADC continuous mode for single reading
	if (ADC_DAQ_MODE == ADC_SINGLE_READ)
	{
		pADCxHandle->pADCx->CR2 &= ~(1 << 1);
	}
	else if (ADC_DAQ_MODE == ADC_CONT_READ)
	{
		pADCxHandle->pADCx->CR2 |= (1 << 1);
	}

	// 3. Set number of channels in sequence to 1
	pADCxHandle->pADCx->SQR[0] &= ~(0xF << 20);

	// 4. Load the channel to be read
	pADCxHandle->pADCx->SQR[2] = (ADC_CHAN << 0);

	// 5. Number of sampling cycles
	pADCxHandle->pADCx->SMPR[1] = (ADC_SMP_CYC << 0);

	// 6. Enable end of conversion interrupt
	pADCxHandle->pADCx->CR1 |= (1 << 5);
	ADC_IRQConfig(IRQ_POS_ADC, ENABLE);

	// 7. Set start conversion of regular channels
	pADCxHandle->pADCx->CR2 |= (1 << 30);
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
 * Function:	ADC_EV_IRQHandling
 *
 * Brief: 		Handle interrupt events from ADCs
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t IRQPriority - Priority value (0 - 255)
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

void ADC_Read_IT_Handle(ADC_Handle_t *pADCxHandle)
{
	// 1. Read value from DR into buffer
	*(pADCxHandle->pDataBuffer) = pADCxHandle->pADCx->DR & 0xFFFF;

	// 2. Send callback
	ADC_ApplicationCallbackEvent(pADCxHandle, ADC_READ_CMPLT);
}


