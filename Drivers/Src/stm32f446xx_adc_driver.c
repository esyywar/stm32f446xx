/*
 * stm32f446xx_adc_driver.c
 *
 *  Created on: Jun. 10, 2020
 *      Author: Rahul
 */

#include "stm32f446xx_adc_driver.h"


/*********************************************** DRIVER API IMPLMENTATION ******************************************/

/*
 * Function:	ADC_PeriClockControl
 *
 * Brief: 		Enable and disable clock signal to ADC peripherals
 *
 * Params: 		struct ADC_RegDef_t* *pUSARTx - USART/UART base address
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


