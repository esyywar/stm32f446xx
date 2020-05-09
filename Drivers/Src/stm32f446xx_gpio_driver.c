/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Mar. 22, 2020
 *      Author: Rahul
 */

#include "stm32f446xx_gpio_driver.h"


/*
 * Function:	GPIO_PortCode
 *
 * Brief: 		Take GPIO address and return port code used to assign EXTI register
 *
 * Params: 		struct GPIO_RegDef_t* - GPIO register base address
 *
 */
uint8_t GPIO_PortCode(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		return 0;
	}
	else if (pGPIOx == GPIOB)
	{
		return 1;
	}
	else if (pGPIOx == GPIOC)
	{
		return 2;
	}
	else if (pGPIOx == GPIOD)
	{
		return 3;
	}
	else if (pGPIOx == GPIOE)
	{
		return 4;
	}
	else if (pGPIOx == GPIOF)
	{
		return 5;
	}
	else if (pGPIOx == GPIOG)
	{
		return 6;
	}
	else if (pGPIOx == GPIOH)
	{
		return 7;
	}

	return 0;
}

/*
 * Function:	GPIO_PeriClockControl
 *
 * Brief: 		Enable and disable clock signal to GPIO ports
 *
 * Params: 		struct GPIO_RegDef_t* - GPIO register base address
 * 				uint8_t - Enable or disable value
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/*
 * Function:	GPIO_Init
 *
 * Brief:		Initialize a GPIO pin
 *
 * Params:		struct GPIO_Handle_t*
 *
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOxHandle)
{
	// Enable clock to GPIO port
	GPIO_PeriClockControl(pGPIOxHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;

	// 1. Set GPIO pin mode
	if (pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOxHandle->pGPIOx->MODER &= ~((0x3) << (2 * pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOxHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else {
		//For setting to interrupt mode...

		// Enable clock to SYSCFG (needed to set IRQ in EXTI)
		SYSCFG_PCLK_EN();

		// 1. Set rising/falling edge trigger(s)
		if (pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_MODE_RT)
		{
			EXTI->RTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_MODE_FT)
		{
			EXTI->FTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_MODE_RFT)
		{
			EXTI->RTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure GPIO port selection in SYSCFG
		uint8_t temp1 = pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_PortCode(pGPIOxHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

		// 3. Enable EXTI interrupt using interrupt register masking
		EXTI->IMR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. Set pin speed
	temp = (pGPIOxHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOxHandle->pGPIOx->OSPEEDR &= ~((0x3) << (2 * pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOxHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Set pin pull-up or pull-down mode
	temp = (pGPIOxHandle->GPIO_PinConfig.GPIO_PuPdCtrl << (2 * pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOxHandle->pGPIOx->PUPDR &= ~((0x3) << (2 * pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOxHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. Configure pin output type (push-pull or open drain)
	temp = (pGPIOxHandle->GPIO_PinConfig.GPIO_OpType << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOxHandle->pGPIOx->OTYPER &= ~((0x1) <<  pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOxHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure alternate mode function
	if (pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOxHandle->pGPIOx->AFR[temp1] &= ~((0xF) << (4 * temp2));
		pGPIOxHandle->pGPIOx->AFR[temp1] |= (pGPIOxHandle->GPIO_PinConfig.GPIO_AfMode << (4 * temp2));
	}
}


/*
 * Function:	GPIO_DeInit
 *
 * Brief: 		Reset a GPIO pin
 *
 * Params: 		struct GPIO_RegDef_t* - GPIO register base address
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_RESET();
	}
}


/*************** INPUT AND OUTPUT OPERATIONS ************************/

/*
 * Function:	GPIO_ReadFromInputPin
 *
 * Brief: 		Read value from a GPIO pin
 *
 * Params: 		struct GPIO_RegDef_t* - GPIO register base address
 * 				uint8_t - GPIO pin number (0 - 15)
 *
 * Return:		uint8_t - Input pin value (0 or 1)
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*
 * Function:	GPIO_ReadFromInputPort
 *
 * Brief: 		Read value from a GPIO port
 *
 * Params: 		struct GPIO_RegDef_t* - GPIO register base address
 *
 * Return:		uint16_t - Input port values (16 1s and 0s)
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = (uint16_t)((pGPIOx->IDR) & 0x0000FFFF);
	return value;
}


/*
 * Function:	GPIO_WriteToOutputPin
 *
 * Brief: 		Write value to GPIO output pin
 *
 * Params: 		struct GPIO_RegDef_t* - GPIO register base address
 * 				uint8_t PinNumber - GPIO pin number (0- - 15)
 * 				uint8_t Value - Output value to write (0 or 1)
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*
 * Function:	GPIO_WriteToOutputPort
 *
 * Brief: 		Write value to GPIO output port
 *
 * Params: 		struct GPIO_RegDef_t* - GPIO register base address
 * 				uint16_t Value - Output value to write
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*
 * Function:	GPIO_ToggleOutputPin
 *
 * Brief: 		Toggle output value on the GPIO pin
 *
 * Params: 		struct GPIO_RegDef_t* - GPIO register base address
 * 				uint8_t Value - Pin number
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}



/*************** INTERRUPT OPERATIONS ************************/

/*
 * Function:	GPIO_IRQConfig
 *
 * Brief: 		Set or clear interrupt in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t Value - Enabling or disabling interrupt (1 or 0)
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * Function:	GPIO_IRQPriorityConfig
 *
 * Brief: 		Set interrupt priority in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t IRQPriority - Priority value (0 - 255)
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t shift = 8 * (IRQNumber % 4) + NVIC_NONIMPL_LOW_BITS;
	NVIC_IPR->IPR[IRQNumber / 4] = (IRQPriority << shift);
}


/*
 * Function:	GPIO_IRQHandling
 *
 * Brief: 		Clear the pending register for set interrupt
 *
 * Params: 		uint8_t PinNumber - GPIO pin number which interrupt is on
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
