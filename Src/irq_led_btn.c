/*
 * irq_led_btn.c
 *
 *  Created on: Mar. 26, 2020
 *      Author: Rahul
 */

#include <stdio.h>

#include "stm32f446xx.h"

#include "stm32f446xx_gpio_driver.h"


void delay(uint32_t multiplier)
{
	uint32_t timer = 0;

	// 16 MHz  internal CLK so multiplier = 1 gives 1 ms delay (4 clock cycles per computation)
	while (timer < (multiplier * 1000))
	{
		timer++;
	}
}

void EXTI15_10_IRQHandler() {
	delay(150);
	GPIO_IRQHandling(GPIO_PIN_NUM_12);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NUM_10);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioIrq;

	GpioIrq.pGPIOx = GPIOC;
	GpioIrq.GPIO_PinConfig.GPIO_PinNumber = 12;
	GpioIrq.GPIO_PinConfig.GPIO_PinMode = GPIO_IT_MODE_FT;
	GpioIrq.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioIrq.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_PU;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = 10;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	// Enable clock to GPIO pins and SYSCFG
	GPIO_PeriClockControl(GPIOC, ENABLE);
	SYSCFG_PCLK_EN();

	// Initialize LED driving and interrupt GPIO pin through EXTI
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioIrq);

	// Configure interrupt at processor(NVIC) end
	GPIO_IRQConfig(IRQ_POS_EXTI_15_10, ENABLE);

	// Set IRQ priority as 15
	GPIO_IRQPriorityConfig(IRQ_POS_EXTI_15_10, IRQ_PRIORITY_15);

	while(1)
	{

	}

	return 0;
}

