/*
 * led_button.c
 *
 *  Created on: Mar. 24, 2020
 *      Author: Rahul
 */


#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"


void delay(uint32_t multiplier)
{
	uint32_t timer = 0;

	while (timer < (multiplier * 1000))
	{
		timer++;
	}
}

int main(void)
{
	/*
	 *	Want to toggle the on-board LED when on-board button is pressed
	 *
	 *	The on-board button is connected to PC13 which is pulled low when button pressed
	 *	The on-board LED is connected at PA5 with no pull-up/pull-down
	 *
	 */

	// 1. Enable clock signal to GPIO Port C and Port A on AHB1
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Handle_t GpioBtn, GpioLed;

	// Initialization of button GPIO
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioBtn.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GpioBtn);

	// Initialization of LED driving GPIO
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GpioLed);

	while(1)
	{
		if (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13)))
		{
			delay(200);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		}
	}

	return 0;
}


