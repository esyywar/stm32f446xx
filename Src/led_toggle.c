/*
 * led_toggle.c
 *
 *  Created on: Mar. 24, 2020
 *      Author: Rahul
 */


#include <stdio.h>

#include "stm32f446xx.h"


void delay (uint32_t multiplier)
{
	uint32_t timer = 0;

	while (timer < (multiplier * 1000))
	{
		timer++;
	}
}

int main(void)
{
	// Want to blink external LED connected on PC10

	// 1. Enable clock signal to GPIO Port C on AHB1
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Handle_t GpioCLed;

	GpioCLed.pGPIOx = GPIOC;
	GpioCLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_10;
	GpioCLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioCLed.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	GpioCLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioCLed.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GpioCLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NUM_10);
		delay(1000);
	}

	return 0;
}
