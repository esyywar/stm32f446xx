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
	 *	Want to toggle the external LED connected at PC10 when external button is pressed
	 *
	 *	LED is connected with resistor in series at PC10
	 *	Button is connected at PC12 and pulls low on press
	 *
	 */

	GPIO_Handle_t GpioBtn, GpioLed;

	// Parameters for button GPIO
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioBtn.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_PU;

	// Parameters for LED GPIO
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_10;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);
	GPIO_Init(&GpioLed);

	while(1)
	{
		if (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_12)))
		{
			delay(200);
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NUM_10);
		}
	}

	return 0;
}


