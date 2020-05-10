/*
 * sys_timer_led.c
 *
 *  Created on: May 10, 2020
 *      Author: Rahul
 */

#include "stm32f446xx.h"

#define SYSTEM_CORE_CLOCK		16000000

// Track time using system timer
static volatile uint32_t msTicks;

void SysTick_Handler(void)
{
	msTicks++;
}

void Delay (uint32_t dlyTicks) {
	uint32_t curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks);
}

void LED_Init()
{
	// Initialization of LED driving GPIO
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GpioLed);
}

void SysTimerInit()
{
	SYSTCK_Config_t sysTimer;

	sysTimer.SYSTCK_ClkSource = SYSTCK_INT_CLK;
	sysTimer.SYSTCK_IRQ = SYSTCK_IRQ_EN;

	// Set timer to interrupt every 1ms
	sysTimer.SYSTCK_Reload = (SYSTEM_CORE_CLOCK / 1000);

	SYSTCK_Init(&sysTimer);
}

int main(void)
{
	LED_Init();

	SysTimerInit();

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		Delay(500);
	}

	return 0;
}


