/*
 * adc_read.c
 *
 *  Created on: Jun. 10, 2020
 *      Author: Rahul
 *
 *  Brief: This program tests the ADC single channel read. An analog reading is continuously read from AI0 (PA0)
 *  	and printed to the console by semihosting. *
 *
 */

#include "stm32f446xx.h"

// For semihosting
extern void initialise_monitor_handles(void);


/* Handle for ADC peripheral */
ADC_Handle_t Adc_Input;

/* value from ADC */
uint16_t value;


/* Initialize the GPIO Pin for ADC input */
void GPIO_AI_Init() {
	// Initialization of LED driving GPIO
	GPIO_Handle_t GpioAI;

	GpioAI.pGPIOx = GPIOA;

	GpioAI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_0;
	GpioAI.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GpioAI.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	GpioAI.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioAI.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GpioAI);
}

/* Initialize ADC1 for 12-bit readings */
void ADC1_In_Init() {
	Adc_Input.pADCx = ADC1;

	Adc_Input.ADC_Config.ADC_Res = ADC_RES_12BIT;
	Adc_Input.ADC_Config.ADC_PreSc = ADC_PCLK_DIV2;

	ADC_Init(&Adc_Input);
}

/* Call implemented ADC IRQ handler */
void ADC_IRQHandler() {
	ADC_EV_IRQHandling(&Adc_Input);
}

int main(void) {
	// Enable semi-hosting
	initialise_monitor_handles();

	/* initialize GPIO PA0 as AI */
	GPIO_AI_Init();

	/* initialize ADC1 */
	ADC1_In_Init();

	/* set data buffer */
	Adc_Input.pDataBuffer = &value;

	/* begin continuous read from ADC */
	ADC_Read_Reg_Channel(&Adc_Input, ADC_IN0, ADC_SMP_3CYC, ADC_CONT_READ);


	while(1) {

	}

	return 0;
}

void ADC_ApplicationCallbackEvent(ADC_Handle_t *pADCxHandle, uint8_t event)
{
	if (event == ADC_READ_CMPLT)
	{
		printf("ADC value is: [%i]\n", value);
	}
}


