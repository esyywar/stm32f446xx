/*
 * adc_read_to_usart.c
 *
 *  Created on: Jun. 11, 2020
 *      Author: Rahul
 *
 *  Brief: This program tests the single channel read of ADC. Value from AI0 (PA0) is continuously read and output
 *  		to PC through ST-Link using USART2. The output values can be read using any serial monitor program.
 *
 */


#include "stm32f446xx.h"


/* Handle for ADC peripheral */
ADC_Handle_t Adc_Input;

/* value from ADC */
uint16_t value;

/* Handle for USART2 transmitter to PC */
USART_Handle_t Usart_Handle;


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


/*
 * Set GPIO pins PA2 and PA3 as USART2 functions
 */
void GPIO_USART_Config()
{
	GPIO_Handle_t Gpio_usartHandle;
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_PU;
	Gpio_usartHandle.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	Gpio_usartHandle.GPIO_PinConfig.GPIO_AfMode = GPIO_AF_7;
	Gpio_usartHandle.pGPIOx = GPIOA;

	// Set PA2 as USART2 Tx
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_2;
	GPIO_Init(&Gpio_usartHandle);

	// Set PA3 as USART2 Rx
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_3;
	GPIO_Init(&Gpio_usartHandle);
}


/*
 * Configure USART2 in Tx mode only
 */
void USART2_Config()
{
	Usart_Handle.USART_Config.USART_Mode = USART_TX_ONLY;
	Usart_Handle.USART_Config.USART_BaudRate = USART_BAUD_9600;
	Usart_Handle.USART_Config.USART_ParitySet = USART_PARITY_DISABLE;
	Usart_Handle.USART_Config.USART_StopBits = USART_1_STOP;
	Usart_Handle.USART_Config.USART_WordLen = USART_8BIT_DATA;
	Usart_Handle.USART_Config.USART_Oversampling = USART_OVERSMPL_8;
	Usart_Handle.USART_Config.USART_HWFlowCtrl = USART_HWCTRL_NONE;
	Usart_Handle.pUSARTx = USART2;

	USART_Init(&Usart_Handle);
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
	/* Initialize GPIO PA0 as AI */
	GPIO_AI_Init();

	/* Configure GPIO to USART2 and initialize USART2 Tx */
	GPIO_USART_Config();
	USART2_Config();

	/* Initialize ADC1 */
	ADC1_In_Init();

	/* Set data buffer */
	Adc_Input.pDataBuffer = &value;

	/* Begin continuous read from ADC */
	ADC_Read_Reg_IT(&Adc_Input, ADC_IN0, ADC_SMP_3CYC, ADC_CONT_READ);


	while(1) {
		/* Infinite loop - data is read and output in ADC interrupt callback */
	}

	return 0;
}

void ADC_ApplicationCallbackEvent(ADC_Handle_t *pADCxHandle, uint8_t event)
{
	if (event == ADC_READ_CMPLT)
	{
		char output[50];
		sprintf(output, "ADC output is: [%i]\r\n", value);
		USART_Write(&Usart_Handle, (uint8_t*)output, strlen(output));
	}
}


