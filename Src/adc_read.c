/*
 * adc_read.c
 *
 *  Created on: Jun. 10, 2020
 *      Author: Rahul
 */

#include "stm32f446xx.h"

// For semihosting
extern void initialise_monitor_handles(void);


/* Handle for ADC peripheral */
ADC_Handle_t Adc_Input;

/* value from adc */
uint16_t value;

/* Handle for USART2 */
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

	// Set PA9 as USART1 Tx
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_2;
	GPIO_Init(&Gpio_usartHandle);

	// Set PA10 as USART1 Rx
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_3;
	GPIO_Init(&Gpio_usartHandle);
}

/*
 * Configure USART2 in Tx
 */
void USART2_Config()
{
	// Handle globally defined
	Usart_Handle.USART_Config.USART_Mode = USART_RX_TX;
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
	Adc_Input.ADC_Config.ADC_Ext_Trig = ADC_EXTEN_DI;

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

	/* initialize UART2 for sending data via ST-Link */
	USART2_Config();

	/* initialize ADC1 */
	ADC1_In_Init();

	/* set data buffer */
	Adc_Input.pDataBuffer = &value;

	/* begin continuous read from ADC */
	ADC_Read_Channel(&Adc_Input, ADC_IN0, ADC_SMP_3CYC, ADC_CONT_READ);


	while(1) {

	}

	return 0;
}

void ADC_ApplicationCallbackEvent(ADC_Handle_t *pADCxHandle, uint8_t event)
{
	if (event == ADC_READ_CMPLT)
	{
		USART_Write(&Usart_Handle, (uint8_t*)&value, (uint8_t)2);
		printf("ADC value is: [%i]\n", value);
	}
}


