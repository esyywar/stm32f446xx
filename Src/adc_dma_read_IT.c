/*
 * adc_dma_read_IT.c
 *
 *  Created on: Jun. 17, 2020
 *      Author: Rahul
 *
 *  Brief: Test for DMA driver. ADC1 reads in continuous loop while the DR
 *  is read by DMA. Value read is output via USART2.
 *
 */


#include "stm32f446xx.h"


/* Handle for ADC peripheral */
ADC_Handle_t Adc_Input;

/* Handle for USART2 transmitter to PC */
USART_Handle_t Usart_Handle;

/* Handle for DMA2 */
DMA_Handle_t Dma_adcHandle;

/* Variable for DMA to transfer to */
uint16_t data = 0;


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
	Adc_Input.ADC_Config.ADC_Mode = ADC_CONT_READ;
	Adc_Input.ADC_Config.ADC_DMA_Ctrl = ADC_DMA_EN;
	Adc_Input.ADC_Config.ADC_DDS_Ctrl = ADC_DDS_EN;

	ADC_Init(&Adc_Input);
}

/* Initialize DMA2 stream 0 to channel 0 for ADC1 */
void DMA2_Init() {
	Dma_adcHandle.pDMAx = DMA2;
	Dma_adcHandle.DMA_Stream = DMA_STREAM_0;

	Dma_adcHandle.DMA_Config.DMA_Channel = DMA_CHANNEL_0;
	Dma_adcHandle.DMA_Config.DMA_Priority = DMA_PRIOR_HIGH;
	Dma_adcHandle.DMA_Config.DMA_Dir = DMA_PERIPH_TO_MEM;
	Dma_adcHandle.DMA_Config.DMA_PeriphInc = DMA_PERIPH_NO_INC;
	Dma_adcHandle.DMA_Config.DMA_MemInc = DMA_MEM_NO_INC;
	Dma_adcHandle.DMA_Config.DMA_MemDataSize = DMA_DATASIZE_HALFWORD;
	Dma_adcHandle.DMA_Config.DMA_PeriphDataSize = DMA_DATASIZE_HALFWORD;
	Dma_adcHandle.DMA_Config.DMA_Mode = DMA_MODE_CIRC;
	Dma_adcHandle.DMA_Config.DMA_FIFOMode = DMA_DIRECT_EN;

	DMA_Init(&Dma_adcHandle);
}

void USART2_Output() {
	/* Output to USART2 */
	char output[50];
	sprintf(output, "ADC output is: [%i]\r\n", data);
	USART_Write(&Usart_Handle, (uint8_t*)output, strlen(output));
}

/* Call implemented ADC IRQ handler */
void ADC_IRQHandler() {
	ADC_EV_IRQHandling(&Adc_Input);
}

/* Call implemented DMA IRQ Handler */
void DMA2_Stream0_IRQHandler() {
	DMA_EV_IRQHandling(&Dma_adcHandle);
}


int main(void) {
	/* Initialize GPIO PA0 as AI */
	GPIO_AI_Init();

	/* Configure GPIO to USART2 and initialize USART2 Tx */
	GPIO_USART_Config();
	USART2_Config();

	/* Initialize ADC1 */
	ADC1_In_Init();

	/* Initialize DMA to transfer from ADC DR to data variable */
	DMA2_Init();

	/* Set interrupt ready */
	while(DMA_Start_IT(&Dma_adcHandle, (uint32_t*)&ADC1->DR, (uint32_t*)&data, (uint16_t)1) != DMA_READY);

	/* Begin continuous read from ADC */
	while(ADC_Read_Reg_IT(&Adc_Input, ADC_IN0, ADC_SMP_480CYC) != ADC_READY);

	while(1) {

	}

	return 0;
}

/* When DMA returns complete, output ADC result on USART2 */
void DMA_ApplicationCallbackEvent(DMA_Handle_t *pDMAxHandle, uint8_t event) {
	if (event == DMA_TRANSFER_CMPLT)
	{
		USART2_Output();
	}
}



