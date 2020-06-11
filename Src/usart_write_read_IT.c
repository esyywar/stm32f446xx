/*
 * usart_write_read_IT.c
 *
 *  Created on: Apr. 20, 2020
 *      Author: Rahul
 *
 *  Brief: Program sends a character to arduino and reads back a character using UART
 *  	   Arduino sketch used is 001UARTRxString (works with 002UARTTxString also)
 *
 */


#include "stm32f446xx.h"


// For semihosting
extern void initialise_monitor_handles(void);

// Handle for USART1
USART_Handle_t Usart1_Handle;

// Status variable for feeback from call back function
uint8_t status = RESET;


void delay(uint32_t multiplier)
{
	uint32_t timer = 0;

	// 16 MHz  internal CLK so multiplier = 1 gives 1 ms delay (4 clock cycles per computation)
	while (timer < (multiplier * 1000))
	{
		timer++;
	}
}

/*
 * Set GPIO pins PA9 - PA12 as USART1 functions
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
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_9;
	GPIO_Init(&Gpio_usartHandle);

	// Set PA10 as USART1 Rx
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_10;
	GPIO_Init(&Gpio_usartHandle);

	// Set PA11 as USART1 CTS
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_11;
	GPIO_Init(&Gpio_usartHandle);

	// Set PA12 as USART1 RTS
	Gpio_usartHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GPIO_Init(&Gpio_usartHandle);
}

/*
 * Configure USART1 in Tx
 */
void USART1_Config(USART_Handle_t *pUsart_Handle)
{
	// Handle globally defined
	pUsart_Handle->USART_Config.USART_Mode = USART_RX_TX;
	pUsart_Handle->USART_Config.USART_BaudRate = USART_BAUD_115200;
	pUsart_Handle->USART_Config.USART_ParitySet = USART_PARITY_DISABLE;
	pUsart_Handle->USART_Config.USART_StopBits = USART_1_STOP;
	pUsart_Handle->USART_Config.USART_WordLen = USART_8BIT_DATA;
	pUsart_Handle->USART_Config.USART_Oversampling = USART_OVERSMPL_8;
	pUsart_Handle->USART_Config.USART_HWFlowCtrl = USART_HWCTRL_NONE;
	pUsart_Handle->pUSARTx = USART1;

	USART_Init(pUsart_Handle);
}

/*
 * Set on board button as input (PC13) and LED driving GPIO pin as output (PA5)
 */
void GPIO_OnBoardLedBtn_Set()
{
	GPIO_Handle_t GpioBtn, GpioLed;
	GPIOA_PCLK_EN();
	GPIOC_PCLK_EN();

	// GPIO input from on-board button
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioBtn.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	// GPIO output to on-board LED
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GpioBtn);
	GPIO_Init(&GpioLed);
}

void USART1_IRQHandler()
{
	USART_EV_IRQHandling(&Usart1_Handle);
}


int main(void)
{
	// Enable semi-hosting
	initialise_monitor_handles();

	char msg[3][100] = {"Eyy you need some milk?\n", "Milk is good with bread\n", "Put PB&J on that break tho\n"};
	char response[100];
	uint8_t count = 0;

	// 1. Set GPIO on-board button and LED GPIO pins for feedback
	GPIO_OnBoardLedBtn_Set();

	// 2. Set GPIOA pins PA9 - PA12 in AF7 for USART1 function
	GPIO_USART_Config();

	// 3. Configure USART1 in Tx mode, no parity bit, 8 bit data, over sampling with 8
	USART1_Config(&Usart1_Handle);

	// 4. Set up interrupts in NVIC for USART1
	USART_IRQConfig(IRQ_POS_USART1, ENABLE);

	while(1)
	{
		/* Count to cycle through messages in msg array */
		count = count % 3;

		// 4. Wait till button press, turn on LED for visual feedback
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		delay(200);

		// 6. Enable read to read back identical message when Arduino sends
		while(USART_Read_IT(&Usart1_Handle, (uint8_t*)response, strlen(msg[count])) != USART_RDY);

		// 5. Send data via USART interrupt
		while(USART_Write_IT(&Usart1_Handle, (uint8_t*)msg[count], strlen(msg[count])) != USART_RDY);

		// 8. Wait till callback notifies that Rx is done
		while(status != SET);
		status = RESET;
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);

		response[strlen(msg[count])] = '\0';
		printf("Response is: %s\n", response);
		count++;
	}

	return 0;
}

/*
 * Call back function from interrupt events
 */
void USART_ApplicationCallbackEvent(USART_Handle_t *pUSARTxHandle, uint8_t event)
{
	if (event == USART_RX_CMPLT)
	{
		status = SET;
	}
}

