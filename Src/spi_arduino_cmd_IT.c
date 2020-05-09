/*
 * spi_arduino_cmd_handle.c
 *
 *  Created on: Mar. 30, 2020
 *      Author: Rahul
 */

/*
 * This program passes commands to Arduino Nano and reads the responses
 *
 * Arduino Nano DIO pin 9 is toggled for turn on/off LED
 *
 * Data transmissions are initiated by on-board button press on Nucleo
 *
 */

#include <stdio.h>
#include <string.h>

#include "stm32f446xx.h"

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"


extern void initialise_monitor_handles();


// Arduino LED info
#define ARDUINO_ALG_PIN			2
#define ARDUINO_LED_PIN			9
#define LED_ON					1
#define LED_OFF					0

// Command codes
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54
#define NACK 					0xA5
#define ACK 					0xF5

// Arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4


// SPI2 handle
SPI_Handle_t SPI2_Handle;


void delay(uint32_t multiplier)
{
	uint32_t timer = 0;

	// 16 MHz  internal CLK so multiplier = 1 gives 1 ms delay (4 clock cycles per computation)
	while (timer < (multiplier * 1000))
	{
		timer++;
	}
}


// Set PC13 connected to on-board LED as input
void GPIO_Btn_Set()
{
	GPIO_Handle_t GPIO_Btn;

	// Configure the GPIO PC13 for on-board button
	GPIO_Btn.pGPIOx = GPIOC;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GPIO_Btn.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GPIO_Btn);
}

// Configure required GPIO pins for SPI communication
void GPIO_Set_SPI2()
{
	GPIO_Handle_t SPI2_GPIO_Set;

	// Configure the GPIO Pins to SPI mode
	SPI2_GPIO_Set.pGPIOx = GPIOB;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_AfMode = GPIO_AF_5;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_PU;

	// Set PB12 to NSS
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GPIO_Init(&SPI2_GPIO_Set);

	// Set PB13 to SCLK
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&SPI2_GPIO_Set);

	// Set PB14 to MISO
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_14;
	GPIO_Init(&SPI2_GPIO_Set);

	// Set PB15 to MOSI
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_15;
	GPIO_Init(&SPI2_GPIO_Set);
}


// Configure SPI2 peripheral bus for full duplex communication
void SPI2_Setup(SPI_Handle_t *SPI2_Handle)
{
	SPI2_Handle->pSPIx = SPI2;
	SPI2_Handle->SPI_Config.SPI_DeviceMode = SPI_MSTR_SET;
	SPI2_Handle->SPI_Config.SPI_BusConfig = SPI_FULL_DPLX;
	SPI2_Handle->SPI_Config.SPI_SclkSpeed = SPI_CLK_PCLK_DIV8;
	SPI2_Handle->SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle->SPI_Config.SPI_CPHA = SPI_CPHA_1E;
	SPI2_Handle->SPI_Config.SPI_DFF = SPI_8BIT_DFF;
	SPI2_Handle->SPI_Config.SPI_SSM = SPI_SSM_HW;

	// Initialize the SPI2 peripheral
	SPI_Init(SPI2_Handle);

	// Allow CS to be toggled by SPI NSS
	SPI_SSOE_Control(SPI2, ENABLE);
}


// Verify response from arduino
uint8_t Verify_Response(uint8_t response)
{
	if (response == ACK)
	{
		return 1;
	}

	return 0;
}


// Implementing SPI2 IRQ function call
void SPI2_IRQHandler()
{
	SPI_IRQHandling(&SPI2_Handle);
}


int main(void)
{
	uint8_t dummyTx, dummyRx, command, response;
	dummyTx = 0xFF;

	// Set connected GPIO to on-board button to input
	GPIO_Btn_Set();

	// Configure GPIO pins for SPI2 function
	GPIO_Set_SPI2();

	// Configure settings for full duplex communication on SPI2
	SPI2_Setup(&SPI2_Handle);

	// Configure SPI2 interrupt at NVIC
	SPI_IRQConfig(IRQ_POS_SPI2, ENABLE);

	// Enable SPI communication
	SPI_PeripheralControl(SPI2, ENABLE);


	/**************** SEND LED ON COMMAND ****************/

	// Wait for button to be pressed and go low
	while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));
	delay(150);

	command = COMMAND_LED_CTRL;

	while(SPI_Write_IT(&SPI2_Handle, &command, 1) != SPI_RDY);
	while(SPI_Read_IT(&SPI2_Handle, &dummyRx, 1) != SPI_RDY);

	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

	while(SPI_Write_IT(&SPI2_Handle, &dummyTx, 1) != SPI_RDY);
	while(SPI_Read_IT(&SPI2_Handle, &response, 1) != SPI_RDY);

	if (Verify_Response(response))
	{
		uint8_t args[] = {ARDUINO_LED_PIN, LED_ON};

		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

		while(SPI_Write_IT(&SPI2_Handle, args, 2) != SPI_RDY);
	}

	// Close SPI communication
	SPI_PeripheralControl(SPI2, DISABLE);

	return 0;
}
