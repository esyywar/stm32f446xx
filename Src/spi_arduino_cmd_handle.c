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

	// Set PC12 to NSS
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
void SPI2_Setup()
{
	SPI_Handle_t SPI2_Config;

	SPI2_Config.pSPIx = SPI2;
	SPI2_Config.SPI_Config.SPI_DeviceMode = SPI_MSTR_SET;
	SPI2_Config.SPI_Config.SPI_BusConfig = SPI_FULL_DPLX;
	SPI2_Config.SPI_Config.SPI_SclkSpeed = SPI_CLK_PCLK_DIV8;
	SPI2_Config.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Config.SPI_Config.SPI_CPHA = SPI_CPHA_1E;
	SPI2_Config.SPI_Config.SPI_DFF = SPI_8BIT_DFF;
	SPI2_Config.SPI_Config.SPI_SSM = SPI_SSM_HW;

	// Initialize the SPI2 peripheral
	SPI_Init(&SPI2_Config);

	// Allow CS to be toggled by SPI NSS
	SPI_SSOE_Control(SPI2, ENABLE);
}


// Send LED control data to arduino
void LED_Toggle_Cmd(uint8_t arduinoPin, uint8_t EnOrDi)
{
	uint8_t args[] = {arduinoPin, EnOrDi};


	SPI_Write(SPI2, args, 2);
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


int main(void)
{
	uint8_t dummyTx, dummyRx, command, response;
	dummyTx = 0xFF;

	// Set connected GPIO to on-board button to input
	GPIO_Btn_Set();

	// Configure GPIO pins for SPI2 function
	GPIO_Set_SPI2();

	// Configure settings for full duplex communication on SPI2
	SPI2_Setup();

	// Enable SPI communication
	SPI_PeripheralControl(SPI2, ENABLE);

	/**************** SEND LED ON COMMAND ****************/

	// Wait for button to be pressed and go low
	while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));
	delay(200);

	command = COMMAND_LED_CTRL;
	SPI_Write(SPI2, &command, 1);
	SPI_Read(SPI2, &dummyRx, 1);

	while(SPI_StatusFlagCheck(SPI2, SPI_FLAG_BSY));

	SPI_Write(SPI2, &dummyTx, 1);
	SPI_Read(SPI2, &response, 1);

	if (Verify_Response(response))
	{
		while(SPI_StatusFlagCheck(SPI2, SPI_FLAG_BSY));

		LED_Toggle_Cmd(ARDUINO_LED_PIN, LED_ON);
	}


	/**************** SEND ANALOG READ COMMAND ****************/

	while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));
	delay(200);

	command = COMMAND_SENSOR_READ;
	SPI_Write(SPI2, &command, 1);
	SPI_Read(SPI2, &dummyRx, 1);

	while(SPI_StatusFlagCheck(SPI2, SPI_FLAG_BSY));

	SPI_Write(SPI2, &dummyTx, 1);
	SPI_Read(SPI2, &response, 1);

	if (Verify_Response(response))
	{
		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

		command = ARDUINO_ALG_PIN;
		SPI_Write(SPI2, &command, 1);
		SPI_Read(SPI2, &dummyRx, 1);

		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));
		delay(50);

		SPI_Write(SPI2, &dummyTx, 1);
		SPI_Read(SPI2, &response, 1);

		uint8_t analogRead = response;
		(void)analogRead;
	}


	/**************** SEND DIO READ COMMAND ****************/

	while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));
	delay(200);

	command = COMMAND_LED_READ;
	SPI_Write(SPI2, &command, 1);
	SPI_Read(SPI2, &dummyRx, 1);

	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

	SPI_Write(SPI2, &dummyTx, 1);
	SPI_Read(SPI2, &response, 1);

	if (Verify_Response(response))
	{
		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

		command = ARDUINO_LED_PIN;
		SPI_Write(SPI2, &command, 1);
		SPI_Read(SPI2, &dummyRx, 1);

		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));
		delay(50);

		SPI_Write(SPI2, &dummyTx, 1);
		SPI_Read(SPI2, &response, 1);

		uint8_t analogRead = response;
		(void)analogRead;
	}


	/**************** SEND PRINT COMMAND ****************/

	while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));
	delay(200);

	command = COMMAND_PRINT;
	SPI_Write(SPI2, &command, 1);
	SPI_Read(SPI2, &dummyRx, 1);

	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

	SPI_Write(SPI2, &dummyTx, 1);
	SPI_Read(SPI2, &response, 1);

	if (Verify_Response(response))
	{
		char message[] = "Lets get it";

		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

		command = strlen(message);
		SPI_Write(SPI2, &command, 1);

		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

		SPI_Write(SPI2, (uint8_t*)message, strlen(message));
	}


	/**************** SEND BOARD ID READ COMMAND ****************/

	while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));
	delay(200);

	command = COMMAND_ID_READ;
	SPI_Write(SPI2, &command, 1);
	SPI_Read(SPI2, &dummyRx, 1);

	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

	SPI_Write(SPI2, &dummyTx, 1);
	SPI_Read(SPI2, &response, 1);

	if (Verify_Response(response))
	{
		uint8_t boardID[10];

		for (int i = 0; i < 10; i++)
		{
			SPI_Write(SPI2, &dummyTx, 1);
			SPI_Read(SPI2, &boardID[i], 1);
			printf("%i\n", boardID[i]);
		}

		boardID[10] = '\0';
	}

	// Close SPI communication
	SPI_PeripheralControl(SPI2, DISABLE);

	return 0;
}
