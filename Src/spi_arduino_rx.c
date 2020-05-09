/*
 * spi_comm.c
 *
 *  Created on: Mar. 30, 2020
 *      Author: Rahul
 */

/*
 * This program initiates SPI communication with Arduino
 * The Arduino will Rx only and ARM will Tx only -> simplex mode
 *
 * Data transmission is triggered by on-board button press (connected to PC13)
 *
 */

#include <string.h>

#include "stm32f446xx.h"

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"


void delay(uint32_t multiplier)
{
	uint32_t timer = 0;

	// 16 MHz  internal CLK so multiplier = 1 gives 1 ms delay (4 clock cycles per computation)
	while (timer < (multiplier * 1000))
	{
		timer++;
	}
}

// Configure on-board button for IRQ (at PC13)
void GPIO_BtnIRQ_Set()
{
	GPIO_Handle_t GPIO_BtnIRQ;

	GPIO_BtnIRQ.pGPIOx = GPIOC;
	GPIO_BtnIRQ.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_BtnIRQ.GPIO_PinConfig.GPIO_PinMode = GPIO_IT_MODE_FT;
	GPIO_BtnIRQ.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GPIO_BtnIRQ.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_NONE;

	GPIO_Init(&GPIO_BtnIRQ);

	GPIO_IRQConfig(IRQ_POS_EXTI_15_10, ENABLE);
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

	// Set PB15 to MOSI
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_15;
	GPIO_Init(&SPI2_GPIO_Set);
}

// Cofigure SPI2 peripheral bus for simplex Tx communication
void SPI2_Setup()
{
	SPI_Handle_t SPI2_Config;

	SPI2_Config.pSPIx = SPI2;
	SPI2_Config.SPI_Config.SPI_DeviceMode = SPI_MSTR_SET;
	SPI2_Config.SPI_Config.SPI_BusConfig = SPI_SMPLX_TX;
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

// Enable SPI, send data, disable SPI
void SPI_SendData(uint8_t* data, uint32_t len)
{
	// Enable SPI communication
	SPI_PeripheralControl(SPI2, ENABLE);

	// First tell Arduino how long data is
	uint8_t dataLen = (uint8_t)len;
	SPI_Write(SPI2, &dataLen, 1);

	// Write data out
	SPI_Write(SPI2, data, len);

	// Wait till SPI BSY flag resets
	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

	// Close SPI communication
	SPI_PeripheralControl(SPI2, DISABLE);
}

// IRQ handler - called on button press
void EXTI15_10_IRQHandler() {
	delay(150);
	GPIO_IRQHandling(GPIO_PIN_NUM_13);

	// Send data
	char userData[] = "Ola Amigo";
	SPI_SendData((uint8_t*)userData, strlen(userData));
}

int main(void)
{
	// Set up PC13 interrupt to trigger when on-board button pressed
	GPIO_BtnIRQ_Set();

	// Set up GPIO Pins as SPI2 (PB13 -> SCLK, PB15 -> MOSI)
	GPIO_Set_SPI2();

	// Set up SPI2 as simplex Tx only
	SPI2_Setup();

	while(1)
	{

	}

	return 0;
}




