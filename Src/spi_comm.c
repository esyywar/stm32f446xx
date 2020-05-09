/*
 * spi_comm.c
 *
 *  Created on: Mar. 29, 2020
 *      Author: Rahul
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

void GPIO_Set_SPI2()
{
	GPIO_Handle_t SPI2_GPIO_Set;

	// Configure the GPIO Pins to SPI mode
	SPI2_GPIO_Set.pGPIOx = GPIOB;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_AfMode = GPIO_AF_5;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_PUPL;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_PD;

	// Set PB13 to SCLK
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&SPI2_GPIO_Set);

	// Set PB14 to MOSI
	SPI2_GPIO_Set.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_15;
	GPIO_Init(&SPI2_GPIO_Set);
}

void SPI2_Setup()
{
	SPI_Handle_t SPI2_Config;

	SPI2_Config.pSPIx = SPI2;
	SPI2_Config.SPI_Config.SPI_DeviceMode = SPI_MSTR_SET;
	SPI2_Config.SPI_Config.SPI_BusConfig = SPI_SMPLX_TX;
	SPI2_Config.SPI_Config.SPI_SclkSpeed = SPI_CLK_PCLK_DIV2;
	SPI2_Config.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Config.SPI_Config.SPI_CPHA = SPI_CPHA_1E;
	SPI2_Config.SPI_Config.SPI_DFF = SPI_8BIT_DFF;
	SPI2_Config.SPI_Config.SPI_SSM = SPI_SSM_SW;

	// Initialize the SPI2 peripheral
	SPI_Init(&SPI2_Config);

}

int main(void)
{
	// Set up GPIO Pins as SPI2 (PB13 -> SCLK, PB15 -> MOSI)
	GPIO_Set_SPI2();

	// Set up SPI2 as simplex Tx only
	SPI2_Setup();

	char userData[] = "Hello World";

	// Set SSI high to avoid MODF flag in single master mode
	SPI_SSI_Control(SPI2, HIGH);

	// Enable SPI communication
	SPI_PeripheralControl(SPI2, ENABLE);

	// Write data out
	SPI_Write(SPI2, (uint8_t*)userData, strlen(userData));

	// Wait till SPI BSY flag resets
	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY));

	// Close SPI communication
	SPI_PeripheralControl(SPI2, DISABLE);

	return 0;
}



