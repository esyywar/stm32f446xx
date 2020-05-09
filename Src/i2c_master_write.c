/*
 * i2c_master_write.c
 *
 *  Created on: Apr. 7, 2020
 *      Author: Rahul
 */

#include "stm32f446xx.h"

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_i2c_driver.h"

#define NUCLEO_ADDR			0x61
#define ARDUINO_ADDR		0x68


void delay(uint32_t multiplier)
{
	uint32_t timer = 0;

	// 16 MHz  internal CLK so multiplier = 1 gives 1 ms delay (4 clock cycles per computation)
	while (timer < (multiplier * 1000))
	{
		timer++;
	}
}

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

void GPIO_I2C1_Config()
{
	GPIO_Handle_t GpioI2C1_ConfigPin;
	GPIOB_PCLK_EN();

	GpioI2C1_ConfigPin.pGPIOx = GPIOB;
	GpioI2C1_ConfigPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_8;
	GpioI2C1_ConfigPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	GpioI2C1_ConfigPin.GPIO_PinConfig.GPIO_OpType = GPIO_OTYPE_OD;
	GpioI2C1_ConfigPin.GPIO_PinConfig.GPIO_AfMode = GPIO_AF_4;
	GpioI2C1_ConfigPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	GpioI2C1_ConfigPin.GPIO_PinConfig.GPIO_PuPdCtrl = GPIO_PUPD_PU;

	// Set PB8 as SCL
	GPIO_Init(&GpioI2C1_ConfigPin);

	// Set PB9 as SDA
	GpioI2C1_ConfigPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_9;
	GPIO_Init(&GpioI2C1_ConfigPin);
}

void I2C1_MasterConfig(I2C_Handle_t* I2CMaster)
{
	I2C1_PCLK_EN();

	I2CMaster->pI2Cx = I2C1;
	I2CMaster->I2C_Config_t.I2C_ACKCtrl = ENABLE;
	I2CMaster->I2C_Config_t.I2C_OwnDeviceAddr = NUCLEO_ADDR;
	I2CMaster->I2C_Config_t.I2C_AddrBits = I2C_7BIT_ADDR;
	I2CMaster->I2C_Config_t.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(I2CMaster);
}

int main(void)
{
	// 1. Initialize on-board button and LED for feedback
	GPIO_OnBoardLedBtn_Set();

	// 2. Set PB8 and PB9 as I2C1 SCL and SDA respectively
	GPIO_I2C1_Config();

	// 3. Set up I2C1 to operate as master, 7-bit data in SM mode
	I2C_Handle_t I2CMasterSend;
	I2C1_MasterConfig(&I2CMasterSend);

	// 4. Prepare data to write
	uint8_t arduinoAddr = ARDUINO_ADDR;
	char message[] = "Pls buy a cabbage";

	while (1)
	{
		// 5. Wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));
		delay(200);

		// 6. Turn on LED and send data to slave
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		I2C_7BitAddr_MasterSend(&I2CMasterSend, (uint8_t*)message, strlen(message), arduinoAddr, NO_RPT_START);
	}

	return 0;
}




