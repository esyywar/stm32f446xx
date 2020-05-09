/*
 * i2c_master_read_cmd.c
 *
 *  Created on: Apr. 11, 2020
 *      Author: Rahul
 *
 *      Reads Arduino Nano over I2C
 *      Receives back a 23 character string from Arduino while in slave mode
 *      Uses interrupts for I2C comm.
 *
 */

#include "stm32f446xx.h"

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_i2c_driver.h"

#define NUCLEO_ADDR			0x68

#define SEND_LENGTH_CMD		0x51
#define SEND_DATA_CMD		0x52

#define IDLE				0
#define DATA_PENDING		1
#define DATA_RECEIVED		2
#define SEND_PENDING		3
#define SEND_COMPLETE		4


I2C_Handle_t I2CSlaveRx;

uint8_t txBuffer[] = "Talking about Chief Keef ain't";


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
 * Configure PB8 as I2C1 SCL and PB9 I2C1 SDA
 */
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

/*
 * Configure I2C1 bus
 */
void I2C1_SlaveConfig(I2C_Handle_t* I2CMaster)
{
	I2C1_PCLK_EN();

	I2CMaster->pI2Cx = I2C1;
	I2CMaster->I2C_Config_t.I2C_ACKCtrl = ENABLE;
	I2CMaster->I2C_Config_t.I2C_OwnDeviceAddr = NUCLEO_ADDR;
	I2CMaster->I2C_Config_t.I2C_AddrBits = I2C_7BIT_ADDR;
	I2CMaster->I2C_Config_t.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(I2CMaster);
}

/*
 * Handle I2C1 events
 */
void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&I2CSlaveRx);
}

/*
 * Handle I2C1 errors
 */
void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2CSlaveRx);
}


/*
 * Main function
 *
 * 1. Nucleo board set in slave mode
 * 2. Command is sent from arduino which prompts nucleo to send data
 *
 */
int main(void)
{
	// 1. Set PB8 and PB9 as I2C1 SCL and SDA respectively
	GPIO_I2C1_Config();

	// 2. Set up I2C1 to operate as slave, 7-bit data in SM mode
	I2C1_SlaveConfig(&I2CSlaveRx);

	// 3. Enable peripheral
	I2C_PeripheralControl(I2CSlaveRx.pI2Cx, ENABLE);

	// 4. Enable ACK
	I2C_AckControl(I2CSlaveRx.pI2Cx, ENABLE);

	// 5. Enable slave callback events
	I2C_SlaveEnableDisableCallbackEvents(I2CSlaveRx.pI2Cx, ENABLE);

	// 6. Configure interrupt in NVIC
	I2C_IRQConfig(IRQ_POS_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_POS_I2C1_ERR, ENABLE);


	while (1)
	{
		// Run and react to programmed interrupt events in callback function
	}

	return 0;
}

/*
 * Application call back managing global state variable
 */
void I2C_ApplicationCallbackEvent(I2C_Handle_t *pI2CxHandle, uint8_t event)
{
	static uint8_t commandCode = 0;
	static uint8_t TxCount = 0;

	if (event == I2C_EV_REQ_DATA)
	{
		if (commandCode == SEND_LENGTH_CMD)
		{
			I2C_SlaveSend(pI2CxHandle->pI2Cx, sizeof(txBuffer) / sizeof(txBuffer[0]));
		}
		else if (commandCode == SEND_DATA_CMD)
		{
			I2C_SlaveSend(pI2CxHandle->pI2Cx, txBuffer[TxCount]);
			TxCount++;
		}
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
	}
	else if (event == I2C_EV_REC_DATA)
	{
		commandCode = I2C_SlaveRead(pI2CxHandle->pI2Cx);
	}
	else if (event == I2C_AF_ERROR)
	{
		// Master done receiving data
		commandCode = 0xFF;
		TxCount = 0;
	}
	else if (event == I2C_EV_STOP)
	{
		// Comm. with slave ended
	}
}

