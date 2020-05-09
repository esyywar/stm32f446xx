/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Apr. 20, 2020
 *      Author: Rahul
 */


#include "stm32f446xx_rcc_driver.h"


/********************** CLOCK RATE CONVERSION FOR APB1, APB2 ************************************/

static uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint8_t PPRE1_Prescaler[4] = {2, 4, 8, 16};
static uint8_t PPRE2_Prescaler[4] = {2, 4, 8, 16};

/*********************************************** DRIVER API IMPLMENTATION ******************************************/


/*
 * Function:	RCC_GetPPLCLK
 *
 * Brief: 		Get the clock speed to I2C peripherals if PPL is being used
 *
 * Return: 		uint32_t PPLClk - accelerated clock speed (Hz)
 *
 */
uint32_t RCC_GetPPLCLK(void)
{
	//TODO
	return 0;
}


/*
 * Function:	RCC_GetAPB1CLK
 *
 * Brief: 		Get the clock speed to I2C peripherals
 *
 * Return: 		uint32_t Apb1Clk - clock speed to I2C peripherals (Hz)
 *
 */
uint32_t RCC_GetAPB1CLK(void)
{
	// 1. Get system clock source (internal, external, PLL)
	uint8_t clkSrc = ((RCC->CFGR >> 2) & 0x3);

	uint32_t sysClk, Apb1Clk;

	if (clkSrc == 0)
	{
		// Internal oscillator
		sysClk = 16000000;
	}
	else if (clkSrc == 1)
	{
		// External on-board oscillator
		sysClk = 8000000;
	}
	else if (clkSrc == 2)
	{
		Apb1Clk = RCC_GetPPLCLK();
		return Apb1Clk;
	}

	// 2. Get the AHB prescaler value
	uint8_t temp = (RCC->CFGR >> 4) & 0xF;
	uint8_t ahbPre = 1, ppre1Pre = 1;

	if (temp >= 8)
	{
		ahbPre = AHB_Prescaler[temp - 8];
	}

	// 3. Get the APB1 prescaler value
	temp = (RCC->CFGR >> 10) & 0x7;
	if (temp >= 4)
	{
		ppre1Pre = PPRE1_Prescaler[temp - 4];
	}

	// 4. Calculate the clock speed to I2C
	Apb1Clk = (sysClk / ahbPre) / ppre1Pre;

	return Apb1Clk;
}


/*
 * Function:	RCC_GetAPB2CLK
 *
 * Brief: 		Get the clock speed to APB2 bus
 *
 * Return: 		uint32_t Apb2Clk - clock speed to APB2 (Hz)
 *
 */
uint32_t RCC_GetAPB2CLK(void)
{
	// 1. Get system clock source (internal, external, PLL)
	uint8_t clkSrc = ((RCC->CFGR >> 2) & 0x3);

	uint32_t sysClk, Apb2Clk;

	if (clkSrc == 0)
	{
		// Internal oscillator
		sysClk = 16000000;
	}
	else if (clkSrc == 1)
	{
		// External on-board oscillator
		sysClk = 8000000;
	}
	else if (clkSrc == 2)
	{
		Apb2Clk = RCC_GetPPLCLK();
		return Apb2Clk;
	}

	// 2. Get the AHB prescaler value
	uint8_t temp = (RCC->CFGR >> 4) & 0xF;
	uint8_t ahbPre = 1, ppre2Pre = 1;

	if (temp >= 8)
	{
		ahbPre = AHB_Prescaler[temp - 8];
	}

	// 3. Get the APB2 prescaler value
	temp = (RCC->CFGR >> 13) & 0x7;
	if (temp >= 4)
	{
		ppre2Pre = PPRE2_Prescaler[temp - 4];
	}

	// 4. Calculate the clock speed to I2C
	Apb2Clk = (sysClk / ahbPre) / ppre2Pre;

	return Apb2Clk;
}


