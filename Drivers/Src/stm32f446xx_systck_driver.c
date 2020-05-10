/*
 * stm32f446xx_systck_driver.c
 *
 *  Created on: May 10, 2020
 *      Author: Rahul
 */

#include "stm32f446xx_systck_driver.h"


/*************************************************** DRIVER API IMPLMENTATION ***********************************************/

/*
 * Function:	SYSTCK_Config
 *
 * Brief: 		Initialize the system timer
 *
 * Params: 		struct SYSTCK_Config_t* pSYSTCK - configuration values for system timer
 *
 */
uint8_t SYSTCK_Init(SYSTCK_Config_t* pSYSTCK)
{
	if (pSYSTCK->SYSTCK_Reload > SYSTCK_MAX_RELOAD)
	{
		return RESET;
	}

	// Load reload value while keeping high 8 reserved bits
	NVIC_SYSTCK->RVR |= (pSYSTCK->SYSTCK_Reload - 1);

	// Clear current value with write operation
	NVIC_SYSTCK->CVR = 0;

	// Write to CSR according to configured values
	uint8_t temp = 1U;
	if (pSYSTCK->SYSTCK_ClkSource == SYSTCK_INT_CLK) { temp |= (1 << 2); }
	if (pSYSTCK->SYSTCK_IRQ == SYSTCK_IRQ_EN) { temp |= (1 << 1); }
	NVIC_SYSTCK->CSR |= temp;

	return SET;
}


/*
 * Function:	SYSTCK_ReadValue
 *
 * Brief: 		Read current count from the system timer
 *
 * Retuen:		uint32_t - current value in system timer
 *
 */
uint32_t SYSTCK_ReadValue()
{
	return (NVIC_SYSTCK->CVR & 0x00FFFFFF);
}


/*
 * Function:	SYSTCK_CountFlag
 *
 * Brief: 		Checks is count flag is raised indicating the timer has hit 0
 * 				The count flag is cleared by the read operation
 *
 * Params: 		struct I2C_RegDef_t* pI2Cx- I2C base address
 * 				uint8_t EnOrDi - Enable or disable value
 *
 */
uint8_t SYSTCK_CountFlag()
{
	if (NVIC_SYSTCK->CSR & (1 << 16))
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}

