/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Apr. 20, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"


/********************* CLOCK RATE GET APIS *********************/

/*
 * Get clock speeds
 */
uint32_t RCC_GetPPLCLK(); //TODO

/*
 * Get APB1 clock rate
 */
uint32_t RCC_GetAPB1CLK();

/*
 * Get APB2 clock rate
 */
uint32_t RCC_GetAPB2CLK();




#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
