/*
 * stm32f446xx_systck.driver.h
 *
 *  Created on: May 10, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_SYSTCK_DRIVER_H_
#define INC_STM32F446XX_SYSTCK_DRIVER_H_

/*
 * SYSTCK Configuration structure
 */
typedef struct {
	uint8_t SYSTCK_ClkSource;
	uint8_t SYSTCK_IRQ;
	uint32_t SYSTCK_Reload;
} SYSTCK_Config_t;


/************************ DRIVER APIS ************************/

/*
 * Enable and configure the system timer - return 1 to indicate success
 */
uint8_t SYSTCK_Config(SYSTCK_Config_t* pSYSTCK);

/*
 * Read current count value from system timer
 */
uint32_t SYSTCK_ReadCurrent();

/*
 * Read count flag to see if timer has reloaded
 */
uint8_t SYSTCK_CountFlag();


/************************* CONFIGURATION VALUES *****************************/

#define SYSTCK_MAX_RELOAD 		0x00FFFFFFUL

/*
 * Clock source
 */
#define SYSTCK_EXT_CLK			1
#define SYSTCK_INT_CLK			0

/*
 * Interrupt enabled?
 */
#define SYSTCK_IRQ_EN		1
#define SYSTCK_IRQ_DI		0


#endif /* INC_STM32F446XX_SYSTCK_DRIVER_H_ */
