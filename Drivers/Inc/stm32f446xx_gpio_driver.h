/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Mar. 22, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"


/*
 * GPIO pin configuration structure
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PuPdCtrl;
	uint8_t GPIO_OpType;
	uint8_t GPIO_AfMode;
} GPIO_PinConfig_t;

/*
 * GPIO pin handle structure
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;


/************* GPIO RESET MACROS **********/

#define GPIOA_RESET()		do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_RESET()		do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_RESET()		do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_RESET()		do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_RESET()		do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_RESET()		do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_RESET()		do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_RESET()		do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)

/************************* HELPER FUNCTIONS *****************************/

uint8_t GPIO_PortCode(GPIO_RegDef_t *pGPIOx);


/************************* DRIVER APIS *****************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Initialize and close
 */
void GPIO_Init(GPIO_Handle_t *pGPIOxHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Input and output operations
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Interrupt handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


/************************* CONFIGURATION VALUES *****************************/

/*
 * GPIO pin numbers
 */
#define GPIO_PIN_NUM_0			0
#define GPIO_PIN_NUM_1			1
#define GPIO_PIN_NUM_2			2
#define GPIO_PIN_NUM_3			3
#define GPIO_PIN_NUM_4			4
#define GPIO_PIN_NUM_5			5
#define GPIO_PIN_NUM_6			6
#define GPIO_PIN_NUM_7			7
#define GPIO_PIN_NUM_8			8
#define GPIO_PIN_NUM_9			9
#define GPIO_PIN_NUM_10			10
#define GPIO_PIN_NUM_11			11
#define GPIO_PIN_NUM_12			12
#define GPIO_PIN_NUM_13			13
#define GPIO_PIN_NUM_14			14
#define GPIO_PIN_NUM_15			15

/*
 * GPIO pin modes
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_AF			2
#define GPIO_MODE_ANALOG		3
#define GPIO_IT_MODE_RT			4
#define GPIO_IT_MODE_FT			5
#define GPIO_IT_MODE_RFT		6

/*
 * GPIO output types
 */
#define GPIO_OTYPE_PUPL			0
#define GPIO_OTYPE_OD			1

/*
 * GPIO output modes
 */
#define GPIO_PIN_SET			0
#define GPIO_PIN_RESET			1
#define GPIO_PORT_SET			0
#define GPIO_PORT_RESET			1

/*
 * GPIO output speeds
 */
#define GPIO_LOW_SPEED			0
#define GPIO_MED_SPEED			1
#define GPIO_FAST_SPEED			2
#define GPIO_HIGH_SPEED			3

/*
 * GPIO pull-up, pull-down options
 */
#define GPIO_PUPD_NONE			0
#define GPIO_PUPD_PU			1
#define GPIO_PUPD_PD			2

/*
 * GPIO alternate function modes
 */
#define GPIO_AF_0				0
#define GPIO_AF_1				1
#define GPIO_AF_2				2
#define GPIO_AF_3				3
#define GPIO_AF_4				4
#define GPIO_AF_5				5
#define GPIO_AF_6				6
#define GPIO_AF_7				7
#define GPIO_AF_8				8
#define GPIO_AF_9				9
#define GPIO_AF_10				10
#define GPIO_AF_11				11
#define GPIO_AF_12				12
#define GPIO_AF_13				13
#define GPIO_AF_14				14
#define GPIO_AF_15				15


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
