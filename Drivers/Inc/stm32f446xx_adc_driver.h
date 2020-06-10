/*
 * stm32f446xx_adc_driver.h
 *
 *  Created on: Jun. 10, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_ADC_DRIVER_H_
#define INC_STM32F446XX_ADC_DRIVER_H_

#include "stm32f446xx.h"

/*
 * ADC Configuration Structure
 */
typedef struct {
	uint8_t ADC_Res;
	uint8_t ADC_PreSc;
	uint8_t ADC_Ext_Trig;
} ADC_Config_t;

/*
 * ADC Handle Structure
 */
typedef struct {
	ADC_RegDef_t *pADCx;
	ADC_Comm_RegDef_t *pAdcComm;
	ADC_Config_t ADC_Config;
} ADC_Handle_t;


/********************* ADC RESET MACROS *********************/

#define ADC_RESET()		do { (RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR |= ~(1 << 8)); } while(0)


/********************* DRIVER APIS *********************/

/*
 * Peripheral clock setup
 */
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnOrDi);

/*
 * Initialize and close
 */
void ADC_Init(ADC_Handle_t *pADCxHandle);
void ADC_DeInit(ADC_RegDef_t *pADCx);

/*
 * Read from ADC
 */
void ADC_SingleRead(ADC_Handle_t *pADCxHandle, uint8_t adcChn, uint8_t sampleCycles);

/*
 * ADC Interrupt Handling
 */
void ADC_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void ADC_IRQHandling(uint8_t PinNumber);


/************************* CONFIGURATION VALUES *****************************/

/*
 * Single read or scan mode
 */
#define ADC_MODE_SCAN			0
#define ADC_MODE_SINGLE			1

/*
 * Read once or continuous
 */
#define ADC_SINGLE_READ			0
#define ADC_CONT_READ			1

/*
 * ADC clock prescaler
 */
#define ADC_PCLK_DIV2			0
#define ADC_PCLK_DIV4			1
#define ADC_PCLK_DIV6			2
#define ADC_PCLK_DIV8			3

/*
 * ADC resolution
 */
#define ADC_RES_12BIT			0
#define ADC_RES_10BIT			1
#define ADC_RES_8BIT			2
#define ADC_RES_6BIT			3

/*
 * External trigger
 */
#define ADC_EXTEN_DI			0
#define ADC_EXTEN_RT			1
#define ADC_EXTEN_FT			2
#define ADC_EXTEN_RFT			3

/*
 * Channels from ADC multiplexer
 */
#define ADC_IN0					0
#define ADC_IN1					1
#define ADC_IN2					2
#define ADC_IN3					3
#define ADC_IN4					4
#define ADC_IN5					5
#define ADC_IN6					6
#define ADC_IN7					7
#define ADC_IN8					8
#define ADC_IN9					9
#define ADC_IN10				10
#define ADC_IN11				11
#define ADC_IN12				12
#define ADC_IN13				13
#define ADC_IN14				14
#define ADC_IN15				15

/*
 * Cycles for reading
 */
#define ADC_SMP_3CYC			0
#define ADC_SMP_15CYC			1
#define ADC_SMP_28CYC			2
#define ADC_SMP_56CYC			3
#define ADC_SMP_84CYC			4
#define ADC_SMP_112CYC			5
#define ADC_SMP_144CYC			6
#define ADC_SMP_480CYC			7



#endif
