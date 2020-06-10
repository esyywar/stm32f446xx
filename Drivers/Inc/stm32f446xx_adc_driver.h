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
	ADC_Config_t ADC_Config;
	uint16_t *pDataBuffer;
} ADC_Handle_t;


/********************* ADC RESET MACROS *********************/

#define ADC_RESET()		do { (RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR |= ~(1 << 8)); } while(0)


/********************* DRIVER APIS *********************/

/*
 * Peripheral clock setup
 */
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnOrDi);

/*
 * Initialize and reset
 */
void ADC_Init(ADC_Handle_t *pADCxHandle);
void ADC_DeInit();

/*
 * Read from ADC
 */
void ADC_Read_Channel(ADC_Handle_t *pADCxHandle, uint8_t ADC_CHAN, uint8_t ADC_SMP_CYC, uint8_t ADC_DAQ_MODE);

/*
 * ADC Interrupt Configuration
 */
void ADC_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Read flag status from the SR
 */
uint8_t ADC_GetFlagStatus(ADC_RegDef_t *pADCx, uint8_t ADC_FLAG);

/*
 * Event and error interrupt handlers
 */
void ADC_EV_IRQHandling(ADC_Handle_t *pADCxHandle);
void ADC_ER_IRQHandling(ADC_Handle_t *pADCxHandle);

/*
 * Application callback used to notify of interrupt events and errors
 */
__weak void ADC_ApplicationCallbackEvent(ADC_Handle_t *pADCxHandle, uint8_t event);


/************************* CONFIGURATION VALUES *****************************/

/*
 * ADC acquisition mode
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


/******************* STATUS REGISTER FLAGS *****************/

#define ADC_FLAG_AWD			(1 << 0)
#define ADC_FLAG_EOC			(1 << 1)
#define ADC_FLAG_JEOC     		(1 << 2)
#define ADC_FLAG_JSTRT     		(1 << 3)
#define ADC_FLAG_STRT     		(1 << 4)
#define ADC_FLAG_OVR       		(1 << 5)


/******************* ADC CALLBACK EVENTS ********************/

#define ADC_READ_CMPLT			0


#endif
