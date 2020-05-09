/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Apr. 6, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/*
 * I2C Configuration Structure
 */
typedef struct {
	uint32_t I2C_SCLSpeed;		/* Speed of I2C - between 100 kHz - 400 kHz */
	uint16_t I2C_OwnDeviceAddr;	/* Own address of the I2C device */
	uint8_t I2C_AddrBits;		/* 7 bit and 10 bit address mode? */
	uint8_t I2C_ACKCtrl;		/* ACK control enabled? - Should be enabled */
	uint8_t I2C_FMDutyCycle;	/* Duty cycle selection for FM - 1:2 or 9:16 */
} I2C_Config_t;

/*
 * I2C Handle Structure
 */
typedef struct {
	I2C_RegDef_t* pI2Cx;
	I2C_Config_t I2C_Config_t;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint16_t SlaveAddr;
	uint8_t RptStart;
} I2C_Handle_t;


/********************* I2C RESET MACROS *********************/

#define I2C1_RESET()	do { (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR |= ~(1 << 21)); } while(0)
#define I2C2_RESET()	do { (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR |= ~(1 << 22)); } while(0)
#define I2C3_RESET()	do { (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR |= ~(1 << 23)); } while(0)


/********************* DRIVER APIS *********************/

/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
* Enable I2C communication
*/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Enable ACKing
 */
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Checks status flag indicated in the I2C status register
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t SR1_OR_SR2, uint8_t I2C_FLAG);

/*
 * Initialize and close
 */
void I2C_Init(I2C_Handle_t *pI2CxHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Read and write in master mode
 */
void I2C_7BitAddr_MasterSend(I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t rptStart);
void I2C_7BitAddr_MasterRead(I2C_RegDef_t *pI2Cx, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t rptStart);

/*
 * Read and write in slave mode
 */
void I2C_SlaveSend(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveRead(I2C_RegDef_t *pI2Cx);

/*
 * I2C interrupt configurations
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Read and write interrupt functions
 */
uint8_t I2C_7BitAddr_MasterSend_IT(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t rptStart);
uint8_t I2C_7BitAddr_MasterRead_IT(I2C_Handle_t *pI2CxHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t rptStart);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CxHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CxHandle);

void I2C_CloseTx_IT(I2C_Handle_t *pI2CxHandle);
void I2C_CloseRx_IT(I2C_Handle_t *pI2CxHandle);

/*
 * Application callback used to notify of interrupt events and errors
 */
__weak void I2C_ApplicationCallbackEvent(I2C_Handle_t *pI2CxHandle, uint8_t event);


/************************* I2C STATUS REGISTER FLAG POSITIONS *****************************/

#define I2C_GET_SR1				0
#define I2C_GET_SR2				1

/*
 * SR1
 */
#define I2C_FLAG_SB				(1 << 0)
#define I2C_FLAG_ADDR			(1 << 1)
#define I2C_FLAG_BTF			(1 << 2)
#define I2C_FLAG_ADD10			(1 << 3)
#define I2C_FLAG_STOPF			(1 << 4)
#define I2C_FLAG_RXNE			(1 << 6)
#define I2C_FLAG_TXE			(1 << 7)
#define I2C_FLAG_BERR			(1 << 8)
#define I2C_FLAG_ARLO			(1 << 9)
#define I2C_FLAG_AF 			(1 << 10)
#define I2C_FLAG_OVR			(1 << 11)
#define I2C_FLAG_PECERR			(1 << 12)
#define I2C_FLAG_TIMEOUT		(1 << 14)
#define I2C_FLAG_SMBALERT		(1 << 15)

/*
 * SR2
 */
#define I2C_FLAG_MSL			(1 << 0)
#define I2C_FLAG_BUSY			(1 << 1)
#define I2C_FLAG_TRA			(1 << 2)


/************************* CONFIGURATION VALUES *****************************/

#define I2C_RDY						0
#define I2C_BUSY_RX					1
#define I2C_BUSY_TX					2

#define I2C_SCL_SPEED_SM			100000U
#define I2C_SCL_SPEED_FM_200K		200000U
#define I2C_SCL_SPEED_FM_400K		400000U

#define I2C_7BIT_ADDR				0
#define I2C_10BIT_ADDR				1

#define I2C_ACK_DISABLE				0
#define I2C_ACK_ENABLE				1

#define I2C_FM_DCYC_2				0
#define I2C_FM_DCYC_16_9			1

#define NO_RPT_START				0
#define SET_RPT_START				1

/*
 * Callback events
 */
#define I2C_TX_CMPLT				0
#define I2C_RX_CMPLT				1
#define I2C_EV_STOP					2
#define I2C_EV_REQ_DATA				3
#define I2C_EV_REC_DATA				4

#define I2C_BUS_ERROR				5
#define I2C_ARLO_ERROR				6
#define I2C_AF_ERROR				7
#define I2C_OVR_ERROR				8
#define I2C_PEC_ERROR				9
#define I2C_TIMEOUT_ERROR			10
#define I2C_SMBALERT_ERROR			11


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
