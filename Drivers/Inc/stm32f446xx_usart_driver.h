/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Apr. 18, 2020
 *      Author: rahul
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"


/*
 * USART Configuration Structure
 */
typedef struct {
	uint8_t USART_Mode;
	uint32_t USART_BaudRate;
	uint8_t USART_StopBits;
	uint8_t USART_WordLen;
	uint8_t USART_ParitySet;
	uint8_t USART_HWFlowCtrl;
	uint8_t USART_Oversampling;
	uint8_t USART_CPOL;
	uint8_t USART_CPHA;
} USART_Config_t;

/*
 * USART Handle Structure
 */
typedef struct {
	USART_RegDef_t* pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} USART_Handle_t;



/********************* USART RESET MACROS *********************/

#define USART1_RESET()		do { (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR |= ~(1 << 4)); } while(0)
#define USART2_RESET()		do { (RCC->APB2RSTR |= (1 << 17)); (RCC->APB2RSTR |= ~(1 << 17)); } while(0)
#define USART3_RESET()		do { (RCC->APB2RSTR |= (1 << 18)); (RCC->APB2RSTR |= ~(1 << 18)); } while(0)
#define UART4_RESET()		do { (RCC->APB2RSTR |= (1 << 19)); (RCC->APB2RSTR |= ~(1 << 19)); } while(0)
#define UART5_RESET()		do { (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR |= ~(1 << 20)); } while(0)
#define USART6_RESET()		do { (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR |= ~(1 << 5)); } while(0)



/********************* DRIVER APIS *********************/

/*
 * Peripheral clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
* Enable USART/UART communication
*/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * USART/UART Flag check and clear
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t USART_FLAG);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t USART_FLAG);

/*
 * Set baud rate of USART peripheral
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t USART_BaudRate);

/*
 * Initialize and close
 */
void USART_Init(USART_Handle_t *pUSARTxHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * USART send and receive data
 */
void USART_Write(USART_Handle_t *pUSARTxHandle, uint8_t *pTxBuffer, uint8_t len);
void USART_Read(USART_Handle_t *pUSARTxHandle, uint8_t *pRxBuffer, uint8_t len);

/*
 * USART/UART interrupt configurations
 */
void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Event and error interrupt handlers
 */
void USART_EV_IRQHandling(USART_Handle_t *pUSARTxHandle);
void USART_ER_IRQHandling(USART_Handle_t *pUSARTxHandle);

/*
 * USART interrupt enables write and read
 */
uint8_t USART_Write_IT(USART_Handle_t *pUSARTxHandle, uint8_t *pTxBuffer, uint8_t len);
uint8_t USART_Read_IT(USART_Handle_t *pUSARTxHandle, uint8_t *pRxBuffer, uint8_t len);

void USART_CloseTx_IT(USART_Handle_t *pUSARTxHandle);
void USART_CloseRx_IT(USART_Handle_t *pUSARTxHandle);

/*
 * Application callback used to notify of interrupt events and errors
 */
__weak void USART_ApplicationCallbackEvent(USART_Handle_t *pUSARTxHandle, uint8_t event);


/************************* CONFIGURATION VALUES *****************************/

#define USART_CPOL_LOW				0
#define USART_CPOL_HI				1

#define USART_CPHA_1E				0
#define USART_CPHA_2E				1

/*
 * USART Mode
 */
#define USART_RX_ONLY				0
#define USART_TX_ONLY				1
#define USART_RX_TX					2

/*
 * USART baud rate
 */
#define USART_BAUD_1200				1200U
#define USART_BAUD_2400				2400U
#define USART_BAUD_9600				9600U
#define USART_BAUD_19200			19200U
#define USART_BAUD_38400			38400U
#define USART_BAUD_57600			57600U
#define USART_BAUD_115200			115200U
#define USART_BAUD_230400			230400U
#define USART_BAUD_460800			460800U
#define USART_BAUD_921600			921600U

/*
 * USART stop bits
 */
#define USART_HALF_STOP				0
#define USART_1_STOP				1
#define USART_1ANDHALF_STOP			2
#define USART_2_STOP				3

/*
 * Word length
 */
#define USART_8BIT_DATA				0
#define USART_9BIT_DATA				1

/*
 * Parity bit
 */
#define USART_PARITY_DISABLE		0
#define USART_EVEN_PARITY			1
#define USART_ODD_PARITY			2

/*
 * HW flow control
 */
#define USART_HWCTRL_NONE			0
#define USART_RTS_ON				1
#define USART_CTS_ON				2
#define USART_RTS_CTS_ON			3

/*
 * Over sampling mode
 */
#define USART_OVERSMPL_16			0
#define USART_OVERSMPL_8			1

/*
 * Clock phase
 */
#define USART_CPHA_1E				0
#define USART_CPHA_2E				1

/*
 * Clock polarity
 */
#define USART_CPOL_LOW				0
#define USART_CPOL_HI				1

/*
 * Interrupt states
 */
#define USART_RDY					0
#define USART_BUSY_RX				1
#define USART_BUSY_TX				2

/*
 * SR flags
 */
#define USART_FLAG_PE				(1 << 0)
#define USART_FLAG_FE				(1 << 1)
#define USART_FLAG_NF				(1 << 2)
#define USART_FLAG_ORE				(1 << 3)
#define USART_FLAG_IDLE				(1 << 4)
#define USART_FLAG_RXNE				(1 << 5)
#define USART_FLAG_TC				(1 << 6)
#define USART_FLAG_TXE				(1 << 7)
#define USART_FLAG_LBD				(1 << 8)
#define USART_FLAG_CTS				(1 << 9)

/*
 * Callback events
 */
#define USART_TX_CMPLT				0
#define USART_RX_CMPLT				1
#define USART_EV_REQ_DATA			2
#define USART_EV_REC_DATA			3

#define USART_ORE_ERROR				4
#define USART_IDLE_ERROR			5
#define USART_PARITY_ERROR			6
#define USART_BREAK_ERROR			7
#define USART_MULTIBUFF_ERROR		8


#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
