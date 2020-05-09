/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Mar 28, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"


/*
 * SPI configuration structure
 */
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_DFF;
} SPI_Config_t;

/*
 * SPI handle structure
 */
typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;


/********************* SPI RESET MACROS *********************/

#define SPI1_RESET()		do { (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR |= ~(1 << 12)); } while(0)
#define SPI2_RESET()		do { (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR |= ~(1 << 14)); } while(0)
#define SPI3_RESET()		do { (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR |= ~(1 << 15)); } while(0)
#define SPI4_RESET()		do { (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR |= ~(1 << 13)); } while(0)


/********************* DRIVER APIS *********************/

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
* Enable SPI communication
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Checks status flag indicated in the SPI status register
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t SPI_FLAG);

/*
 * Control the SSI bit when NSS is software managed
 */
void SPI_SSI_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Control the SSOE bit which determines if NSS is toggled
 *
 * Note: Set high when in master mode with single slave. For multi slaves leave low as we must use GPIO as CS pins
 *
 */
void SPI_SSOE_Control(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Initialize and close
 */
void SPI_Init(SPI_Handle_t *pSPIxHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Read and write
 */
void SPI_Read(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);
void SPI_Write(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

/*
 * SPI interrupt configurations
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIxHandle);
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Interrupt enabled send and read data
 */
uint8_t SPI_Read_IT(SPI_Handle_t *pSPIxHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t SPI_Write_IT(SPI_Handle_t *pSPIxHandle, uint8_t *pTxBuffer, uint32_t len);


/************************* SPI STATUS REGISTER FLAG POSITIONS *****************************/

#define SPI_FLAG_RXNE			(1 << 0)
#define SPI_FLAG_TXE			(1 << 1)
#define SPI_FLAG_UDR			(1 << 3)
#define SPI_FLAG_CRCERR			(1 << 4)
#define SPI_FLAG_MODF			(1 << 5)
#define SPI_FLAG_OVR			(1 << 6)
#define SPI_FLAG_BSY			(1 << 7)
#define SPI_FLAG_FRE			(1 << 8)


/********************* SPI APPLICATION STATES ***************/

#define SPI_RDY					0
#define SPI_BUSY_TX				1
#define SPI_BUSY_RX				2

/*
 * Callback application states
 */
#define SPI_EV_TX_CMPLT			3
#define SPI_EV_RX_CMPLT			4
#define SPI_ERROR_OVR			5


/************************* CONFIGURATION VALUES *****************************/

/*
 * Master or slave
 */
#define SPI_SLAVE_SET			0
#define SPI_MSTR_SET			1

/*
 * SPI communication modes
 */
#define SPI_HALF_DPLX			0
#define SPI_FULL_DPLX			1
#define SPI_SMPLX_RX			2
#define SPI_SMPLX_TX			3

/*
 * Clock phase
 */
#define SPI_CPHA_1E				0
#define SPI_CPHA_2E				1

/*
 * Clock polarity
 */
#define SPI_CPOL_LOW			0
#define SPI_CPOL_HI				1

/*
 * Baud rate (scaling the PCLK)
 */
#define SPI_CLK_PCLK_DIV2		0
#define SPI_CLK_PCLK_DIV4		1
#define SPI_CLK_PCLK_DIV8		2
#define SPI_CLK_PCLK_DIV16		3
#define SPI_CLK_PCLK_DIV32		4
#define SPI_CLK_PCLK_DIV64		5
#define SPI_CLK_PCLK_DIV128		6
#define SPI_CLK_PCLK_DIV256		7

/*
 * SPI frame format
 */
#define SPI_MSB_FIRST			0
#define SPI_LSB_FIRST			1

/*
 * CS management
 */
#define SPI_SSM_HW				0
#define SPI_SSM_SW				1

/*
 * Data frame format
 */
#define SPI_8BIT_DFF			0
#define SPI_16BIT_DFF			1


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
