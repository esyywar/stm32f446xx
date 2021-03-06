/*
 * stm32f446xx_dma_driver.h
 *
 *  Created on: Jun. 11, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_DMA_DRIVER_H_
#define INC_STM32F446XX_DMA_DRIVER_H_


/*
 * DMA Configuration structure
 */
typedef struct {
	uint8_t DMA_Channel;
	uint8_t DMA_Dir;
	uint8_t DMA_PeriphInc;
	uint8_t DMA_MemInc;
	uint8_t DMA_PeriphDataSize;
	uint8_t DMA_MemDataSize;
	uint8_t DMA_Mode;
	uint8_t DMA_Priority;
	uint8_t DMA_FIFOMode;
	uint8_t DMA_FIFOThresh;
	uint8_t DMA_MBurst;
	uint8_t DMA_PBurst;
} DMA_Config_t;

/*
 * DMA Handle structure
 */
typedef struct {
	DMA_RegDef_t *pDMAx;
	DMA_Config_t DMA_Config;
	uint8_t DMA_Stream;
	uint8_t state;
} DMA_Handle_t;


/********************* DMA RESET MACROS *********************/

#define DMA1_RESET()		do { (RCC->AHB1RSTR |= (1 << 21)); (RCC->AHB1RSTR |= ~(1 << 21)); } while(0)
#define DMA2_RESET()		do { (RCC->AHB1RSTR |= (1 << 22)); (RCC->AHB1RSTR |= ~(1 << 22)); } while(0)



/************************ DRIVER APIS ************************/

/*
 * Peripheral clock setup
 */
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnOrDi);


/*
 * Initialize and reset DMA
 */
void DMA_Init(DMA_Handle_t *pDMAxHandle);
void DMA_DeInit(DMA_RegDef_t *pDMAx);

/*
 * Start DMA transfer
 */
void DMA_Start(DMA_Handle_t *pDMAxHandle, uint32_t *pSrcAddr, uint32_t *pDestArrd, uint16_t dataLen);

/*
 * DMA Interrupt Configuration
 */
void DMA_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void DMA_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Interrupt enabled DMA transfer
 */
uint8_t DMA_Start_IT(DMA_Handle_t *pDMAxHandle, uint32_t *pSrcAddr, uint32_t *pDestArrd, uint16_t dataLen);

void DMA_Close_IT(DMA_Handle_t *pDMAxHandle);

/*
 * Read flag status from the SR
 */
uint8_t DMA_GetFlagStatus(DMA_Handle_t *pDMAxHandle, uint8_t DMA_FLAG);

/*
 * Event and error interrupt handler
 */
void DMA_EV_IRQHandling(DMA_Handle_t *pDMAxHandle);

/*
 * Application callback used to notify of interrupt events and errors
 */
__weak void DMA_ApplicationCallbackEvent(DMA_Handle_t *pDMAxHandle, uint8_t event);


/************************* CONFIGURATION VALUES *****************************/

/*
 * DMA Channel number
 */
#define DMA_CHANNEL_0				0
#define DMA_CHANNEL_1				1
#define DMA_CHANNEL_2				2
#define DMA_CHANNEL_3				3
#define DMA_CHANNEL_4				4
#define DMA_CHANNEL_5				5
#define DMA_CHANNEL_6				6
#define DMA_CHANNEL_7				7

/*
 * DMA direction
 */
#define DMA_PERIPH_TO_MEM			0
#define DMA_MEM_TO_PERIPH			1
#define DMA_MEM_TO_MEM				2

/*
 * Peripheral increment
 */
#define DMA_PERIPH_NO_INC			0
#define DMA_PERIPH_INC				1

/*
 * Memory increment
 */
#define DMA_MEM_NO_INC				0
#define DMA_MEM_INC					1

/*
 * Data size
 */
#define DMA_DATASIZE_BYTE			0
#define DMA_DATASIZE_HALFWORD		1
#define DMA_DATASIZE_WORD			2

/*
 * Circular mode
 */
#define DMA_MODE_NORMAL				0
#define DMA_MODE_CIRC				1
#define DMA_MODE_PFCCTRL			2

/*
 * Stream priority level
 */
#define DMA_PRIOR_LOW				0
#define DMA_PRIOR_MED				1
#define DMA_PRIOR_HIGH				2
#define DMA_PRIOR_VERY_HIGH			3

/*
 * FIFO or direct mode
 */
#define DMA_DIRECT_EN				0
#define DMA_DIRECT_DI				1


/*
 * FIFO threshold
 */
#define DMA_FIFO_THRESH_1DIV4		0
#define DMA_FIFO_THRESH_1DIV2		1
#define DMA_FIFO_THRESH_3DIV4		2
#define DMA_FIFO_THRESH_FULL		3


/*
 * Burst for memory and peripheral
 */
#define DMA_BURST_SINGLE			0
#define DMA_BURST_INCR4				1
#define DMA_BURST_INCR8				2
#define DMA_BURST_INCR16			3


/*
 * DMA Stream
 */
#define DMA_STREAM_0				0
#define DMA_STREAM_1				1
#define DMA_STREAM_2				2
#define DMA_STREAM_3				3
#define DMA_STREAM_4				4
#define DMA_STREAM_5				5
#define DMA_STREAM_6				6
#define DMA_STREAM_7				7


/*
 * Interrupt states
 */
#define DMA_READY					0
#define DMA_BUSY					1


/******************* STATUS REGISTER FLAGS *****************/

#define DMA_FLAG_FEIF				0
#define DMA_FLAG_DMEIF				1
#define DMA_FLAG_TEIF				2
#define DMA_FLAG_HTIF				3
#define DMA_FLAG_TCIF				4


/******************* DMA CALLBACK EVENTS ********************/

#define DMA_TRANSFER_CMPLT			0


#endif /* INC_STM32F446XX_DMA_DRIVER_H_ */
