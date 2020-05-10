/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Apr. 6, 2020
 *      Author: Rahul
 */

#include "stm32f446xx_i2c_driver.h"


/************************************************* HELPER FUNCTIONS *******************************************/

static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStop(I2C_RegDef_t *pI2Cx);
static void I2C_7BitAddr_LoadAddr_Tx(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_7BitAddr_LoadAddr_Rx(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearADDR(I2C_RegDef_t *pI2Cx);
static void I2C_RiseTimeSet(I2C_Handle_t *pI2CxHandle);

static void I2C_MasterSend_IT_Handle(I2C_Handle_t *pI2CxHandle);
static void I2C_MasterRead_IT_Handle(I2C_Handle_t *pI2CxHandle);


/*************************************************** DRIVER API IMPLMENTATION ***********************************************/


/*
 * Function:	I2C_PeriClockControl
 *
 * Brief: 		Enable and disable clock signal to I2C peripherals
 *
 * Params: 		struct I2C_RegDef_t* pI2Cx- I2C base address
 * 				uint8_t EnOrDi - Enable or disable value
 *
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


/*
 * Function:	I2C_PeripheralControl
 *
 * Brief: 		Enable or disable I2C peripheral communication
 *
 * Params: 		struct I2C_RegDef_t* pI2Cx - I2C base address
 * 				uint8_t EnOrDi - Enable or disable I2C communication (0 or 1)
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << 0);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}


/*
 * Function:	I2C_AckControl
 *
 * Brief: 		Control ACK on the I2C channel
 *
 * Params: 		struct I2C_RegDef_t *pI2Cx - I2C register structure
 * 				uint8_t EnOrDi - Enable or disable ACK?
 *
 */
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << 10);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << 10);
	}
}


/*
 * Function:	I2C_GetFlagStatus
 *
 * Brief: 		Check status registers of I2C to see if flag is set
 *
 * Params: 		struct I2C_RegDef_t* pI2Cx - I2C base address
 * 				uint8_t SR1_OR_SR2 - I2C has 2 status registers; must specify which to check
 * 				uint8_t I2C_FLAG - flag being checked
 *
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t SR1_OR_SR2, uint8_t I2C_FLAG)
{
	if (SR1_OR_SR2 == I2C_GET_SR1)
	{
		if ((pI2Cx->SR1 & I2C_FLAG) > 0)
		{
			return SET;
		}
		else
		{
			return RESET;
		}
	}
	else
	{
		if ((pI2Cx->SR2 & I2C_FLAG) > 0)
		{
			return SET;
		}
		else
		{
			return RESET;
		}
	}
}


/*
 * Function:	I2C_Init
 *
 * Brief: 		Initialize the I2C peripheral
 *
 * Params: 		struct I2C_Handle_t* pI2CxHandle - I2C peripheral handle
 *
 */
void I2C_Init(I2C_Handle_t *pI2CxHandle)
{
	// 1. Set peripheral clock frequency
	uint8_t freq = RCC_GetAPB1CLK() / 1000000U;
	pI2CxHandle->pI2Cx->CR2 |= (freq & 0x3F);

	// 2. Set the device's own address
	pI2CxHandle->pI2Cx->OAR1 |= (pI2CxHandle->I2C_Config_t.I2C_AddrBits << 15);

	if (pI2CxHandle->I2C_Config_t.I2C_AddrBits == I2C_7BIT_ADDR)
	{
		pI2CxHandle->pI2Cx->OAR1 |= (pI2CxHandle->I2C_Config_t.I2C_OwnDeviceAddr << 1);
	}
	else if (pI2CxHandle->I2C_Config_t.I2C_AddrBits == I2C_10BIT_ADDR)
	{
		pI2CxHandle->pI2Cx->OAR1 |= (pI2CxHandle->I2C_Config_t.I2C_OwnDeviceAddr << 0);
	}

	// 3. Set SM or FM in the CCR register
	uint16_t CCR;

	if (pI2CxHandle->I2C_Config_t.I2C_SCLSpeed > I2C_SCL_SPEED_SM)
	{
		// Fast mode
		pI2CxHandle->pI2Cx->CCR |= (1 << 15);
		pI2CxHandle->pI2Cx->CCR |= (pI2CxHandle->I2C_Config_t.I2C_FMDutyCycle << 14);

		if (pI2CxHandle->I2C_Config_t.I2C_FMDutyCycle == I2C_FM_DCYC_2)
		{
			CCR = RCC_GetAPB1CLK() / (3 * pI2CxHandle->I2C_Config_t.I2C_SCLSpeed);
		}
		else if (pI2CxHandle->I2C_Config_t.I2C_FMDutyCycle == I2C_FM_DCYC_16_9)
		{
			CCR = RCC_GetAPB1CLK() / (25 * pI2CxHandle->I2C_Config_t.I2C_SCLSpeed);
		}
	}
	else
	{
		// Standard mode
		pI2CxHandle->pI2Cx->CCR &= ~(1 << 15);

		CCR = RCC_GetAPB1CLK() / (2 * pI2CxHandle->I2C_Config_t.I2C_SCLSpeed);
	}

	//  Set the CCR
	pI2CxHandle->pI2Cx->CCR |= (CCR & 0xFFF);

	// 5. Set the rise time register
	I2C_RiseTimeSet(pI2CxHandle);
}


/*
 * Function:	I2C_DeInit
 *
 * Brief: 		Reset the I2C peripheral
 *
 * Params: 		struct I2C_RegDef_t* pI2Cx - I2C register pointer
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_RESET();
	}
}


/****************************************** MASTER MODE DRIVER APIS *************************************************/

/*
 * Function:	I2C_7BitAddr_MasterSend
 *
 * Brief: 		Send data when set as master and using 7-bit addressing
 *
 * Params: 		struct I2C_RegDef_t *pI2Cx - I2C register address
 * 				uint8_t* pTxBuffer - pointer to data to be sent
 * 				uint32_t len - number of 8-bit data packets to be sent
 * 				uint8_t* pSlaveAddr - 7-bit address of slave
 * 				uint8_t - is repeated start required?
 *
 */
void I2C_7BitAddr_MasterSend(I2C_RegDef_t *pI2Cx, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t rptStart)
{
	// Enable peripheral
	I2C_PeripheralControl(pI2Cx, ENABLE);

	// 1. Enable ACK
	I2C_AckControl(pI2Cx, ENABLE);

	// 2. Generate start
	I2C_GenerateStart(pI2Cx);

	// 3. Wait for SB flag and load address
	while(! I2C_GetFlagStatus(pI2Cx, I2C_GET_SR1, I2C_FLAG_SB));
	I2C_7BitAddr_LoadAddr_Tx(pI2Cx, slaveAddr);

	// 4. Wait for address ACK and clear ADDR bit
	while(! I2C_GetFlagStatus(pI2Cx, I2C_GET_SR1, I2C_FLAG_ADDR));
	I2C_ClearADDR(pI2Cx);

	// 5. Write the data checking TXE before sending each bit to DR
	while (len > 0)
	{
		while(! I2C_GetFlagStatus(pI2Cx, I2C_GET_SR1, I2C_FLAG_TXE));
		pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	// 6. Wait for TXE and BTF set
	while(! I2C_GetFlagStatus(pI2Cx, I2C_GET_SR1, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2Cx, I2C_GET_SR1, I2C_FLAG_BTF));

	// 7. Send stop bit if not repeated start
	if (rptStart == NO_RPT_START)
	{
		I2C_GenerateStop(pI2Cx);
	}
}


/*
 * Function:	I2C_7BitAddr_MasterRead
 *
 * Brief: 		Read data when set as master and using 7-bit addressing
 *
 * Params: 		struct I2C_RegDef_t *pI2Cx - I2C register address
 * 				uint8_t* pRxBuffer - pointer to location where read data is stored
 * 				uint32_t len - number of 8-bit data packets to be read
 * 				uint8_t* pSlaveAddr - 7-bit address of slave
 * 				uint8_t - is repeated start required?
 *
 */
void I2C_7BitAddr_MasterRead(I2C_RegDef_t *pI2Cx, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t rptStart)
{
	// Enable peripheral
	I2C_PeripheralControl(pI2Cx, ENABLE);

	// 1. Enable ACK
	I2C_AckControl(pI2Cx, ENABLE);

	// 2. Generate start
	I2C_GenerateStart(pI2Cx);

	// 3. Wait for SB flag and load address
	while(! I2C_GetFlagStatus(pI2Cx, I2C_GET_SR1, I2C_FLAG_SB));
	I2C_7BitAddr_LoadAddr_Rx(pI2Cx, slaveAddr);

	// 4. If data is 1 byte, disable ACK so that NACK bit is sent
	if (len == 1)
	{
		I2C_AckControl(pI2Cx, DISABLE);
	}

	// 4. Wait for address ACK - clear by reading SR1 and SR2
	while(! I2C_GetFlagStatus(pI2Cx, I2C_GET_SR1, I2C_FLAG_ADDR));
	I2C_ClearADDR(pI2Cx);

	// 5. Read data from DR
	while (len > 0)
	{
		while(! I2C_GetFlagStatus(pI2Cx, I2C_GET_SR1, I2C_FLAG_RXNE));
		*pRxBuffer = pI2Cx->DR;
		pRxBuffer++;
		len--;

		// 6. On second last byte, disable ACK and send STOP if no repeated start
		if (len == 1)
		{
			I2C_AckControl(pI2Cx, DISABLE);

			if (rptStart == NO_RPT_START)
			{
				I2C_GenerateStop(pI2Cx);
			}
		}
	}
}


/*
 * Function:	I2C_7BitAddr_MasterSend_IT
 *
 * Brief: 		Send I2C data using non-blocking interrupts
 *
 * Params: 		struct I2C_Handle_t* - I2C handle
 * 				uint8_t* pTxBuffer - pointer to data to be sent
 * 				uint32_t len - number of 8-bit data packets to be sent
 * 				uint8_t* pSlaveAddr - 7-bit address of slave
 * 				uint8_t - is repeated start required?
 *
 */
uint8_t I2C_7BitAddr_MasterSend_IT(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t rptStart)
{
	uint8_t state = pI2CxHandle->TxRxState;

	if (state == I2C_RDY)
	{
		// 1. Set state to busy in Tx
		pI2CxHandle->TxRxState = I2C_BUSY_TX;

		// 2. Store write data parameters in handle
		pI2CxHandle->SlaveAddr = slaveAddr;
		pI2CxHandle->pTxBuffer = pTxBuffer;
		pI2CxHandle->TxLen = len;
		pI2CxHandle->RptStart = rptStart;

		// 3. Enable peripheral
		I2C_PeripheralControl(pI2CxHandle->pI2Cx, ENABLE);

		// 4. Enable ACK
		I2C_AckControl(pI2CxHandle->pI2Cx, ENABLE);

		// 5. Send start to claim the bus
		I2C_GenerateStart(pI2CxHandle->pI2Cx);

		// 6. Event interrupt enable
		pI2CxHandle->pI2Cx->CR2 |= (1 << 10);
		pI2CxHandle->pI2Cx->CR2 |= (1 << 9);
		pI2CxHandle->pI2Cx->CR2 |= (1 << 8);
	}

	return state;
}


/*
 * Function:	I2C_7BitAddr_MasterRead_IT
 *
 * Brief: 		Read data from I2V with non-blocking interrupt
 *
 * Params: 		struct I2C_Handle_t* - I2C handle
 * 				uint8_t* pRxBuffer - pointer to location where read data is stored
 * 				uint32_t RxSixe - size of data packets to be received
 * 				uint32_t len - number of 8-bit data packets to be sent
 * 				uint8_t* pSlaveAddr - 7-bit address of slave
 * 				uint8_t - is repeated start required?
 *
 */
uint8_t I2C_7BitAddr_MasterRead_IT(I2C_Handle_t *pI2CxHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t rptStart)
{
	uint8_t state = pI2CxHandle->TxRxState;

	if (state == I2C_RDY)
	{
		// 1. Set state to busy in Rx
		pI2CxHandle->TxRxState = I2C_BUSY_RX;

		// 2. Store write data parameters in handle
		pI2CxHandle->SlaveAddr = slaveAddr;
		pI2CxHandle->pRxBuffer = pRxBuffer;
		pI2CxHandle->RxLen = len;
		pI2CxHandle->RptStart = rptStart;

		// 3. Enable peripheral
		I2C_PeripheralControl(pI2CxHandle->pI2Cx, ENABLE);

		// 4. Enable ACK
		I2C_AckControl(pI2CxHandle->pI2Cx, ENABLE);

		// 5. Send start to claim the bus
		I2C_GenerateStart(pI2CxHandle->pI2Cx);

		// 6. Event interrupt enable
		pI2CxHandle->pI2Cx->CR2 |= (1 << 10);
		pI2CxHandle->pI2Cx->CR2 |= (1 << 9);
		pI2CxHandle->pI2Cx->CR2 |= (1 << 8);
	}

	return state;
}


/*
 * I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
 *
 * Brief: Toggle interrupts which will fire callback events
 *
 * Params:		struct I2C_RegDef_t *pI2Cx - pointer to I2C peripheral
 * 				uint8_t EnOrDi - enable or disable?
 *
 *
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		// Enable error and event interrupts
		pI2Cx->CR2 |= (1 << 10);
		pI2Cx->CR2 |= (1 << 9);
		pI2Cx->CR2 |= (1 << 8);
	}
	else
	{
		pI2Cx->CR2 &= ~(1 << 10);
		pI2Cx->CR2 &= ~(1 << 9);
		pI2Cx->CR2 &= ~(1 << 8);
	}
}


/*********************************************** SLAVE MODE DRIVER APIS ****************************************************/


void I2C_SlaveSend(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveRead(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)(pI2Cx->DR);
}


/************************************************ INTERRUPT CONFIG AND HANLDER APIS ****************************************/

/*
 * Function:	I2C_EV_IRQHandling
 *
 * Brief: 		Function called by event interrupt on I2C
 * 				Determines interrupt flag and responds
 *
 * Params: 		struct I2C_Handle_t* pI2CxHandle - I2C handle
 *
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CxHandle)
{
	uint16_t temp1, temp2, temp3;

	// 1. Check if event and buffer interrupt are set
	temp1 = pI2CxHandle->pI2Cx->CR2 & (1 << 10);
	temp2 = pI2CxHandle->pI2Cx->CR2 & (1 << 9);

	// 2. Is SB interrupt?
	temp3 = pI2CxHandle->pI2Cx->SR1 & (1 << 0);
	if (temp1 && temp3)
	{
		// Send slave address with R/W bit depending on state
		if (pI2CxHandle->TxRxState == I2C_BUSY_TX)
		{
			I2C_7BitAddr_LoadAddr_Tx(pI2CxHandle->pI2Cx, pI2CxHandle->SlaveAddr);
		}
		else if (pI2CxHandle->TxRxState == I2C_BUSY_RX)
		{
			I2C_7BitAddr_LoadAddr_Rx(pI2CxHandle->pI2Cx, pI2CxHandle->SlaveAddr);
		}
	}

	// 3. Is ADDR interrupt?
	temp3 = pI2CxHandle->pI2Cx->SR1 & (1 << 1);
	if (temp1 && temp3)
	{
		// If in master read and only 1 byte, need to disable ACK before clearing ADDR
		if (I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_GET_SR2, I2C_FLAG_MSL) && pI2CxHandle->TxRxState == I2C_BUSY_RX && (pI2CxHandle->RxLen == 1))
		{
			I2C_AckControl(pI2CxHandle->pI2Cx, DISABLE);
		}

		// Clear the ADDR bit
		I2C_ClearADDR(pI2CxHandle->pI2Cx);
	}

	// 4. Is BTF interrupt?
	temp3 = pI2CxHandle->pI2Cx->SR1 & (1 << 2);
	if (temp1 && temp3)
	{
		// If in master transmission mode, TxLen = 0 -> close Tx interrupt
		if (I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_GET_SR2, I2C_FLAG_MSL) && pI2CxHandle->TxRxState == I2C_BUSY_TX && (pI2CxHandle->pI2Cx->SR1 & (1 << 7)))
		{
			I2C_CloseTx_IT(pI2CxHandle);

			// Send stop bit if not repeated start
			if (pI2CxHandle->RptStart == NO_RPT_START)
			{
				I2C_GenerateStop(pI2CxHandle->pI2Cx);
			}
		}
		// If in Rx mode and RxNE flag is set, read from DR
		else if (pI2CxHandle->TxRxState == I2C_BUSY_RX && pI2CxHandle->pI2Cx->SR1 & (1 << 6))
		{
			if (pI2CxHandle->pI2Cx->SR2 & (1 << 0))
			{
				I2C_MasterRead_IT_Handle(pI2CxHandle);
			}
			else
			{
				I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_EV_REC_DATA);
			}
		}
	}

	// 5. Is STOPF interrupt? (only happens in slave mode)
	temp3 = pI2CxHandle->pI2Cx->SR1 & (1 << 4);
	if (temp1 && temp3)
	{
		// Clear STOPF by reading SR1 and writing CR1
		pI2CxHandle->pI2Cx->CR1 |= 0x0000;

		I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_EV_STOP);
	}

	// 6. Is RxNE interrupt and master mode?
	temp3 = pI2CxHandle->pI2Cx->SR1 & (1 << 6) && pI2CxHandle->pI2Cx->SR2 & (1 << 0);
	if (temp1 && temp2 && temp3)
	{
		I2C_MasterRead_IT_Handle(pI2CxHandle);
	}

	// 7. Is in RxNE and slave mode?
	temp3 = pI2CxHandle->pI2Cx->SR1 & (1 << 6);
	if (temp1 && temp2 && temp3)
	{
		// If slave, confirm TRA bit is Rx
		if (!(pI2CxHandle->pI2Cx->SR2 & (1 << 2)))
		{
			I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_EV_REC_DATA);
		}
	}


	// 8. Is TxE interrupt?
	temp3 = pI2CxHandle->pI2Cx->SR1 & (1 << 7);
	if (temp1 && temp2 && temp3)
	{
		// Check master or slave mode
		if (I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_GET_SR2, I2C_FLAG_MSL))
		{
			I2C_MasterSend_IT_Handle(pI2CxHandle);
		}
		else
		{
			// If slave, confirm TRA bit is Tx
			if (pI2CxHandle->pI2Cx->SR2 & (1 << 2))
			{
				I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_EV_REQ_DATA);
			}
		}

	}
}


/*
 * Function:	I2C_ER_IRQHandling
 *
 * Brief: 		Function called by error interrupt on I2C
 * 				Determines interrupt flag and responds
 *
 * Params: 		struct I2C_Handle_t* pI2CxHandle - I2C handle
 *
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CxHandle)
{
	uint16_t temp1, temp2;

	// 1. Check if error interrupt is set
	temp1 = pI2CxHandle->pI2Cx->CR2 & (1 << 8);

	// 2. Is BERR error?
	temp2 = pI2CxHandle->pI2Cx->SR1 & (1 << 8);
	if (temp1 && temp2)
	{
		// Clear error flag and send callback
		pI2CxHandle->pI2Cx->SR1 &= ~(1 << 8);
		I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_BUS_ERROR);
	}

	// 3. Is ARLO error?
	temp2 = pI2CxHandle->pI2Cx->SR1 & (1 << 9);
	if (temp1 && temp2)
	{
		// Clear error flag and send callback
		pI2CxHandle->pI2Cx->SR1 &= ~(1 << 9);
		I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_ARLO_ERROR);
	}

	// 3. Is AF error?
	temp2 = pI2CxHandle->pI2Cx->SR1 & (1 << 10);
	if (temp1 && temp2)
	{
		// Clear error flag and send callback
		pI2CxHandle->pI2Cx->SR1 &= ~(1 << 10);
		I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_AF_ERROR);
	}

	// 4. Is OVR error?
	temp2 = pI2CxHandle->pI2Cx->SR1 & (1 << 11);
	if (temp1 && temp2)
	{
		// Clear error flag and send callback
		pI2CxHandle->pI2Cx->SR1 &= ~(1 << 11);
		I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_OVR_ERROR);
	}

	// 5. Is PEC error?
	temp2 = pI2CxHandle->pI2Cx->SR1 & (1 << 12);
	if (temp1 && temp2)
	{
		// Clear error flag and send callback
		pI2CxHandle->pI2Cx->SR1 &= ~(1 << 12);
		I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_PEC_ERROR);
	}

	// 6. Is TIMEOUT error?
	temp2 = pI2CxHandle->pI2Cx->SR1 & (1 << 14);
	if (temp1 && temp2)
	{
		// Clear error flag and send callback
		pI2CxHandle->pI2Cx->SR1 &= ~(1 << 14);
		I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_TIMEOUT_ERROR);
	}

	// 7. Is SMBAlert error?
	temp2 = pI2CxHandle->pI2Cx->SR1 & (1 << 15);
	if (temp1 && temp2)
	{
		// Clear error flag and send callback
		pI2CxHandle->pI2Cx->SR1 &= ~(1 << 15);
		I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_SMBALERT_ERROR);
	}
}


/*
 * Function:	I2C_IRQConfig
 *
 * Brief: 		Set or clear interrupt in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t Value - Enabling or disabling interrupt (1 or 0)
 *
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		NVIC_ISER->ISER[IRQNumber / 32] |= (1 << (IRQNumber % 32));
	}
	else if (EnOrDi == DISABLE)
	{
		NVIC_ICER->ICER[IRQNumber / 32] |= (1 << (IRQNumber % 32));
	}
}


/*
 * Function:	I2C_IRQPriorityConfig
 *
 * Brief: 		Set interrupt priority in processor NVIC
 *
 * Params: 		uint8_t IRQNumber - IRQ position being configured
 * 				uint8_t IRQPriority - Priority value (0 - 255)
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t shift = 8 * (IRQNumber % 4) + NVIC_NONIMPL_LOW_BITS;
	NVIC_IPR->IPR[IRQNumber / 4] |= (IRQPriority << shift);
}


/*
 * Function:	I2C_GenerateStart
 *
 * Brief: 		Helper function to generate START on the I2C channel
 *
 * Params: 		struct I2C_RegDef_t *pI2Cx - I2C register structure
 *
 */
static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << 8);
}


/*
 * Function:	I2C_GenerateStop
 *
 * Brief: 		Helper function to generate STOP on the I2C channel
 *
 * Params: 		struct I2C_RegDef_t *pI2Cx - I2C register structure
 *
 */
static void I2C_GenerateStop(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << 9);
}


/*
 * Function:	I2C_7BitAddr_LoadAddr_Tx
 *
 * Brief: 		Helper function to load address in DR with W bit to enter master Tx mode
 *
 * Params: 		struct I2C_RegDef_t *pI2Cx - I2C register structure
 * 				uint8_t slaveAddr - 7-bit slave address
 *
 */
static void I2C_7BitAddr_LoadAddr_Tx(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	pI2Cx->DR = ((slaveAddr << 1) & 0xFE);
}


/*
 * Function:	I2C_7BitAddr_LoadAddr_Rx
 *
 * Brief: 		Helper function to load address in DR with R bit to enter master Tx mode
 *
 * Params: 		struct I2C_RegDef_t *pI2Cx - I2C register structure
 * 				uint8_t slaveAddr - 7-bit slave address
 *
 */
static void I2C_7BitAddr_LoadAddr_Rx(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	pI2Cx->DR = ((slaveAddr << 1) | 1);
}


/*
 * Function:	I2C_ClearADDR
 *
 * Brief: 		Helper function to clear ADDR bit by reading from SR1 then SR2
 *
 * Params: 		struct I2C_RegDef_t *pI2Cx - I2C register structure
 *
 */
static void I2C_ClearADDR(I2C_RegDef_t *pI2Cx)
{
	uint16_t dummyRead;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}


/*
 * Function:	I2C_RiseTimeSet
 *
 * Brief: 		Calculates and writes to rise time register using configured speed and I2C clock
 *
 * Params: 		struct I2C_Handle_t *pI2CxHandle - I2C handle structure
 *
 */
static void I2C_RiseTimeSet(I2C_Handle_t *pI2CxHandle)
{
	uint8_t riseTimeReg;

	if (pI2CxHandle->I2C_Config_t.I2C_SCLSpeed > I2C_SCL_SPEED_SM)
	{
		// Fast mode rise time maximum is 300 ns
		riseTimeReg = ((RCC_GetAPB1CLK() * 3) / 10000000U) + 1;
	}
	else
	{
		// Standard mode rise time is maximum 1000 ns
		riseTimeReg = (RCC_GetAPB1CLK() / 1000000U) + 1;
	}

	pI2CxHandle->pI2Cx->TRISE = (riseTimeReg & 0x3F);
}


/*
 * Function:	I2C_MasterSend_IT_Handle
 *
 * Brief: 		Helper function for interrupt handling in master mode
 * 				Called when TxE flags interrupt
 * 				Writes data to DR
 *
 * Params: 		struct I2C_Handle_t *pI2CxHandle - I2C handle structure
 *
 */
static void I2C_MasterSend_IT_Handle(I2C_Handle_t *pI2CxHandle)
{
	// If in Tx mode, write data
	if (pI2CxHandle->TxRxState == I2C_BUSY_TX && pI2CxHandle->TxLen > 0)
	{
		pI2CxHandle->pI2Cx->DR = *(pI2CxHandle->pTxBuffer);
		(pI2CxHandle->TxLen)--;
		(pI2CxHandle->pTxBuffer)++;

		// If last byte has transmitted, wait for BTF and close transmission
		if (pI2CxHandle->TxLen == 0)
		{
			while(! I2C_GetFlagStatus(pI2CxHandle->pI2Cx, I2C_GET_SR1, I2C_FLAG_BTF));

			if (pI2CxHandle->RptStart == NO_RPT_START)
			{
				I2C_GenerateStop(pI2CxHandle->pI2Cx);
			}

			I2C_CloseTx_IT(pI2CxHandle);
		}
	}
}


/*
 * Function:	I2C_MasterRead_IT_Handle
 *
 * Brief: 		Helper function for interrupt handling in master mode
 * 				Called when RxNE flags interrupt
 * 				Closes communication by managing ACK and STOP bits
 *
 * Params: 		struct I2C_Handle_t *pI2CxHandle - I2C handle structure
 *
 */
static void I2C_MasterRead_IT_Handle(I2C_Handle_t *pI2CxHandle)
{
	// If in Rx mode, read data
	if (pI2CxHandle->TxRxState == I2C_BUSY_RX && pI2CxHandle->RxLen > 0)
	{
		*(pI2CxHandle->pRxBuffer) = pI2CxHandle->pI2Cx->DR;
		(pI2CxHandle->RxLen)--;
		(pI2CxHandle->pRxBuffer)++;

		// If received second last bit, disable NACK
		if (pI2CxHandle->RxLen == 1)
		{
			I2C_AckControl(pI2CxHandle->pI2Cx, DISABLE);

			// If not repeated start, set STOP
			if (pI2CxHandle->RptStart == NO_RPT_START)
			{
				I2C_GenerateStop(pI2CxHandle->pI2Cx);
			}
		}

		// If received last bit -> clear interrupt, reset handle, send callback
		if (pI2CxHandle->RxLen == 0)
		{
			I2C_CloseRx_IT(pI2CxHandle);
		}
	}
}


/*
 * Function:	I2C_CloseTx_IT
 *
 * Brief: 		Helper function for closing data write interrupts
 *
 * Params: 		struct I2C_Handle_t *pI2CxHandle - I2C handle structure
 *
 */
void I2C_CloseTx_IT(I2C_Handle_t *pI2CxHandle)
{
	// Clear all interrupts
	pI2CxHandle->pI2Cx->CR2 &= ~(1 << 10);
	pI2CxHandle->pI2Cx->CR2 &= ~(1 << 9);
	pI2CxHandle->pI2Cx->CR2 &= ~(1 << 8);

	// Reset handle
	pI2CxHandle->TxLen = 0;
	pI2CxHandle->pTxBuffer = NULL;
	pI2CxHandle->TxRxState = I2C_RDY;

	I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_TX_CMPLT);
}


/*
 * Function:	I2C_CloseRx_IT
 *
 * Brief: 		Helper function for closing data read interrupts
 *
 * Params: 		struct I2C_Handle_t *pI2CxHandle - I2C handle structure
 *
 */
void I2C_CloseRx_IT(I2C_Handle_t *pI2CxHandle)
{
	// Clear ITBUFEN and ITEVFEN
	pI2CxHandle->pI2Cx->CR2 &= ~(1 << 10);
	pI2CxHandle->pI2Cx->CR2 &= ~(1 << 9);
	pI2CxHandle->pI2Cx->CR2 &= ~(1 << 8);

	// Reset handle
	pI2CxHandle->RxLen = 0;
	pI2CxHandle->pRxBuffer = NULL;
	pI2CxHandle->TxRxState = I2C_RDY;

	// Re-enable ACK if configured
	if (pI2CxHandle->I2C_Config_t.I2C_ACKCtrl == ENABLE)
	{
		I2C_AckControl(pI2CxHandle->pI2Cx, ENABLE);
	}

	I2C_ApplicationCallbackEvent(pI2CxHandle, I2C_RX_CMPLT);
}

