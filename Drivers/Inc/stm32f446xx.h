/*
 * stm32f446xx.h
 *
 *  Created on: Mar 21, 2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>


/*
 * General macros
 */
#define __vo		volatile
#define __weak 		__attribute__((weak))

#define LOW			0
#define HIGH		1
#define DISABLE		0
#define ENABLE		1
#define SET			ENABLE
#define RESET		DISABLE


/************* ARM M4 PROCESSOR SPECIFIC ADDRESSES **********/

#define NVIC_ISER0_BASEADDR		0xE000E100U
#define NVIC_ICER0_BASEADDR		0xE000E180U
#define NVIC_IPR0_BASEADDR		0xE000E400U

#define NVIC_SYSTCK_BASEADDR	0xE000E010U

#define NVIC_NONIMPL_LOW_BITS	4


/************* STM32F446RE MEMORY BASE ADDRESSES **********/

/*
 * Main memory block addresses
 */
#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0X20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define SYSMEM_BASEADDR		0X1FFF0000U
#define SRAM				SRAM1_BASEADDR


/*
 * Peripheral bus base addresses
 */
#define PERIPH_BASEADDR		0x40000000U
#define APB1_BASEADDR		PERIPH_BASEADDR
#define APB2_BASEADDR		0x40010000U
#define AHB1_BASEADDR		0x40020000U
#define AHB2_BASEADDR		0x50000000U
#define AHB3_BASEADDR		0X60000000U


/*
 * APB1 peripheral base addresses (SPI, USART, I2C)
 */
#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0X3C00)

#define USART2_BASEADDR		(APB1_BASEADDR + 0X4400)
#define USART3_BASEADDR		(APB1_BASEADDR + 0X4800)
#define UART4_BASEADDR		(APB1_BASEADDR + 0X4C00)
#define UART5_BASEADDR		(APB1_BASEADDR + 0X5000)

#define I2C1_BASEADDR		(APB1_BASEADDR + 0X5400)
#define I2C2_BASEADDR		(APB1_BASEADDR + 0X5800)
#define I2C3_BASEADDR		(APB1_BASEADDR + 0X5C00)


/*
 * APB2 peripheral base addresses (SPI, USART, EXTI, SYSCFG)
 */
#define USART1_BASEADDR		(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400)

#define ADC_BASEADDR		(APB2_BASEADDR + 0x2000)
#define ADC1_BASEADDR		(ADC_BASEADDR)
#define ADC2_BASEADDR		(ADC_BASEADDR + 0x100)
#define ADC3_BASEADDR		(ADC_BASEADDR + 0x200)
#define ADC_COMM_BASEADDR	(ADC1_BASEADDR + 0x300)

#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR		(APB2_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800)

#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00)


/*
 * AHB1 peripheral base addresses
 */
#define GPIOA_BASEADDR		AHB1_BASEADDR
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x800)
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0xC00)
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1_BASEADDR + 0x1C00)

#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800)

#define DMA1_BASEADDR		(AHB1_BASEADDR + 0x6000)
#define DMA2_BASEADDR		(AHB1_BASEADDR + 0x6400)


/*
 * GPIO register offsets
 */
#define GPIO_MODER_OFFSET		0x00U
#define GPIO_OTYPER_OFFSET		0x04U
#define GPIO_OSPEEDR_OFFSET		0x08U
#define GPIO_PUPDR_OFFSET		0x0CU
#define GPIO_IDR_OFFSET			0x10U
#define GPIO_ODR_OFFSET			0x14U
#define GPIO_BSRR_OFFSET		0x18U
#define GPIO_LCKR_OFFSET		0x1CU
#define GPIO_AFRL_OFFSET		0x20U
#define GPIO_AFRH_OFFSET		0x24U


/*
 * SPI register offsets
 */
#define SPI_CR1_OFFSET			0x00U
#define SPI_CR2_OFFSET			0x04U
#define SPI_SR_OFFSET			0x08U
#define SPI_DR_OFFSET			0x0CU
#define SPI_CRCPR_OFFSET		0x10U
#define SPI_RXCRCR_OFFSET		0x14U
#define SPI_TXCRCR_OFFSET		0x18U
#define SPI_I2SCFGR_OFFSET		0x1CU
#define SPI_I2SPR_OFFSET		0x20U


/************* PERIPHERAL REGISTER DEFINITION STRUCTURES **********/

/* GPIO register */
typedef struct {
	__vo uint32_t MODER;			/* GPIO port mode register - OFFSET 0x00 */
	__vo uint32_t OTYPER;			/* GPIO port output type register - OFFSET 0x04 */
	__vo uint32_t OSPEEDR;			/* GPIO port output speed register - OFFSET 0x08 */
	__vo uint32_t PUPDR;			/* GPIO port pull-up/pull-down register - OFFSET 0x0C */
	__vo uint32_t IDR;				/* GPIO port input data register - OFFSET 0x10 */
	__vo uint32_t ODR;				/* GPIO port output data register - OFFSET 0x14 */
	__vo uint32_t BSRR;				/* GPIO port bit set/reset register - OFFSET 0x18 */
	__vo uint32_t LCKR;				/* GPIO port configuration lock register - OFFSET 0x1C */
	__vo uint32_t AFR[2];			/* [0] GPIO alternate function low register - OFFSET 0x20, [1] GPIO alternate function high register - OFFSET 0x24 */
} GPIO_RegDef_t;


/* SPI register */
typedef struct {
	__vo uint32_t CR1;				/* SPI control register 1 - OFFSET 0x00 */
	__vo uint32_t CR2;				/* SPI control register 2 - OFFSET 0x04 */
	__vo uint32_t SR;				/* SPI status register - OFFSET 0x08 */
	__vo uint32_t DR;				/* SPI data register - OFFSET 0x0C */
	__vo uint32_t CRCPR;			/* SPI CRC polynomial register - OFFSET 0x10 */
	__vo uint32_t RXCRCR;			/* SPI RX CRC register - OFFSET 0x14 */
	__vo uint32_t TXCRCR;			/* SPI TX CRC register - OFFSET 0x18 */
	__vo uint32_t I2SCFGR;			/* SPI_I2S configuration register - OFFSET 0x1C */
	__vo uint32_t I2SPR;			/* SPI_I2S prescaler register - OFFSET 0x20 */
} SPI_RegDef_t;


/* I2C register */
typedef struct {
	__vo uint32_t CR1;				/* I2C Control register 1 - OFFSET 0x00 */
	__vo uint32_t CR2;				/* I2C Control register 2 - OFFSET 0x04 */
	__vo uint32_t OAR1;				/* I2C Own address register 1 - OFFSET 0x08 */
	__vo uint32_t OAR2;				/* I2C Own address register 2 - OFFSET 0x0C */
	__vo uint32_t DR;				/* I2C Data register - OFFSET 0x10 */
	__vo uint32_t SR1;				/* I2C Status register 1 - OFFSET 0x14 */
	__vo uint32_t SR2;				/* I2C Status register 2 - OFFSET 0x18 */
	__vo uint32_t CCR;				/* I2C Clock control register - OFFSET 0x1C */
	__vo uint32_t TRISE;			/* I2C TRISE register - OFFSET 0x20 */
	__vo uint32_t FLTR;				/* I2C FLTR register - OFFSET 0x24 */
} I2C_RegDef_t;


/* UART / USART register */
typedef struct {
	__vo uint32_t SR;				/* USART Status register - OFFSET 0x00 */
	__vo uint32_t DR;				/* USART Data register - OFFSET 0x04 */
	__vo uint32_t BRR;				/* USART Baud rate register - OFFSET 0x08 */
	__vo uint32_t CR1;				/* USART Control register 1 - OFFSET 0x0C */
	__vo uint32_t CR2;				/* USART Control register 2 - OFFSET 0x10 */
	__vo uint32_t CR3;				/* USART Control register 3 - OFFSET 0x14 */
	__vo uint32_t GTPR;				/* USART Guard time and prescaler register - OFFSET 0x18 */
} USART_RegDef_t;


/* ADC register */
typedef struct {
	__vo uint32_t SR;				/* ADC Status register - OFFSET 0x00 */
	__vo uint32_t CR1;				/* ADC Control register 1 - OFFSET 0x04 */
	__vo uint32_t CR2;				/* ADC Control register 2 - OFFSET 0x08 */
	__vo uint32_t SMPR[2];			/* ADC sample time register x - OFFSET 0x0C */
	__vo uint32_t JOFR[4];			/* ADC injected channel data offset register x - OFFSET 0x14 */
	__vo uint32_t HTR;				/* ADC watchdog higher threshold register - OFFSET 0x24 */
	__vo uint32_t LTR;				/* ADC watchdog lower threshold register - OFFSET 0x28 */
	__vo uint32_t SQR[3];			/* ADC regular sequence register x - OFFSET 0x2C */
	__vo uint32_t JSQR;				/* ADC injected sequence register - OFFSET 0x38 */
	__vo uint32_t JDR[4];			/* ADC injected data register x - OFFSET 0x3C */
	__vo uint32_t DR;				/* ADC regular data register - OFFSET 0x4C */
} ADC_RegDef_t;


/* ADC Common register */
typedef struct {
	__vo uint32_t CSR;				/* ADC Common status register - OFFSET 0x00 */
	__vo uint32_t CCR;				/* ADC common control register - OFFSET 0x04 */
	__vo uint32_t CDR;				/* ADC common regular data register for dual and triple modes - OFFSET 0x08 */
} ADC_Comm_RegDef_t;


/* DMA Stream register */
typedef struct {
	__vo uint32_t SCR;				/* DMA stream x configuration register - OFFSET 0x10 + 0x18 * stream number */
	__vo uint32_t SNDTR;			/* DMA stream x number of data register - OFFSET 0x14 + 0x18 * stream number */
	__vo uint32_t SPAR;				/* DMA stream x peripheral address register - OFFSET 0x18 + 0x18 * stream number */
	__vo uint32_t SM0AR;			/* DMA stream x memory 0 address register - OFFSET 0x1C + 0x18 * stream number */
	__vo uint32_t SM1AR;			/* DMA stream x memory 1 address register - OFFSET 0x20 + 0x18 * stream number */
	__vo uint32_t SFCR;				/* DMA stream x FIFO control register - OFFSET 0x24 + 0x18 * stream number */
} DMA_Strm_RegDef_t;


/* DMA register */
typedef struct {
	__vo uint32_t LISR;						/* DMA low interrupt status register - OFFSET 0x00 */
	__vo uint32_t HISR;						/* DMA high interrupt status register - OFFSET 0x04 */
	__vo uint32_t LIFCR;					/* DMA low interrupt flag clear register - OFFSET 0x08 */
	__vo uint32_t HIFCR;					/* DMA high interrupt flag clear register - OFFSET 0x0C */
	__vo DMA_Strm_RegDef_t DMA_Strm[8];		/* DMA stream registers (8 streams for each DMA) - OFFSET 0x10 */
} DMA_RegDef_t;


/* RCC register */
typedef struct {
	__vo uint32_t CR;				/* RCC clock control register - OFFSET 0x00 */
	__vo uint32_t PLLCFGR;			/* RCC PLL configuration register - OFFSET 0x04 */
	__vo uint32_t CFGR;				/* RCC clock configuration register - OFFSET 0x08 */
	__vo uint32_t CIR;				/* RCC clock interrupt register - OFFSET 0x0C */
	__vo uint32_t AHB1RSTR;			/* RCC AHB1 peripheral reset register - OFFSET 0x10 */
	__vo uint32_t AHB2RSTR;			/* RCC AHB2 peripheral reset register - OFFSET 0x14 */
	__vo uint32_t AHB3RSTR;			/* RCC AHB3 peripheral reset register - OFFSET 0x18 */
	uint32_t RESERVED0;				/* RESERVED */
	__vo uint32_t APB1RSTR;			/* RCC APB1 peripheral reset register - OFFSET 0x20 */
	__vo uint32_t APB2RSTR;			/* RCC APB2 peripheral reset register - OFFSET 0x24 */
	uint32_t RESERVED1[2];			/* RESERVED */
	__vo uint32_t AHB1ENR;			/* RCC AHB1 peripheral clock enable register - OFFSET 0x30  */
	__vo uint32_t AHB2ENR;			/* RCC AHB2 peripheral clock enable register - OFFSET 0x34 */
	__vo uint32_t AHB3ENR;			/* RCC AHB3 peripheral clock enable register - OFFSET 0x38 */
	uint32_t RESERVED2;				/* RESERVED */
	__vo uint32_t APB1ENR;			/* RCC APB1 peripheral clock enable register - OFFSET 0x40 */
	__vo uint32_t APB2ENR;			/* RCC APB2 peripheral clock enable register - OFFSET 0x44 */
	uint32_t RESERVED3[2];			/* RESERVED */
	__vo uint32_t AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register - OFFSET 0x50 */
	__vo uint32_t AHB2LPENR;		/* SRCC AHB2 peripheral clock enable in low power mode register - OFFSET 0x54 */
	__vo uint32_t AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register - OFFSET 0x58 */
	uint32_t RESERVED4;				/* RESERVED */
	__vo uint32_t APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register - OFFSET 0x60 */
	__vo uint32_t APB2LPENR;		/* RCC APB2 peripheral clock enabled in low power mode register - OFFSET 0x64 */
	uint32_t RESERVED5[2];			/* RESERVED */
	__vo uint32_t BDCR;				/* RCC Backup domain control register - OFFSET 0x70 */
	__vo uint32_t CSR;				/* RCC clock control & status register - OFFSET 0x74 */
	uint32_t RESERVED6[2];			/* RESERVED */
	__vo uint32_t SSCGR;			/* RCC spread spectrum clock generation register - OFFSET 0x80 */
	__vo uint32_t PLLI2SCFGR;		/* RCC PLLI2S configuration register - OFFSET 0x84 */
	__vo uint32_t PLLSAICFGR;		/* RCC PLL configuration register - OFFSET 0x88 */
	__vo uint32_t DCKCFGR;			/* RCC Dedicated Clock Configuration Register - OFFSET 0x8C */
	__vo uint32_t CKGATENR;			/* RCC clocks gated enable register - OFFSET 0x90 */
	__vo uint32_t DCKCFGR2;			/* RCC dedicated clocks configuration register 2 - OFFSET 0x94 */
} RCC_RegDef_t;


/* SYSCFG register */
typedef struct {
	__vo uint32_t MEMRMP;			/* SYSCFG memory remap register - OFFSET 0x00 */
	__vo uint32_t PMC;				/* SYSCFG peripheral mode configuration register - OFFSET 0x04 */
	__vo uint32_t EXTICR[4];		/* SYSCFG external interrupt configuration register 1 - OFFSET 0x08 */
	uint32_t RESERVED0[2];			/* RESERVED */
	__vo uint32_t CMPCR;			/* Compensation cell control register - OFFSET 0x20 */
	uint32_t RESERVED1[2];			/* RESERVED */
	__vo uint32_t CFGR;				/* SYSCFG configuration register - OFFSET 0x2C */
} SYSCFG_RegDef_t;


/* EXTI register */
typedef struct {
	__vo uint32_t IMR;				/* Interrupt mask register - OFFSET 0x00 */
	__vo uint32_t EMR;				/* Event mask register - OFFSET 0x04 */
	__vo uint32_t RTSR;				/* Rising trigger selection register - OFFSET 0x08 */
	__vo uint32_t FTSR;				/* Falling trigger selection register - OFFSET 0x0C */
	__vo uint32_t SWIER;			/* Software interrupt event register - OFFSET 0x10 */
	__vo uint32_t PR;				/* Pending register - 0x14 */
} EXTI_RegDef_t;


/* NVIC ISER register */
typedef struct {
	__vo uint32_t ISER[8];			/* NVIC interrupt set-enable register - OFFSET 0x00 */
} NVIC_ISER_RegDef_t;


/* NVIC ICER register */
typedef struct {
	__vo uint32_t ICER[8];			/* NVIC interrupt clear-enable register - OFFSET 0x00 */
} NVIC_ICER_RegDef_t;


/* NVIC IPR register */
typedef struct {
	__vo uint32_t IPR[60];			/* NVIC interrupt priority register - OFFSET 0x00 */
} NVIC_IPR_RegDef_t;


/* NVIC SYSTCK register */
typedef struct {
	__vo uint32_t  CSR;				/* SYSTCK control and status register - OFFSET 0x00 */
	__vo uint32_t  RVR;				/* SYSTCK reload value register - OFFSET 0x04 */
	__vo uint32_t  CVR;				/* SYSTCK current value register - OFFSET 0x08 */
	__vo uint32_t  CALIB;			/* SYSTCK calibration value register - OFFSET 0x0C */
} NVIC_SYSTCK_RegDef_t;



/************* PERIPHERAL STRUCTURE MEMORY BLOCKS **********/

#define GPIOA			((GPIO_RegDef_t*)(GPIOA_BASEADDR))
#define GPIOB			((GPIO_RegDef_t*)(GPIOB_BASEADDR))
#define GPIOC			((GPIO_RegDef_t*)(GPIOC_BASEADDR))
#define GPIOD			((GPIO_RegDef_t*)(GPIOD_BASEADDR))
#define GPIOE			((GPIO_RegDef_t*)(GPIOE_BASEADDR))
#define GPIOF			((GPIO_RegDef_t*)(GPIOF_BASEADDR))
#define GPIOG			((GPIO_RegDef_t*)(GPIOG_BASEADDR))
#define GPIOH			((GPIO_RegDef_t*)(GPIOH_BASEADDR))

#define SPI1			((SPI_RegDef_t*)(SPI1_BASEADDR))
#define SPI2			((SPI_RegDef_t*)(SPI2_BASEADDR))
#define SPI3			((SPI_RegDef_t*)(SPI3_BASEADDR))
#define SPI4			((SPI_RegDef_t*)(SPI4_BASEADDR))

#define I2C1			((I2C_RegDef_t*)(I2C1_BASEADDR))
#define I2C2			((I2C_RegDef_t*)(I2C2_BASEADDR))
#define I2C3			((I2C_RegDef_t*)(I2C3_BASEADDR))

#define USART1			((USART_RegDef_t*)(USART1_BASEADDR))
#define USART2			((USART_RegDef_t*)(USART2_BASEADDR))
#define USART3			((USART_RegDef_t*)(USART3_BASEADDR))
#define UART4			((USART_RegDef_t*)(UART4_BASEADDR))
#define UART5			((USART_RegDef_t*)(UART5_BASEADDR))
#define USART6			((USART_RegDef_t*)(USART6_BASEADDR))

#define ADC1			((ADC_RegDef_t*)(ADC1_BASEADDR))
#define ADC2			((ADC_RegDef_t*)(ADC2_BASEADDR))
#define ADC3			((ADC_RegDef_t*)(ADC3_BASEADDR))
#define ADC_COMM		((ADC_Comm_RegDef_t*)(ADC_COMM_BASEADDR))

#define DMA1			((DMA_RegDef_t*)(DMA1_BASEADDR))
#define DMA2			((DMA_RegDef_t*)(DMA2_BASEADDR))

#define RCC				((RCC_RegDef_t*)(RCC_BASEADDR))

#define SYSCFG			((SYSCFG_RegDef_t*)(SYSCFG_BASEADDR))

#define EXTI			((EXTI_RegDef_t*)(EXTI_BASEADDR))


/***************** NVIC STRUCTURE MEMORY BLOCK *************/

#define NVIC_ISER		((NVIC_ISER_RegDef_t*)(NVIC_ISER0_BASEADDR))
#define NVIC_ICER		((NVIC_ICER_RegDef_t*)(NVIC_ICER0_BASEADDR))
#define NVIC_IPR		((NVIC_IPR_RegDef_t*)(NVIC_IPR0_BASEADDR))

#define NVIC_SYSTCK 	((NVIC_SYSTCK_RegDef_t*)(NVIC_SYSTCK_BASEADDR))



/************* CLOCK ENABLE MACROS **********/

/*
 * Clock enable macros for GPIO
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock enable macros for I2C
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPI
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

/*
 * Clock enable macros for USART
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macros for ADC
 */
#define ADC1_PCLK_EN() 		(RCC->APB2ENR |= (1 << 8))
#define ADC2_PCLK_EN() 		(RCC->APB2ENR |= (1 << 9))
#define ADC3_PCLK_EN() 		(RCC->APB2ENR |= (1 << 10))

/*
 * Clock enable macro for SYSCFG
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))


/************* CLOCK DISABLE MACROS **********/

/*
 * Clock disable macros for GPIO
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

/*
 * Clock disable macros for I2C
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPI
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock disable macros for USART
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macros for ADC
 */
#define ADC1_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 8))
#define ADC2_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 9))
#define ADC3_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 10))

/*
 * Clock enable macro for SYSCFG
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))



/************************* NVIC IRQ POSITIONS *****************************/

/*
 * Exception interrupt positions
 */
#define IRQ_POS_SYSTCK			-1

/*
 * EXTI lines global interrupts
 */
#define IRQ_POS_EXTI_0			6
#define IRQ_POS_EXTI_1			7
#define IRQ_POS_EXTI_2			8
#define IRQ_POS_EXTI_3			9
#define IRQ_POS_EXTI_4			10
#define IRQ_POS_EXTI_9_5		23
#define IRQ_POS_EXTI_15_10		40


/*
 * SPI global interrupts
 */
#define IRQ_POS_SPI1			35
#define IRQ_POS_SPI2			36
#define IRQ_POS_SPI3			51
#define IRQ_POS_SPI4			84


/*
 * I2C global interrupts
 */
#define IRQ_POS_I2C1_EV			31
#define IRQ_POS_I2C2_EV			33
#define IRQ_POS_I2C3_EV			72

#define IRQ_POS_I2C1_ERR		32
#define IRQ_POS_I2C2_ERR		34
#define IRQ_POS_I2C3_ERR		73


/*
 * USART/UART global interrupts
 */
#define IRQ_POS_USART1			37
#define IRQ_POS_USART2			38
#define IRQ_POS_USART3			39
#define IRQ_POS_UART4			52
#define IRQ_POS_UART5			53
#define IRQ_POS_USART6			71

/*
 * ADC global interrupts
 */
#define IRQ_POS_ADC				18

/*
 * IRQ priority macros
 */

#define IRQ_PRIORITY_1			1
#define IRQ_PRIORITY_2			2
#define IRQ_PRIORITY_3			3
#define IRQ_PRIORITY_4			4
#define IRQ_PRIORITY_5			5
#define IRQ_PRIORITY_6			6
#define IRQ_PRIORITY_7			7
#define IRQ_PRIORITY_8			8
#define IRQ_PRIORITY_9			9
#define IRQ_PRIORITY_10			10
#define IRQ_PRIORITY_11			11
#define IRQ_PRIORITY_12			12
#define IRQ_PRIORITY_13			13
#define IRQ_PRIORITY_14			14
#define IRQ_PRIORITY_15			15


/*
 * Include peripheral driver API headers
 */

#include "stm32f446xx_rcc_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_systck_driver.h"
#include "stm32f446xx_adc_driver.h"
#include "stm32f446xx_dma_driver.h"


#endif /* INC_STM32F446XX_H_ */
