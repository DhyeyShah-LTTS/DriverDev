/*
 * stm32f407xx.h
 *
 *  Created on: 29-Jul-2021
 *      Author: DHYEY
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include "stdint.h"
#define __vo volatile

/*-----------------------------------------NVIC SET ENABLE ADDRESSESS----------------------------------*/
#define NVIC_ISER0			(__vo uint32_t*)0xE000E100U
#define NVIC_ISER1			(__vo uint32_t*)(NVIC_ISER0 + 0x04U)
#define NVIC_ISER2			(__vo uint32_t*)(NVIC_ISER0 + 0x08U)
#define NVIC_ISER3			(__vo uint32_t*)(NVIC_ISER0 + 0x0CU)
#define NVIC_ISER4			(__vo uint32_t*)(NVIC_ISER0 + 0x10U)
#define NVIC_ISER5			(__vo uint32_t*)(NVIC_ISER0 + 0x14U)
#define NVIC_ISER6			(__vo uint32_t*)(NVIC_ISER0 + 0x18U)
#define NVIC_ISER7			(__vo uint32_t*)(NVIC_ISER0 + 0x1CU)

/*-----------------------------------------NVIC CLEAR ENABLE ADDRESSESS----------------------------------*/
#define NVIC_ICER0			(__vo uint32_t*)0XE000E180U
#define NVIC_ICER1			(__vo uint32_t*)(NVIC_ICER0 + 0x04U)
#define NVIC_ICER2			(__vo uint32_t*)(NVIC_ICER0 + 0x08U)
#define NVIC_ICER3			(__vo uint32_t*)(NVIC_ICER0 + 0x0CU)
#define NVIC_ICER4			(__vo uint32_t*)(NVIC_ICER0 + 0x10U)
#define NVIC_ICER5			(__vo uint32_t*)(NVIC_ICER0 + 0x14U)
#define NVIC_ICER6			(__vo uint32_t*)(NVIC_ICER0 + 0x18U)
#define NVIC_ICER7			(__vo uint32_t*)(NVIC_ICER0 + 0x1CU)

/*-----------------------------------------NVIC PRIORITY REGISTER ADDRESSESS----------------------------------*/
/**
 * Starts from this base address and has many priority registers.
 * 60 of 4 bytes each
 * end address is 0xE000E4EFU
 * No of priority bits that are reserved is first 4 bits of each section in IPRx.
 * Refer Generic User Guide - Cortex-m4.pdf.
 */
#define NVIC_PRI_BASEADDR	(__vo uint32_t*)0xE000E400U
#define NO_OF_PRIBITS_NOTUSED		4

/*-----------------------------------------BASIC ELEMENT BASE ADDRESSESS----------------------------------*/

#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define SRAM_BASEADDR		SRAM1_BASEADDR

/*-----------------------------------------BUSES BASE ADDRESSESS----------------------------------*/

#define APB1_BASEADDR		0x40000000U
#define APB2_BASEADDR		0x40010000U
#define AHB1_BASEADDR		0x40020000U
#define AHB2_BASEADDR		0x50000000U
#define AHB3_BASEADDR		0xA0000000U

/*-----------------------------------------BASIC ELEMENT ON APB1 BUS ADDRESSESS----------------------------------*/

#define SP12_ADDR			((APB1_BASEADDR) + (0x3800U))
#define SP13_ADDR			((APB1_BASEADDR) + (0x3C00U))
#define USART2_ADDR 		((APB1_BASEADDR) + (0x4400U))
#define USART3_ADDR 		((APB1_BASEADDR) + (0x4800U))
#define UART4_ADDR 			((APB1_BASEADDR) + (0x4C00U))
#define UART5_ADDR 			((APB1_BASEADDR) + (0x5000U))
#define I2C1_ADDR 			((APB1_BASEADDR) + (0x5400U))
#define I2C2_ADDR 			((APB1_BASEADDR) + (0x5800U))
#define I2C3_ADDR 			((APB1_BASEADDR) + (0x5C00U))

/*-----------------------------------------BASIC ELEMENT ON APB2 BUS ADDRESSESS----------------------------------*/

#define USART1_ADDR			((APB2_BASEADDR) + (0x1000U))
#define USART6_ADDR			((APB2_BASEADDR) + (0x1400U))
#define SPI1_ADDR			((APB2_BASEADDR) + (0x3000U))
//#define SPI4_ADDR			((APB2_BASEADDR) + (0x3400U))
#define SYSCFG_ADDR			((APB2_BASEADDR) + (0x3800U))
#define EXTI_ADDR			((APB2_BASEADDR) + (0x3C00U))

/*-----------------------------------------BASIC ELEMENT ON AHB1 BUS ADDRESSESS----------------------------------*/

#define GPIOA_ADDR 			((AHB1_BASEADDR) + (0x0000U))
#define GPIOB_ADDR 			((AHB1_BASEADDR) + (0x0400U))
#define GPIOC_ADDR 			((AHB1_BASEADDR) + (0x0800U))
#define GPIOD_ADDR 			((AHB1_BASEADDR) + (0x0C00U))
#define GPIOE_ADDR 			((AHB1_BASEADDR) + (0x1000U))
#define GPIOF_ADDR 			((AHB1_BASEADDR) + (0x1400U))
#define GPIOG_ADDR 			((AHB1_BASEADDR) + (0x1800U))
#define GPIOH_ADDR 			((AHB1_BASEADDR) + (0x1C00U))
#define GPIOI_ADDR 			((AHB1_BASEADDR) + (0x2000U))
//#define GPIOJ_ADDR 		((AHB1_BASEADDR) + (0x2400U))
//#define GPIOK_ADDR 		((AHB1_BASEADDR) + (0x2800U))
#define RCC_ADDR 			((AHB1_BASEADDR) + (0x3800U))

/*-----------------------------------------STRUCT MACROS FOR GPIO REGISTERS----------------------------------*/

typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;

/**
 * type casting the base address of GPIOA port A to a structure.
 * and that structure will give the values to GPIO_RegDef_t
 */
#define GPIOA				(GPIO_RegDef_t*)GPIOA_ADDR
#define GPIOB				(GPIO_RegDef_t*)GPIOB_ADDR
#define GPIOC				(GPIO_RegDef_t*)GPIOC_ADDR
#define GPIOD				(GPIO_RegDef_t*)GPIOD_ADDR
#define GPIOE				(GPIO_RegDef_t*)GPIOE_ADDR
#define GPIOF				(GPIO_RegDef_t*)GPIOF_ADDR
#define GPIOG				(GPIO_RegDef_t*)GPIOG_ADDR
#define GPIOH				(GPIO_RegDef_t*)GPIOH_ADDR
#define GPIOI				(GPIO_RegDef_t*)GPIOI_ADDR
//#define GPIOJ				(GPIO_RegDef_t*)GPIOJ_ADDR
//#define GPIOK				(GPIO_RegDef_t*)GPIOK_ADDR

/*-----------------------------------------STRUCT MACROS FOR RCC REGISTERS----------------------------------*/

typedef struct {
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB3ENR;
	uint32_t RESERVED2;
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t RESERVED6[2];
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

/**
 * Accessing RCC registers using structure
 */
#define RCC 				((RCC_RegDef_t*) RCC_ADDR)

/**
 * Clock enable and disable of the peripherals on AHB1 bus.
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

#define GPIOA_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DN()		(RCC->AHB1ENR &= ~(1<<8))

/**
 * Clock enable and disable of the peripherals on APB1 bus.
 */
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1<<20))
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

#define SPI2_PCLK_DN()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DN()		(RCC->APB1ENR &= ~(1<<15))
#define USART2_PCLK_DN()	(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DN()	(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DN()		(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DN()		(RCC->APB1ENR &= ~(1<<20))
#define I2C1_PCLK_DN()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DN()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DN()		(RCC->APB1ENR &= ~(1<<23))

/**
 * Clock enable and disable of the peripherals on APB2 bus.
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1<<4))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1<<5))
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))

#define USART1_PCLK_DN()	(RCC->APB2ENR &= ~(1<<4))
#define USART6_PCLK_DN()	(RCC->APB2ENR &= ~(1<<5))
#define SPI1_PCLK_DN()		(RCC->APB2ENR &= ~(1<<12))
#define SYSCFG_PCLK_DN()	(RCC->APB2ENR &= ~(1<<14))

/**
 * GPIO Resetting using AHB1RSTR
 */
#define GPIOA_RESET() 		do{ (RCC->AHB1RSTR |= (1<<0));		(RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_RESET()		do{ (RCC->AHB1RSTR |= (1<<1));		(RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_RESET() 		do{ (RCC->AHB1RSTR |= (1<<2));		(RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_RESET() 		do{ (RCC->AHB1RSTR |= (1<<3));		(RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_RESET() 		do{ (RCC->AHB1RSTR |= (1<<4));		(RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_RESET()		do{ (RCC->AHB1RSTR |= (1<<5));		(RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_RESET() 		do{ (RCC->AHB1RSTR |= (1<<6));		(RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_RESET() 		do{ (RCC->AHB1RSTR |= (1<<7));		(RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_RESET() 		do{ (RCC->AHB1RSTR |= (1<<8));		(RCC->AHB1RSTR &= ~(1<<8)); }while(0)

/**
 * Structure for EXTI register.
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

#define EXTI			((EXTI_RegDef_t*)EXTI_ADDR)
/**
 * System configuration for external interrupts.
 * SYSCFG_EXTICRx (x == 0,1,2..,15) Gives the pin where EXT INT is enabled.
 * Which GPIOx (x == A,B,C..,I) is selected.
 */
#define PORT_IN_INTERRUPT(x)	((x==GPIOA)?0:\
								 (x==GPIOB)?1:\
								 (x==GPIOC)?2:\
								 (x==GPIOD)?3:\
								 (x==GPIOE)?4:\
								 (x==GPIOF)?5:\
								 (x==GPIOG)?6:\
								 (x==GPIOH)?7:\
								 (x==GPIOI)?8:0)

/**
 * Structure of SYSCFG register.
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t RESERVED[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_ADDR)
/**
 * Add other macros like SPI, I2C, USART
 * To be done.
 */

/**
 * IRQ numbers related to EXTI
 */
#define IRQ_POSITION_EXTI0		6
#define IRQ_POSITION_EXTI1		7
#define IRQ_POSITION_EXTI2		8
#define IRQ_POSITION_EXTI3		9
#define IRQ_POSITION_EXTI4		10
#define IRQ_POSITION_EXTI9_5	23
#define IRQ_POSITION_EXTI15_10	40

/**
 * General Macros
 */
#define ENABLE 				1
#define DISABLE				0
#define SET					1
#define RESET 				0

#include "stm32f407xx_GPIO_Driver.h"

#endif /* INC_STM32F407XX_H_ */
