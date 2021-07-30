/*
 * stm32f407xx_GPIO_Driver.h
 *
 *  Created on: 29-Jul-2021
 *      Author: DHYEY
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/**
 * PinConfig-> Configuration of pins of a specific port
 */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/**
 * Port Handler-> Will handle which port is selected and will react accordingly
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfiguration;
}GPIO_Handle_t;

/**
 * Macros of Pin Numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/**
 * Macros of Pin Mode
 */
#define GPIO_PIN_MODE_IN 		0
#define GPIO_PIN_MODE_OP 		1
#define GPIO_PIN_MODE_ALTFUN 	2
#define GPIO_PIN_MODE_ANALOG	3

/**
 * Rising edge or falling edge interrupt.
 * RT = Rising edge
 * FT = Falling edge
 * RTFT = Nothing Specific
 */
#define GPIO_PIN_MODE_RT		4
#define GPIO_PIN_MODE_FT		5
#define GPIO_PIN_MODE_RTFT		6

/**
 * Macros for Speed
 */
#define GPIO_PIN_LOSPEED		0
#define GPIO_PIN_MEDSPEED		1
#define GPIO_PIN_HISPEED		2
#define GPIO_PIN_VHISPEED		3

/**
 * Macros for Output Pin Type
 * Push-Pull or Open-Drain
 */
#define GPIO_PIN_PUSH_PULL		0
#define GPIO_PIN_OPEN_DRAIN		1

/**
 * Macros for Push_Pull resistor control
 */
#define GPIO_PIN_NO_PUPD		0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/**
 * Initialization and Deactivate
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * Peripheral Clock start and stop
 */
void GPIO_PeriClock(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis);

/**
 * Data Read and Write
 */
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDn);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
