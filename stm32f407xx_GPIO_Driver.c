/*
 * stm32f407xx_GPIO_Driver.c
 *
 *  Created on: 29-Jul-2021
 *      Author: DHYEY
 */
#include "stm32f407xx_GPIO_Driver.h"

/**
 * @brief: Initialize the GPIO Peripheral Port
 * @Param1: The GPIO Handler
 * @ReturnValue: None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	if(pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode < GPIO_PIN_MODE_ANALOG)
	{
		/**
		 * Mode Initialization
		 */
		temp = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 <<(2*pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode == GPIO_PIN_MODE_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode == GPIO_PIN_MODE_RT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfiguration.GPIO_PinMode == GPIO_PIN_MODE_RTFT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
		}
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber%4;
		uint8_t portcode = PORT_IN_INTERRUPT(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber;
	}
	/**
	 * Speed Initialization
	 */
	temp = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 <<(2*pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/**
	 * Pin Output Type
	 */
	temp = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinOpType << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/**
	 * Push Pull Resistor Mode
	 */
	temp = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 <<(2*pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/**
	 * Alternate Function
	 */
	if(pGPIOHandle->GPIO_PinConfiguration.GPIO_PinAltFunMode == SET)
	{
		uint32_t temp1 = 0, temp2 = 0;
		temp1 = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber/8);
		temp2 = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinNumber%8);
		temp = (pGPIOHandle->GPIO_PinConfiguration.GPIO_PinAltFunMode) << (4*temp2);
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
		temp = 0;
	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_RESET();
	}
}

/**
 * Peripheral Clock start and stop
 */
void GPIO_PeriClock(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DN();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DN();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DN();
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_DN();
		}
	}
}

/**
 * Data Read and Write
 */
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDn)
{
	if(EnOrDn == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber < 64 && IRQNumber > 31)
		{
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}
		else if(IRQNumber < 96 && IRQNumber >= 64)
		{
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber < 64 && IRQNumber > 31)
		{
			*NVIC_ICER1 |= (1 << IRQNumber%32);
		}
		else if(IRQNumber < 96 && IRQNumber >= 64)
		{
			*NVIC_ICER2 |= (1 << IRQNumber%64);
		}
	}
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
