#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i=0;i<50000;i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfiguration.GPIO_PinMode = GPIO_PIN_MODE_OP;
	GpioLed.GPIO_PinConfiguration.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfiguration.GPIO_PinOpType = GPIO_PIN_PUSH_PULL;
	GpioLed.GPIO_PinConfiguration.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GpioLed.GPIO_PinConfiguration.GPIO_PinSpeed = GPIO_PIN_MEDSPEED;

	GPIO_PeriClock(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfiguration.GPIO_PinMode = GPIO_PIN_MODE_FT;
	GpioBtn.GPIO_PinConfiguration.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfiguration.GPIO_PinSpeed = GPIO_PIN_MEDSPEED;
	GpioBtn.GPIO_PinConfiguration.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClock(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	GPIO_WriteToPin(GPIOD, GPIO_PIN_NO_12, DISABLE);
	GPIO_IRQInterruptConfig(IRQ_POSITION_EXTI0, ENABLE);

	while(1);
//	{
//		GPIO_TogglePin(GPIOD, GPIO_PIN_NO_12);
//		delay();
//	}
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_TogglePin(GPIOD, GPIO_PIN_NO_12);
}
