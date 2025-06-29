/*
 * 001LED_toggle.c
 *
 *  Created on: Jun 27, 2025
 *      Author: Kivilak Chathuranga
 */

#include <string.h>
#include "stm32l476xx.h"

#define BTN_PRESSED DISABLE

void delay() {
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t GPIOLed, GPIOButton;
	memset(&GPIOLed, 0, sizeof(GPIOLed));
	memset(&GPIOButton, 0, sizeof(GPIOButton));

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; 	// you can also use GPIO_OP_TYPE_OD to Open drain output but you need to use external pull up resistor
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOLed);

	GPIOButton.pGPIOx = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOButton);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRO_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_7);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
}
