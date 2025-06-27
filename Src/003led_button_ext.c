/*
 * 001LED_toggle.c
 *
 *  Created on: Jun 27, 2025
 *      Author: Kivilak Chathuranga
 */

#include "stm32l476xx.h"

#define BTN_PRESSED DISABLE

void delay() {
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t GPIOLed;

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; 	// you can also use GPIO_OP_TYPE_OD to Open drain output but you need to use external pull up resistor
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOLed);

	GPIO_Handle_t GPIOButton;

	GPIOButton.pGPIOx = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	//GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOButton);

	while(1) {
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_7) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
		}
	}

	return 0;
}
