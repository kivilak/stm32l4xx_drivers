/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Jul 18, 2025
 *      Author: Kivilak Chathuranga
 */

#include<string.h>
#include "stm32l476xx.h"

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void delay() {
	for(uint32_t i = 0; i < 500000/2; i++);
}

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);


}

void SPI2_Init(void)
{

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV128;//generates sclk of 2MHz
	SPI2handle.SPIConfig.SPI_DS = SPI_DS_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //software slave management disabled for NSS pin

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOButton;

	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOButton);
}

int main(void)
{
	char user_data[] = "Hello World!";

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Init();

	GPIO_ButtonInit();

	/*
	 * making SSOE 1 does NSS output enable
	 * when SPE 1, NSS will be pull to low
	 * when SPE 0, NSS will be high
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1) {
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//send length info
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		//to send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)); //checks the SPI is not busy
		SPI_PeripheralControl(SPI2, DISABLE); //disable the SPI2 peripheral
	}

	return 0;

}


