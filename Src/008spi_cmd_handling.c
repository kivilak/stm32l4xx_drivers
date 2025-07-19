/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Jul 19, 2025
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

//commands
#define CMD_LED_CTRL		0x50
#define CMD_SENSOR_READ		0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

#define LED_ON		1
#define LED_OFF		0

//Arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3

//Arduino led pin
#define LED_PIN			2

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

uint8_t SPI_VerifyResponse(uint8_t ack_byte) {
	if(ack_byte == 0xF5) {
		return 1;
	}

	return 0;
}

int main(void)
{
	uint8_t dummyWrtie = 0xff;
	uint8_t dummyRead;
	uint8_t ackByte;
	uint8_t args[2];

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
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		/* Led control command */
		uint8_t cmdcode = CMD_LED_CTRL;

		//send command
		SPI_SendData(SPI2, &cmdcode, 1);

		//read dummy data to clear off the RXNE
		//SPI_ReceiveData(SPI2, &dummyRead, 1);

		//send dummy bits(1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrtie, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		/*if(SPI_VerifyResponse(ackByte)) {
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2, args, 2);
		}*/

		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)); //checks the SPI is not busy
		SPI_PeripheralControl(SPI2, DISABLE); //disable the SPI2 peripheral
	}

	return 0;

}
