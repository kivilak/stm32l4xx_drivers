/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jul 2, 2025
 *      Author: Kivilak Chathuranga
 */

#include <string.h>
#include "stm32l476xx.h"

/*
* PB15 --> SPI2_MOSI
* PB14 --> SPI2_MISO
* PB13 --> SPI2_SCLK
* PB12 --> SPI2_NSS
* ALT function mode 5
*/

void SPI2_GPIOInits(void) {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode =5;
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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

}

void SPI2_Inits(void) {
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;		// Generate SCLK of 8MHz
	SPI2Handle.SPIConfig.SPI_CRCL = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;		// Software slave management enable for NSS pin

	SPI_Init(&SPI2Handle);
}

int main(void) {
	char user_data[] = "Hello World";

	SPI2_GPIOInits(); // Initialize the GPIO pins to behave as SPI2 pins
	SPI2_Inits(); // Initialize the SPI2 peripheral parameters
	SPI_PeripheralControl(SPI2, ENABLE); // Enable the SPI2 peripheral

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while(1);

	return 0;
}
