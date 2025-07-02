/*
 * stm32l476xx_spi_driver.c
 *
 *  Created on: Jul 1, 2025
 *      Author: Kivilak Chathuranga
 */

#include "stm32l476xx_spi_driver.h"

/*
 * Peripheral Clock setup
 */

/*************************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @retrun			- none
 *
 * @Note			- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if(pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}


/*************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @retrun			- none
 *
 * @Note			- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	//configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DFF(CRCL)
	tempreg |= pSPIHandle->SPIConfig.SPI_CRCL << SPI_CR1_CRCL;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}
