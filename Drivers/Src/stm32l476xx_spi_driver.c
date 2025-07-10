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
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//configure the SPI_CR1 register
	uint32_t cr1_tempreg = 0;

	//1. configure the device mode
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//bidi mode should be cleared
		cr1_tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		//bidi mode should be set
		cr1_tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//bidi mode should be cleared
		cr1_tempreg&= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		cr1_tempreg|= (1 << SPI_CR1_RXONLY);
	}

	//3. configure the SPI serial clock speed (baud rate)
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the CPOL
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//5. configure the CPHA
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//6. configure the SSM
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	//configure the SPI_CR2 register
	uint32_t cr2_tempreg = 0;

	//1. configure the DS
	cr2_tempreg |= pSPIHandle->SPIConfig.SPI_DS << SPI_CR2_DS;

	pSPIHandle->pSPIx->CR1 = cr1_tempreg;
	pSPIHandle->pSPIx->CR2 = cr2_tempreg;
}

/*************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			-
 *
 * @param[in]		- SPI_RegDef_t - SPI register definition structure
 *
 * @retrun			- none
 *
 * @Note			- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if(pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if(pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if(pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
}

/*************************************************************************
 * @fn				- SPI_GetFlagStatus
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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if(pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @retrun			- none
 *
 * @Note			- This is a blocking call
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length)
{
    while(length > 0) // blocking
    {
        //1. wait until TXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        //2. check the 4 DS (DFF) bits in CR2 and handle for correct size

        if ((((1 << SPI_DS_NUM_BITS) - 1) & (pSPIx->CR2 >> SPI_CR2_DS)) <= SPI_DS_8BITS) {
			 // 8-bit data
			// pSPIx->DR = *pTxBuffer;
			 *((volatile uint8_t *)&pSPIx->DR) = *pTxBuffer;
			 length--; // decrement once because data is 8-bit wide
			 pTxBuffer++;
		 } else {
			 // 16-bit data
			 pSPIx->DR = *((uint16_t*)pTxBuffer); //typecast to 16-bits
			 length -= 2; // decrement twice for 2x 8-bit data
			 (uint16_t*)pTxBuffer++; //typecast to 16-bits
		 }
    }
}

/*void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
	while(len > 0) {
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF(CRCL) bit in CR1
		if(pSPIx->SR & (1 << SPI_CR1_CRCL)) {
			//16 bit DFF
			//1. load data into the DR(data register)
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len--;
			len--;
			//2. point to next data item
			(uint16_t*) pTxBuffer++;
		} else {
			//8 bit DFF
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;
		}
	}
}*/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		} else {
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}
