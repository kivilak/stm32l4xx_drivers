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
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR; //macro value is 2

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//bidi mode should be cleared
		cr1_tempreg &= ~(1 << SPI_CR1_BIDIMODE); //macro value is 15
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		//bidi mode should be set
		cr1_tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//bidi mode should be cleared
		cr1_tempreg&= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		cr1_tempreg|= (1 << SPI_CR1_RXONLY); //macro value is 10
	}

	//3. configure the SPI serial clock speed (baud rate)
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR; //macro value is 3

	//4. configure the CPOL
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL; //macro value is 1

	//5. configure the CPHA
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA; //macro value is 0

	//6. configure the SSM
	cr1_tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM; //macro value is 9

	//configure the SPI_CR2 register
	uint32_t cr2_tempreg = 0;

	//1. configure the DS
	cr2_tempreg |= pSPIHandle->SPIConfig.SPI_DS << SPI_CR2_DS; //macro value is 8

	pSPIHandle->pSPIx->CR1 = cr1_tempreg;
	pSPIHandle->pSPIx->CR2 = cr2_tempreg;
}

/*************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			-
 *
 * @param[in]		- SPI_RegDef_t (SPI register definition structure)
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
 * @param[in]		- SPI_RegDef_t (SPI register definition structure)
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
 * @param[in]		- SPI_RegDef_t (SPI register definition structure)
 * @param[in]		- uint8_t (Type casted data)
 * @param[in]		- uint32_t (length of the data)
 *
 * @retrun			- none
 *
 * @Note			- This API send the SPI data
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
			 pTxBuffer += 2;
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


/*************************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			-
 *
 * @param[in]		- SPI_RegDef_t (SPI register definition structure)
 * @param[in]		-
 *
 * @retrun			- none
 *
 * @Note			- This is a blocking call
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{
	while(length > 0) // blocking
	{
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. check the 4 DS (DFF) bits in CR2 and handle for correct size

		if (((pSPIx->CR2 >> SPI_CR2_DS) & 0xF) <= SPI_DS_8BITS) {
			// 8-bit data
			// pSPIx->DR = *pTxBuffer;
			// load the data from DR to buffer
			*pRxBuffer = (uint8_t) pSPIx->DR;
			length--; // decrement once because data is 8-bit wide
			pRxBuffer++;
		 } else {
			// 16-bit data
			 *((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR; //typecast to 16-bits
			length -= 2; // decrement twice for 2x 8-bit data
			pRxBuffer += 2; //typecast to 16-bits
		 }
	}
}

/*************************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			-
 *
 * @param[in]		- SPI_RegDef_t (SPI register definition structure)
 * @param[in]		- ENABLE or DISABLE
 *
 * @retrun			- none
 *
 * @Note			- This API enable or disable the SPI peripheral
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*************************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			-
 *
 * @param[in]		- SPI_RegDef_t (SPI register definition structure)
 * @param[in]		- ENABLE or DISABLE
 *
 * @retrun			- none
 *
 * @Note			- This API enable or disable the SPI SSI(Internal slave select)
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*************************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			-
 *
 * @param[in]		- SPI_RegDef_t (SPI register definition structure)
 * @param[in]		- ENABLE or DISABLE
 *
 * @retrun			- none
 *
 * @Note			- This API enable or disable the SPI SSOE(SS output enable)
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
