/*
 * stm32l476xx_gpio_driver.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Kivilak Chathuranga
 */

#include "stm32l476xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @retrun			- none
 *
 * @Note			- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init & De-init
 */

/*************************************************************************
 * @fn				- GPIO_Init
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0; //temp. register

	//Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // each pin have 2bit there for 2bit shifting needed
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));		//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;		//setting
	} else {
		//to be sure that this pin is configure as an input
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=  GPIO_MODE_IT_FT) {
			//configure falling trigger register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT) {
			//configure rising trigger register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT) {
			//configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//configure the GPIO port selection in SYSCFG_EXTICR
			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode =  GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		//configure the alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; 	//get the pin in AFRL or AFRH (0, 1)
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/*************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			-
 *
 * @param[in]		- GPIO_RegDef_t - GPIO register definition structure
 *
 * @retrun			- none
 *
 * @Note			- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if(pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if(pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*
 * Data read & write
 */

/*************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @retrun			- 0 or 1
 *
 * @Note			- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @retrun			- 0 or 1
 *
 * @Note			- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

/*************************************************************************
 * @fn				- GPIO_WriteToOutputPin
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
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if(Value == GPIO_PIN_SET) {
		//write 1 to output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*************************************************************************
 * @fn				- GPIO_WriteToOutputPort
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
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/*************************************************************************
 * @fn				- GPIO_ToggleOutputPin
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
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration & ISR handling
 */

/*************************************************************************
 * @fn				- GPIO_IRQConfig
 *
 * @brief			- GPIO interrupt request configuration
 *
 * @param[in]		- IRQNumber
 * @param[in]		- IRQPriority
 * @param[in]		- ENorDi (ENABLE or DISABLE)
 *
 * @retrun			- none
 *
 * @Note			- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(IRQNumber <= 31) {
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64) {
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber >= 64 && IRQNumber < 96) {
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if(IRQNumber <= 31) {
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64) {
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber >= 64 && IRQNumber < 96) {
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*************************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			-
 *
 * @param[in]		- IRQPriority
 *
 * @retrun			- none
 *
 * @Note			- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	//ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			-
 *
 * @param[in]		- PinNumber
 *
 * @retrun			- none
 *
 * @Note			- none
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)) {
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
