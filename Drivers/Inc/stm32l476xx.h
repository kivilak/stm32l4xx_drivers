/*
 * stm32l476xx.h
 *
 *  Created on: Jun 22, 2025
 *      Author: Kivilak Chathuranga
 */
#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_

#include <stdint.h>

#define __vo volatile

/*	base addresses of Flash and SRAM memories	*/

#define FLASH_BASEADDR 	0x08000000U 	//	base address of flash memory
#define SRAM1_BASEADDR 	0x20000000U		//	base address of SRAM1
#define SRAM2_BASEADDR	0x20018000U		//	base address of SRAM2
#define ROM				0x1FFF0000U		//	base address of ROM
#define SRAM 			SRAM1_BASEADDR	// starting address of SRAM

/*	base addresses of BUS	*/

#define PERIPH_BASEADDR	0x4000 0000U
#define APB1_BASEADDR	PERIPH_BASEADDR
#define APB2_BASEADDR	0x40010000U
#define AHB1_BASEADDR 	0x40020000U
#define AHB2_BASEADDR 	0x48000000U

/*	RCC	*/

#define RCC_BASEADDR	(AHB1_BASEADDR + 0x1000)

/*	base addresses of peripherals in AHB2	*/

#define GPIOA_BASEADDR	(AHB2_BASEADDR + 0x0000)
#define GPIOB_BASEADDR	(AHB2_BASEADDR + 0x0400)
#define GPIOC_BASEADDR	(AHB2_BASEADDR + 0x0800)
#define GPIOD_BASEADDR	(AHB2_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR	(AHB2_BASEADDR + 0x1000)
#define GPIOF_BASEADDR	(AHB2_BASEADDR + 0x1400)
#define GPIOG_BASEADDR	(AHB2_BASEADDR + 0x1800)
#define GPIOH_BASEADDR	(AHB2_BASEADDR + 0x1C00)

/*	base addresses of peripherals in APB1	*/

#define I2C1_BASEADDR	(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR	(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR	(APB1_BASEADDR + 0x5C00)
#define SPI2_BASEADDR	(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR	(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR	(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR	(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR	(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR	(APB1_BASEADDR + 0x5000)

/*	base addresses of peripherals in APB2	*/

#define EXTI_BASEADDR	(APB2_BASEADDR + 0x0400)
#define SYSCFG_BASEADDR	(APB2_BASEADDR + 0x0000)
#define SPI1_BASEADDR	(APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR	(APB2_BASEADDR + 0x3800)

/************************************************************************/
/*	Peripheral register definition structures	*/

typedef struct {
	__vo uint32_t MODER;		// GPIO port mode register
	__vo uint32_t OTYPER;		// GPIO port output type register
	__vo uint32_t OSPEEDR;		// GPIO port output speed register
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register
	__vo uint32_t IDR;			// GPIO port input data register
	__vo uint32_t ODR;			// GPIO port output data register
	__vo uint32_t BSRR;			// GPIO port bit set/reset register
	__vo uint32_t LCKR;			// GPIO port configuration lock register
	__vo uint32_t AFRL;			// GPIO alternate function low register
	__vo uint32_t AFRH;			// GPIO alternate function high register
	__vo uint32_t BRR;			// GPIO port bit reset register
	__vo uint32_t ASCR;			// GPIO port analog switch control register
} GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;			// Clock control register
	__vo uint32_t ICSCR;		// Internal clock sources calibration register
	__vo uint32_t CFGR;			// Clock configuration register
	__vo uint32_t PLLCFGR;		// PLL configuration register
	__vo uint32_t PLLSAI1CFGR;	// PLLSAI1 configuration register
	__vo uint32_t PLLSAI2CFGR;	// PLLSAI2 configuration register
	__vo uint32_t CIER;			// Clock interrupt enable register
	__vo uint32_t CIFR;			// Clock interrupt flag register
	__vo uint32_t CICR;			// Clock interrupt clear register
	uint32_t      RESERVED0;
	__vo uint32_t AHB1RSTR;		// AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;		// AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;		// AHB3 peripheral reset register
	uint32_t      RESERVED1;
	__vo uint32_t APB1RSTR1;	// APB1 peripheral reset register 1
	__vo uint32_t APB1RSTR2;	// APB1 peripheral reset register 2
	__vo uint32_t APB2RSTR;		// APB2 peripheral reset register
	uint32_t      RESERVED2;
	__vo uint32_t AHB1ENR;		// AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;		// AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;		// AHB3 peripheral clock enable register
	uint32_t      RESERVED3;
	__vo uint32_t APB1ENR1;		// APB1 peripheral clock enable register 1
	__vo uint32_t APB1ENR2;		// APB1 peripheral clock enable register 2
	__vo uint32_t APB2ENR;		// APB2 peripheral clock enable register
	uint32_t      RESERVED4;
	__vo uint32_t AHB1SMENR;	// AHB1 peripheral clocks enable in Sleep and Stop modes register
	__vo uint32_t AHB2SMENR;	// AHB2 peripheral clocks enable in Sleep and Stop modes register
	__vo uint32_t AHB3SMENR;	// AHB3 peripheral clocks enable in Sleep and Stop modes register
	uint32_t      RESERVED5;
	__vo uint32_t APB1SMENR1;	// APB1 peripheral clocks enable in Sleep and Stop modes register 1
	__vo uint32_t APB1SMENR2;	// APB1 peripheral clocks enable in Sleep and Stop modes register 2
	__vo uint32_t APB2SMENR;	// APB2 peripheral clocks enable in Sleep and Stop modes register
	uint32_t      RESERVED6;
	__vo uint32_t CCIPR;		// Peripherals independent clock configuration register
	uint32_t      RESERVED7;
	__vo uint32_t BDCR;			// Backup domain control register
	__vo uint32_t CSR;			// Control/status register
	__vo uint32_t CRRCR;		// Clock recovery RC register
	__vo uint32_t CCIPR2;		// Peripherals independent clock configuration register
} RCC_RegDef_t;

/************************************************************************/
/*	peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)	*/

#define GPIOA 		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)

/************************************************************************/
/*	Clock enable Macros for GPIOx peripherals	*/

#define GPIOA_PCLK_EN()	(RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB2ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB2ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB2ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB2ENR |= (1 << 7))

/************************************************************************/
/*	Clock enable Macros for I2Cx peripherals	*/

#define I2C1_PCLK_EN()	(RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR1 |= (1 << 23))

/************************************************************************/
/*	Clock enable Macros for SPIx peripherals	*/

#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	(RCC->APB1ENR1 |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR1 |= (1 << 15))

/************************************************************************/
/*	Clock enable Macros for USARTx peripherals	*/

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()	(RCC->APB1ENR1 |= (1 << 17))
#define USART3_PCLK_EN()  	(RCC->APB1ENR1 |= (1 << 18))
#define UART4_PCLK_EN()  	(RCC->APB1ENR1 |= (1 << 19))
#define UART5_PCLK_EN()  	(RCC->APB1ENR1 |= (1 << 20))

/************************************************************************/
/*	Clock enable Macros for SYSCFG peripherals	*/

#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1 << 0))

/************************************************************************/
/*	Clock disable Macros for GPIOx peripherals	*/

#define GPIOA_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 7))

/*	Clock disable Macros for I2Cx peripherals	*/

#define I2C1_PCLK_DI()	(RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC->APB1ENR1 &= ~(1 << 23))

/************************************************************************/
/*	Clock disable Macros for SPIx peripherals	*/

#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()	(RCC->APB1ENR1 &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR1 &= ~(1 << 15))

/************************************************************************/
/*	Clock disable Macros for USARTx peripherals	*/

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()	(RCC->APB1ENR1 &= ~(1 << 17))
#define USART3_PCLK_DI()  	(RCC->APB1ENR1 &= ~(1 << 18))
#define UART4_PCLK_DI()  	(RCC->APB1ENR1 &= ~(1 << 19))
#define UART5_PCLK_DI()  	(RCC->APB1ENR1 &= ~(1 << 20))

/************************************************************************/
/*	Clock disable Macros for SYSCFG peripherals	*/

#define SYSCFG_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 0))

//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32L476XX_H_ */
