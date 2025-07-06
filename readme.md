<h1 align="center" style="color:white;">🔧 STM32L476xx Driver Development</h1>

<p align="center">
  <img src="https://img.shields.io/badge/STM32L476xx-Driver-blue?style=flat-square">
  <img src="https://img.shields.io/badge/Cortex--M4-ARM--Based-blueviolet?style=flat-square">
  <img src="https://img.shields.io/badge/Bare--Metal-Development-green?style=flat-square">
  <img src="https://img.shields.io/badge/C99-Standard-orange?style=flat-square">
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=flat-square">
</p>

> 🚀 **Low-level peripheral driver development for the STM32L476xx microcontroller** (ARM Cortex-M4 core) without using HAL or CMSIS drivers. Built from scratch for maximum control and educational value.

---

## 📌 Project Overview

This project is a **bare-metal embedded systems project** focused on writing peripheral drivers for the **STM32L476xx** MCU. The goal is to understand microcontroller peripherals at the **register level** and develop efficient, production-ready drivers without relying on vendor-provided abstractions.

### 🎯 Objectives
- **Educational**: Learn how STM32 peripherals work at the hardware level
- **Performance**: Optimize for speed and memory efficiency
- **Modularity**: Each driver is independent and reusable
- **Professional**: Industry-standard coding practices and documentation

---

## 🧩 Features

-  **GPIO Driver** - Input/Output, Pull-up/down, Alternate functions, Speed control
-  **Timer Driver** - Basic/Advanced timers, PWM generation, Input capture
-  **USART/UART Driver** - Polling & Interrupt-based communication, DMA support
-  **External Interrupts (EXTI)** - Edge/Level triggered, Priority management
-  **NVIC Configuration** - Interrupt priority grouping and management
-  **RCC Driver** - System/Peripheral clock management, PLL configuration
-  **SysTick Timer** - Precise delays and system tick generation
-  **SPI Driver** - Master/Slave mode, Full-duplex communication
-  **I2C Driver** - Master mode, 7/10-bit addressing, Clock stretching
-  **ADC Driver** - Single/Continuous conversion, Multi-channel sampling
-  **PWM Driver** - Multi-channel PWM with dead-time control
-  **Flash Memory Access** - Read/Write/Erase operations (Optional)

---

## ✅ Implemented Drivers

| Peripheral | File(s)                              | Status        | Description                              | Power Mode |
| ---------- | ------------------------------------ | ------------- | ---------------------------------------- | ---------- |
| GPIO       | `stm32l476xx_gpio_driver.c/.h`       | ✅ Complete   | Input, output, alternate, pull-up/down   | Low Power  |
| SPI        | `stm32l476xx_spi_driver.c/.h`        | ✅ Complete   | Master mode, 8/16-bit, polling TX/RX     | Standard   |
| I2C      | `stm32l476xx_i2c_driver.c/.h`      | ⏳ Planned    | Master mode communication                | Low Power  |
| USART      | `stm32l476xx_usart_driver.c/.h`      | ❌ Not Started  | Polling + Interrupt modes                | Low Power  |

**Legend**: ✅ Complete | ⏳ Planned (Next) | ❌ Not Started

---

## 📁 Project Structure

```bash
stm32l4xx_drivers/
├── 📁 Drivers/
│   ├── 📁 Inc/                              # Custom driver headers
│   │   ├── 📄 stm32l476xx_gpio_driver.h     # GPIO driver interface ✅
│   │   ├── 📄 stm32l476xx_spi_driver.h      # SPI driver interface ✅
│   │   ├── 📄 stm32l476xx_i2c_driver.h      # I2C driver interface ⏳
│   │   └── 📄 stm32l476xx.h                 # MCU register definitions
│   │
│   └── 📁 Src/                              # Custom driver implementations
│       ├── 📄 stm32l476xx_gpio_driver.c     # GPIO driver implementation ✅
│       ├── 📄 stm32l476xx_spi_driver.c      # SPI driver implementation ✅
│       └── 📄 stm32l476xx_i2c_driver.c      # I2C driver implementation ⏳
│
├── 📁 Src/                        # Application source files
│   └── 📄 main.c                  # Main application code
│
├── 📁 Startup/                    # System startup files
│   └── 📄 startup_stm32l476rgtxx.s  # Startup assembly code
│
├── 📁 Examples/                   # Example applications
│   └── 📁 GPIO_Blink/            # Basic GPIO LED blink

├── 📁 Tests/                              # Unit tests
│   ├── 📄 test_stm32l476xx_gpio.c        # GPIO driver tests
│   ├── 📄 test_stm32l476xx_spi.c         # SPI driver tests
│   └── 📄 test_framework.h               # Simple test framework
│
├── 📁 Docs/                       # Documentation
│   ├── 📄 API_Reference.md       # Driver API documentation
│   ├── 📄 Hardware_Setup.md      # Hardware setup guide
│   └── 📁 Datasheets/            # MCU and peripheral datasheets
│
├── 📄 STM32L476RGTX_FLASH.ld     # Linker script for flash memory
├── 📄 STM32L476RGTX_RAM.ld       # Linker script for RAM execution
├── 📄 Makefile                    # Build configuration
├── 📄 .gitignore                  # Git ignore rules
└── 📄 README.md                   # This file
```

---

## 🛠️ Getting Started

### 📋 Prerequisites

#### Hardware Requirements
- **STM32L476RG Nucleo Board** (recommended)
- **STM32L476G-DISCO** (alternative)
- **ST-Link V2** debugger/programmer
- **USB Cable** (USB-A to Mini-B)

#### Software Requirements
- **ARM GCC Toolchain** (`arm-none-eabi-gcc`)
- **OpenOCD** or **ST-Link Utility**
- **Make** build system
- **VS Code** with C/C++ extensions (recommended)

### 🚀 Installation

### 💻 Windows Installation (STM32CubeIDE)


#### 1. Clone the Repository

- Open STM32CubeIDE.
- Go to **File** → **Import...**
- Select **Git** > **Projects from Git** and click **Next**.
- Choose **Clone URI** and click **Next**.
- Enter the repository URL:
  ```
  https://github.com/kivilak/stm32l4xx_drivers.git
  ```
- Follow the prompts to clone the project.

#### 2. Import the Project

- After cloning, CubeIDE may auto-detect a project and prompt to import.
- If not, use **File** → **Import...** → **General** → **Existing Projects into Workspace**.
- Select the cloned folder and finish the import.

#### 3. Build the Project

- Right-click the project in the **Project Explorer**.
- Choose **Build Project** or click the hammer icon in the toolbar.

#### 4. Flash to MCU

- Connect your STM32 board via USB.
- Click the **Debug** or **Run** button (green bug or play icon).
- STM32CubeIDE will flash the firmware to your MCU and start a debug session if selected.

> **Tip:** If prompted, make sure your **Debug Configuration** is set to use ST-Link and the correct board/MCU.

---


---

## 📊 Hardware Configuration

### 🎯 Target MCU Specifications
- **Core**: ARM Cortex-M4 with FPU
- **Flash Memory**: 1MB
- **SRAM**: 128KB
- **Max Clock**: 80MHz
- **Package**: LQFP64
- **Supply Voltage**: 1.71V to 3.6V

### 📍 Pin Configuration

#### GPIO Pin Mapping
```c
// LED Configuration
#define LED_GREEN_PIN    GPIO_PIN_NO_5    // PA5 (Nucleo onboard LED)
#define LED_PORT         GPIOA

// Button Configuration  
#define BUTTON_PIN       GPIO_PIN_NO_13   // PC13 (Nucleo onboard button)
#define BUTTON_PORT      GPIOC

// USART Configuration
#define USART_TX_PIN     GPIO_PIN_NO_2    // PA2 (USART2_TX)
#define USART_RX_PIN     GPIO_PIN_NO_3    // PA3 (USART2_RX)
#define USART_PORT       GPIOA
```

---

## 📝 API Usage Examples

### GPIO Driver Usage
```c
#include "stm32l476xx.h"

// Initialize GPIO
GPIO_Handle_t GPIOLed;

GPIOLed.pGPIOx = GPIOA;
GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

GPIO_Init(&GPIOLed);

// Toggle LED
GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
```

---

## 📚 Documentation

### Driver API Reference
Each driver includes comprehensive documentation with:
- **Function descriptions** and parameters
- **Return values** and error codes
- **Usage examples** and best practices
- **Hardware requirements** and pin configurations

### Code Documentation
```c
/*************************************************************************************************
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
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);
```

---

## 🤝 Contributing

### Development Guidelines
1. **Follow coding standards** (MISRA-C compliance preferred)
2. **Write comprehensive tests** for all new drivers
3. **Document all public APIs** using Doxygen format
4. **Optimize for low power** consumption when applicable
5. **Maintain compatibility** with existing drivers

### Pull Request Process
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-driver`)
3. Implement your changes with tests
4. Update documentation
5. Submit a pull request with detailed description

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **STMicroelectronics** for comprehensive documentation
- **ARM** for Cortex-M4 architecture reference
- **Open source community** for tools and inspiration

---

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/kivilak/stm32l4xx_drivers/issues)
- **Discussions**: [GitHub Discussions](https://github.com/kivilak/stm32l4xx_drivers/discussions)
- **Documentation**: [Wiki](https://github.com/kivilak/stm32l4xx_drivers/wiki)

---