---

# STM32F446RE Peripheral Driver Development

This project focuses on writing low-level drivers for various peripherals of the STM32F446RE microcontroller. The drivers are implemented entirely from scratch in C by referring to the STM32F446RE reference manual and datasheets. The goal is to provide a comprehensive set of peripheral drivers, including GPIO, RCC, I2C, UART, and SPI, offering direct control over the hardware.

## Table of Contents
- [Overview](#overview)
- [Peripherals Implemented](#peripherals-implemented)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Overview
The STM32F446RE is a powerful 32-bit ARM Cortex-M4 microcontroller widely used in embedded applications. This repository includes:
- Custom drivers for key peripherals like GPIO, RCC, I2C, UART, and SPI.
- Code written from scratch in C, adhering closely to the reference manual and datasheet.
- A modular and reusable driver architecture to simplify integration into any project.

This project is ideal for those interested in gaining a deeper understanding of microcontroller peripherals by directly interacting with hardware at the register level.

## Peripherals Implemented
1. **GPIO (General Purpose Input/Output)**
   - Configure pins as input, output, alternate function, or analog.
   - Handle pull-up/pull-down resistors, and configure speed and output type.
   - Implement interrupt handling for GPIO pins.

2. **RCC (Reset and Clock Control)**
   - Configure system clocks, peripheral clocks, and oscillators.
   - Enable and disable clock signals to peripherals.

3. **I2C (Inter-Integrated Circuit)**
   - Implement master/slave mode communication.
   - Support for standard and fast mode speeds.
   - Handle multi-byte data transfers.

4. **UART (Universal Asynchronous Receiver/Transmitter)**
   - Full-duplex communication.
   - Baud rate configuration and error handling.
   - Transmit and receive data via interrupts or polling.

5. **SPI (Serial Peripheral Interface)**
   - Configure master/slave mode communication.
   - Support full-duplex or simplex communication.
   - Handle multi-byte data transfers.

## Project Structure
```
├── Drivers/
│   ├── GPIO/
│   ├── RCC/
│   ├── I2C/
│   ├── UART/
│   └── SPI/
├── Inc/
│   ├── gpio.h
│   ├── rcc.h
│   ├── i2c.h
│   ├── uart.h
│   └── spi.h
├── Src/
│   ├── main.c
│   ├── gpio.c
│   ├── rcc.c
│   ├── i2c.c
│   ├── uart.c
│   └── spi.c
├── README.md
└── Makefile
```

## Getting Started

### Prerequisites
- **STM32F446RE Nucleo Board** (or a similar development board)
- **STM32CubeIDE** or any ARM GCC toolchain for compiling.
- Basic understanding of microcontroller architecture and peripheral registers.

### Setup
1. Clone the repository:
   ```bash
   git clone (https://github.com/KunalSharma17003/Stm32f446xx-Peripheral-Drivers.git)
   ```

2. Open the project in your development environment (e.g., STM32CubeIDE).

3. Flash the compiled binaries onto the STM32F446RE board using a compatible debugger (e.g., ST-Link).

## Usage
The drivers provided in this project allow you to easily interface with peripherals by configuring registers at the hardware level. Each peripheral driver comes with its own initialization and configuration functions.

### Example: GPIO Configuration
```c
//Include the header files
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void Delay(void)
{
	for(uint32_t i = 0; i<500000/2 ; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig_t.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);
	while(1)
	{

		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_5);
		Delay();

	}
	return 0;
}
```




## Contributing
Contributions to improve existing drivers or add new peripheral drivers are welcome. Please fork the repository and submit a pull request with detailed explanations of your changes.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.

---
