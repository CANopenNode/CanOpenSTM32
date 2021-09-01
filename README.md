# CANopenNode example for STM32

CANopenSTM32 is a CANopen stack running in STM32 microcontrollers

## How to run demos

Examples are prepared for [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) tool,
official ST developed studio for STM32.
You can directly open projects there and run example on the board.

Examples are developed for [STM32H735G-DK](https://www.st.com/en/evaluation-tools/stm32h735g-dk.html),
since it has a full features STM32H7 microcontroller and includes CAN transceiver on board.

There is also built-in programmer and virtual COM port for communication, hence no external components are necessary.

### Features

* Runs out of the box on STM32H735G-DK board
* Bare metal or FreeRTOS operating system example
* FDCAN1 (CN18) hardware is used for communication at 125kHz
* CANopen LED control is well integrated
* Debug messages are available through VCP COM port at `115200` bauds
* Can be used as a reference code for end product

## License

This file is part of CANopenNode, an opensource CANopen Stack. Project home page is https://github.com/CANopenNode/CANopenNode. For more information on CANopen see http://www.can-cia.org/.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0
