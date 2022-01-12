# CANopenNode example for STM32

CANopenSTM32 is a CANopen stack example running on STM32 microcontroller.

## How to run demos

Examples are developed in [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) tool,
official ST development studio for any STM32 microcontroller.
You can directly open projects run prepared examples on the board.

Currently used board is [STM32H735G-DK](https://www.st.com/en/evaluation-tools/stm32h735g-dk.html).
It has many features of STM32H7xx series and includes 3 CAN transceivers on the board.
You do not need any additional hardware to connect to existing CAN network.
It also includes built-in programmer and virtual COM port for communication, hence evaluation is quick and easy.

> CanOpen demo works at `FDCAN1` port. Use connector *CN18*.

> FDCAN IP block is same for any STM32H7xx MCU family, hence migration to your custom board should be straight-forward.

### VSCode support

Examples come ready for CMake and VSCode support.
[This tutorial](https://github.com/MaJerle/stm32-cube-cmake-vscode) explains all VSCode and CMake pre-requisities users should have installed.

### Features

* Runs out of the box on STM32H735G-DK board
* Bare metal or FreeRTOS operating system examples
* `FDCAN1` (*CN18*) hardware is used for communication at 125kHz
* CANopen LED control is well integrated
* Debug messages are available through VCP COM port at `115200` bauds
* Can be used as a reference code for end product

### Clone project or update to latest

Clone the project from git repository and get submodules:

```
git clone https://github.com/CANopenNode/CANopenSTM32
cd CANopenSTM32
git submodule update --init --remote
```

Update an existing project including submodules:

```
cd CANopenSTM32
git pull
git submodule update --init --remote
```

## License

This file is part of CANopenNode, an opensource CANopen Stack. Project home page is https://github.com/CANopenNode/CANopenNode. For more information on CANopen see http://www.can-cia.org/.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0
