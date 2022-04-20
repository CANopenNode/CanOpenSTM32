# CANopenNode example for STM32

CANopenSTM32 is a CANopen stack example running on STM32 microcontroller.

## How to run demos

Examples are developed in [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) tool,
official ST development studio for any STM32 microcontroller.
You can directly open projects run prepared examples on the board.

## Supported boards and MCUs
 

### [STM32H735G-DK](https://www.st.com/en/evaluation-tools/stm32h735g-dk.html).
It has many features of STM32H7xx series and includes 3 CAN transceivers on the board.
You do not need any additional hardware to connect to existing CAN network.
It also includes built-in programmer and virtual COM port for communication, hence evaluation is quick and easy.

> CanOpen demo works at `FDCAN1` port. Use connector *CN18*.

> FDCAN IP block is same for any STM32H7xx MCU family, hence migration to your custom board should be straight-forward.

### [NUCLEO-F303ZE](https://www.st.com/en/evaluation-tools/nucleo-f303ze.html) / [NUCLEO-F072RB](https://www.st.com/en/evaluation-tools/nucleo-f072rb.html) + [MAX33040ESHLD](https://www.digikey.ie/en/products/detail/analog-devices-inc-maxim-integrated/MAX33040ESHLD/13558019)

Nucleo includes an arduino compatible headers which can be used to add MAX33040ESHLD to it and this bundle provide  the minimum required components to establish a CAN communication and CanOpenNode on top of that.

This project is tied to the CubeMX configuration, so it is up to the user to provide compatible configuration using CubeMX (bitrate, interrupt activiation and etc). (In this example, the MX_CAN_Init function will be called by CO_Driver_STM32Fxxx.c)

#### When using/porting NUCLEO examples do not forget to : 
- Set the right baudrate for CAN (with TimeSeg1 set to 10 and TimeSeg2 set to 1) in the CubeMX GUI
- Activate the RX and TX interrupt on the CAN peripheral
- Configure a timer for a 1ms overflow interrupt (TIM17 used in these examples)
- From `Prject Manager` tab in the STM32Cube and `Code Generator` section, choose Generate peripheral initialization as a pair of '.c/.h' files per peripheral to create `Tim.H` and `can.h` files
- if copying `CANOpenNode` folder entirely, you should remove or filter `example` folder in that directory.
- if pulling the main `CANOpenNode`, please set  `CO_CONFIG_STORAGE_ENABLE`  to `0x00` in `301\CO_config.h`, as storage is not yet impelemented in this port.

### Features

* Runs out of the box on STM32H735G-DK board
* Bare metal or FreeRTOS operating system examples
* `FDCAN1` (*CN18*) hardware is used for communication at 125kHz
* CANopen LED control is well integrated
* Debug messages are available through VCP COM port at `115200` bauds
* Can be used as a reference code for end product

### Clone or update

Clone the project from git repository and get submodules:

```
git clone https://github.com/CANopenNode/CANopenSTM32
cd CANopenSTM32
git submodule update --init --recursive
```

Update an existing project including submodules:

```
cd CANopenSTM32
git pull
git submodule update --init --recursive
```

## License

This file is part of CANopenNode, an opensource CANopen Stack. Project home page is https://github.com/CANopenNode/CANopenNode. For more information on CANopen see http://www.can-cia.org/.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0
