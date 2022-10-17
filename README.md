# CANopenNode STM32

CANopenSTM32 is a CANopen stack running on STM32 microcontroller based on [CANOpenNode](https://github.com/CANopenNode/CANopenNode) stack.

## How to run demos

Examples are developed in [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) tool,
official ST development studio for any STM32 microcontroller.
You can directly open projects in the STM32CubeIDE and run examples on the relevant boards.

## Repository directories

- `.\CANopenNode` : Includes the stack implemenation, for most of usecases you don't need to touch these files as they are constant between all the variations and ports (i.e. Linux, PIC, STM32 and etc.)
- `.\CANopenNodeSTM32` : Includes the implementation of low-level driver for STM32 microcontrollers, support both CAN based controllers and FDCAN without any changes. It automatically detect the controller type and activate the relevant calls to STM32 HAL libraries
- `.\examples` : It include many examples on various boards including STM32F4-Discovery, STM32G0C1 Evaluation board, STM32F076 Nucleo Board, STM32H735G-Development Kit.
- `.\Legacy` : It include an older version of CANOpenSTM32 implementation, specifically made for FDCAN controllers, however it was stable and include the FreeRTOS implementation. 

## Supported boards and MCUs
 

### [STM32H735G-DK](https://www.st.com/en/evaluation-tools/stm32h735g-dk.html).
It has many features of STM32H7xx series and includes 3 CAN transceivers on the board.
You do not need any additional hardware to connect to existing CAN network.
It also includes built-in programmer and virtual COM port for communication, hence evaluation is quick and easy.

> CanOpen demo works at `FDCAN1` port. Use connector *CN18*.

> FDCAN IP block is same for any STM32H7xx MCU family, hence migration to your custom board should be straight-forward.

* Runs out of the box on STM32H735G-DK board
* Bare metal, and FreeRTOS operating system examples
* `FDCAN1` (*CN18*) hardware is used for communication at 125kHz
* CANopen LED control is well integrated
* Debug messages are available through VCP COM port at `115200` bauds
* Can be used as a reference code for end product



### [STM32G0C1VE-EV](https://www.st.com/en/evaluation-tools/stm32g0c1e-ev.html)
The STM32G0C1E-EV Evaluation board is a high-end development platform for the STM32G0C1VET6 microcontroller. It has many features including two CAN FD controller and physical layer on board.
You don't need any additional hardware to connect to existing CAN network.
It also includes built-in programmer and virtual COM port for communication, hence evaluation is quick and easy.
> CanOpen demo works at `FDCAN1` port. Use connector *CN12*.
> FDCAN IP block is same for any STM32G0xx MCU family, hence migration to your custom board should be straight-forward.


### [NUCLEO-F303ZE](https://www.st.com/en/evaluation-tools/nucleo-f303ze.html) / [NUCLEO-F072RB](https://www.st.com/en/evaluation-tools/nucleo-f072rb.html) + [MAX33040ESHLD](https://www.digikey.ie/en/products/detail/analog-devices-inc-maxim-integrated/MAX33040ESHLD/13558019)

Nucleo includes an arduino compatible headers which can be used to add MAX33040ESHLD to it and this bundle provide  the minimum required components to establish a CAN communication and CanOpenNode on top of that.

This project is tied to the CubeMX configuration, so it is up to the user to provide compatible configuration using CubeMX (bitrate, interrupt activiation and etc).


### [STM32F4DISCOVERY](https://www.st.com/en/evaluation-tools/stm32f4discovery.html) + Any CAN Bus Physical Layer Module

Have a look at STM32CubeMX configuration file for pin mapping.


## Video Tutorial

To get a good grasp of CANOpenNode Stack and CANOpenNodeSTM32 stack, you can refer to this video, which explains from basics to implementation and porting of the CANOpenNode stack.

[![CANOpen Node STM32 From basics to coding](https://img.youtube.com/vi/R-r5qIOTjOo/0.jpg)](https://www.youtube.com/watch?v=R-r5qIOTjOo)

>[00:00](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=0s) Introduction and Overview [1:13](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=73s) Why CAN ? [4:51](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=291s) CAN Bus [8:55](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=535s) Why CANOpen ? [13:27](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=807s) CANOpen architecture [20:00](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=1200s) Object dictionary [21:38](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=1298s) Important CANOpen concepts [23:29](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=1409s) PDO [27:25](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=1645s) SDO [32:23](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=1943s) NMT [33:25](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=2005s) CANOpenNode Open-Source Stack [39:26](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=2366s) STM32 Practical implementation [40:29](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=2429s) CANOpen Tutorial code preparation [43:09](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=2589s) Importing examples to STM32CubeIDE and programming them [47:04](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=2824s) Examples explanation [57:00](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=3420s) Porting to custom STM32 board [1:18:20](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=4700s) EDS Editor (Object dictionary editor) [1:25:54](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=5154s) Creating a TPDO [1:39:55](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=5995s) Accessing OD Variables [1:54:08](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=6848s) Creating an RPDO [2:05:50](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=7550s) Using the SDOs [2:52:52](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=10372s) Node guarding [3:04:38](https://www.youtube.com/watch?v=R-r5qIOTjOo&t=11078s) Transmitting PDOs manually



## Porting to other STM32 microcontrollers checklist :
- Create a new project in STM32CubeMXIDE
- Configure CAN/FDCAN to your desired bitrate and map it to relevant tx/rx pins - Make sure yo activate Auto Bus recovery (bxCAN) / protocol exception handling (FDCAN)
- Activate the RX and TX interrupt on the CAN peripheral
- Enable a timer for a 1ms overflow interrupt and activate interrupt for that timer
- Copy or clone `CANopenNode` and `CANopenNodeSTM32` into your project directory 
- Add `CANopenNode` and `CANopenNodeSTM32` to Source locations in `Project Properties -> C/C++ General -> Paths and Symbols -> Source Locations`
  - add an exclusion filter for `example/` folder for `CANopenNode` folder
- Add `CANOpenNode` and `CANopenNodeSTM32` to `Project Properties -> C/C++ General -> Paths and Symbols -> Includes` under `GNU C` items
- In your main.c, add `#include "CO_app_STM32.h"`
  ```c
    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */
    #include "CO_app_STM32.h"
    /* USER CODE END Includes */
  ```
  - Make sure that you have the `HAL_TIM_PeriodElapsedCallback` function implmented with a call to `canopen_app_interrupt`.

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  // Handle CANOpen app interrupts
  if (htim == canopenNodeSTM32->timerHandle) {
      canopen_app_interrupt();
  }
  /* USER CODE END Callback 1 */
}

```

- Now based on your application, you'll take one of the following approaches :

  ### In Baremetal application
- In your main.c, add following codes to your USER CODE BEGIN 2
  ```c
    /* USER CODE BEGIN 2 */
    CANopenNodeSTM32 canOpenNodeSTM32;
    canOpenNodeSTM32.CANHandle = &hcan;
    canOpenNodeSTM32.HWInitFunction = MX_CAN_Init;
    canOpenNodeSTM32.timerHandle = &htim17;
    canOpenNodeSTM32.desiredNodeID = 29;
    canOpenNodeSTM32.baudrate = 125;
    canopen_app_init(&canOpenNodeSTM32);
    /* USER CODE END 2 */
  ```

- In your main.c, add following codes to your USER CODE BEGIN WHILE
  ```c
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  canopen_app_process();
    /* USER CODE END WHILE */

  ```

  ### In FreeRTOS Applications
- You need to create a task for CANOpen, we call it `canopen_task` with a high priority and in that task use the following code : 

```c

void canopen_task(void *argument)
{
  /* USER CODE BEGIN canopen_task */
  CANopenNodeSTM32 canOpenNodeSTM32;
  canOpenNodeSTM32.CANHandle = &hfdcan1;
  canOpenNodeSTM32.HWInitFunction = MX_FDCAN1_Init;
  canOpenNodeSTM32.timerHandle = &htim17;
  canOpenNodeSTM32.desiredNodeID = 21;
  canOpenNodeSTM32.baudrate = 125;
  canopen_app_init(&canOpenNodeSTM32);
  /* Infinite loop */
  for(;;)
  {
	  //Reflect CANopenStatus on LEDs
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, !canOpenNodeSTM32.outStatusLEDGreen);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, !canOpenNodeSTM32.outStatusLEDRed);
    canopen_app_process();
    // Sleep for 1ms, you can decrease it if required, in the canopen_app_process we will double check to make sure 1ms passed
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END canopen_task */
}

```
> In RTOS applications, be very careful when accessing OD variables, CAN Send and EMCY variable. You should lock the these critical sections to make sure prevent race conditions. Have a look at `CO_LOCK_CAN_SEND`, `CO_LOCK_OD` and `CO_LOCK_EMCY`.

- Run your project on the target board, you should be able to see bootup message on startup

### Known limitations : 

- We have never tested the multi CANOpen on a single STM32 device, but the the original CANOpenNode has the capability to use multi modules, which you can develop yourself.

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
