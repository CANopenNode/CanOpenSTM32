/*
 * CO_app_STM32.h
 *
 *  Created on: Aug 7, 2022
 *      Author: hamed
 */

#ifndef CANOPENSTM32_CO_APP_STM32_H_
#define CANOPENSTM32_CO_APP_STM32_H_

#include "CANopen.h"
#include "main.h"
typedef struct{
	uint8_t desiredNodeID;
	uint8_t activeNodeID;
	uint8_t baudrate;
	TIM_HandleTypeDef *timerHandle;
#ifdef CO_STM32_FDCAN_Driver
	FDCAN_HandleTypeDef *CANHandle;
#else
	CAN_HandleTypeDef *CANHandle;
#endif
	void (*HWInitFunction)();
} CANopenNodeSTM32;



int canopen_app_init(CANopenNodeSTM32 *canopenSTM32);
int canopen_app_resetCommunication();
void canopen_app_process();
#endif /* CANOPENSTM32_CO_APP_STM32_H_ */
