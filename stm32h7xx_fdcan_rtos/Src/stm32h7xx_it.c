/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "task.h"

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * \brief           This function handles Non maskable interrupt.
 */
void
NMI_Handler(void) {
    while (1) {

    }
}

/**
 * \brief           This function handles Hard fault interrupt.
 */
void
HardFault_Handler(void) {
    while (1) {

    }
}

/**
 * \brief           This function handles Memory management fault.
 */
void
MemManage_Handler(void) {
    while (1) {

    }
}

/**
 * \brief           This function handles Pre-fetch fault, memory access fault.
 */
void
BusFault_Handler(void) {
    while (1) {

    }
}

/**
 * \brief           This function handles Undefined instruction or illegal state.
 */
void
UsageFault_Handler(void) {
    while (1) {

    }
}

/**
 * \brief           This function handles Debug monitor.
 */
void
DebugMon_Handler(void) {

}

/**
 * \brief           This function handles System tick timer.
 */
void
SysTick_Handler(void) {
    HAL_IncTick();

#if (INCLUDE_xTaskGetSchedulerState == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
#endif /* INCLUDE_xTaskGetSchedulerState */
        xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
    }
#endif /* INCLUDE_xTaskGetSchedulerState */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/
