/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MC_NTC_Pin GPIO_PIN_9
#define MC_NTC_GPIO_Port GPIOB
#define VCP_TX_Pin GPIO_PIN_10
#define VCP_TX_GPIO_Port GPIOC
#define VCP_RX_Pin GPIO_PIN_11
#define VCP_RX_GPIO_Port GPIOC
#define Tamper_key_Pin GPIO_PIN_13
#define Tamper_key_GPIO_Port GPIOC
#define EX_RESET_OD_Pin GPIO_PIN_0
#define EX_RESET_OD_GPIO_Port GPIOC
#define MC_PFCsync1_Pin GPIO_PIN_1
#define MC_PFCsync1_GPIO_Port GPIOC
#define JOY_UP_Pin GPIO_PIN_2
#define JOY_UP_GPIO_Port GPIOC
#define JOY_DOWN_Pin GPIO_PIN_3
#define JOY_DOWN_GPIO_Port GPIOC
#define JOY_SEL_Pin GPIO_PIN_0
#define JOY_SEL_GPIO_Port GPIOA
#define MC_BusVoltage_Pin GPIO_PIN_1
#define MC_BusVoltage_GPIO_Port GPIOA
#define MC_CurrentA_Pin GPIO_PIN_2
#define MC_CurrentA_GPIO_Port GPIOA
#define Audio_OUT_L_Pin GPIO_PIN_4
#define Audio_OUT_L_GPIO_Port GPIOA
#define Audio_OUT_R_Pin GPIO_PIN_5
#define Audio_OUT_R_GPIO_Port GPIOA
#define AUdio_IN_Pin GPIO_PIN_6
#define AUdio_IN_GPIO_Port GPIOA
#define MC_EnIndex_Pin GPIO_PIN_0
#define MC_EnIndex_GPIO_Port GPIOB
#define MC_PFCpwm_Pin GPIO_PIN_1
#define MC_PFCpwm_GPIO_Port GPIOB
#define Potentiometer_Pin GPIO_PIN_2
#define Potentiometer_GPIO_Port GPIOB
#define MC_CurrentC_Pin GPIO_PIN_10
#define MC_CurrentC_GPIO_Port GPIOB
#define MC_EmergencySTOP_Pin GPIO_PIN_12
#define MC_EmergencySTOP_GPIO_Port GPIOB
#define MC_DissipativeBrake_Pin GPIO_PIN_15
#define MC_DissipativeBrake_GPIO_Port GPIOB
#define MC_UH_Pin GPIO_PIN_8
#define MC_UH_GPIO_Port GPIOA
#define MC_VH_Pin GPIO_PIN_9
#define MC_VH_GPIO_Port GPIOA
#define MC_EnA_Pin GPIO_PIN_6
#define MC_EnA_GPIO_Port GPIOC
#define JOY_RIGHT_Pin GPIO_PIN_7
#define JOY_RIGHT_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_9
#define LED4_GPIO_Port GPIOD
#define MC_WH_Pin GPIO_PIN_10
#define MC_WH_GPIO_Port GPIOA
#define JOY_LEFT_Pin GPIO_PIN_8
#define JOY_LEFT_GPIO_Port GPIOC
#define SDcard_detect_Pin GPIO_PIN_9
#define SDcard_detect_GPIO_Port GPIOC
#define MC_PFCsync2_Pin GPIO_PIN_0
#define MC_PFCsync2_GPIO_Port GPIOD
#define MicroSD_CS_OD_Pin GPIO_PIN_1
#define MicroSD_CS_OD_GPIO_Port GPIOD
#define MC_UL_Pin GPIO_PIN_2
#define MC_UL_GPIO_Port GPIOD
#define MC_VL_Pin GPIO_PIN_3
#define MC_VL_GPIO_Port GPIOD
#define MC_WL_Pin GPIO_PIN_4
#define MC_WL_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOD
#define MC_EnB_Pin GPIO_PIN_5
#define MC_EnB_GPIO_Port GPIOB
#define FDCAN1_STBY_Pin GPIO_PIN_0
#define FDCAN1_STBY_GPIO_Port GPIOE
#define FDCAN2_STBY_Pin GPIO_PIN_1
#define FDCAN2_STBY_GPIO_Port GPIOE
#define LCD_CS_OD_Pin GPIO_PIN_8
#define LCD_CS_OD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
