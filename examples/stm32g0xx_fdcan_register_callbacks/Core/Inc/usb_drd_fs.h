/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_drd_fs.h
  * @brief   This file contains all the function prototypes for
  *          the usb_drd_fs.c file
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
#ifndef __USB_DRD_FS_H__
#define __USB_DRD_FS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USB_DRD_FS_PCD_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USB_DRD_FS_H__ */

