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
#include "stm32f7xx_hal.h"

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
#define A20_Pin GPIO_PIN_4
#define A20_GPIO_Port GPIOE
#define A19_Pin GPIO_PIN_3
#define A19_GPIO_Port GPIOE
#define MII_TXD3_Pin GPIO_PIN_2
#define MII_TXD3_GPIO_Port GPIOE
#define MII_TXD1_Pin GPIO_PIN_14
#define MII_TXD1_GPIO_Port GPIOG
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define ULPI_D7_Pin GPIO_PIN_5
#define ULPI_D7_GPIO_Port GPIOB
#define SD2_D3_Pin GPIO_PIN_4
#define SD2_D3_GPIO_Port GPIOB
#define SD2_D2_Pin GPIO_PIN_3
#define SD2_D2_GPIO_Port GPIOB
#define SD2_CMD_Pin GPIO_PIN_7
#define SD2_CMD_GPIO_Port GPIOD
#define SD1_CLK_Pin GPIO_PIN_12
#define SD1_CLK_GPIO_Port GPIOC
#define TDI_Pin GPIO_PIN_15
#define TDI_GPIO_Port GPIOA
#define TCK_SWCLK_Pin GPIO_PIN_14
#define TCK_SWCLK_GPIO_Port GPIOA
#define TMS_SWDIO_Pin GPIO_PIN_13
#define TMS_SWDIO_GPIO_Port GPIOA
#define A21_Pin GPIO_PIN_5
#define A21_GPIO_Port GPIOE
#define SAI2_MCLKB_Pin GPIO_PIN_6
#define SAI2_MCLKB_GPIO_Port GPIOE
#define MII_TXD0_Pin GPIO_PIN_13
#define MII_TXD0_GPIO_Port GPIOG
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define PAR_VSYNC_Pin GPIO_PIN_7
#define PAR_VSYNC_GPIO_Port GPIOB
#define QSPI_BK1_NCS_Pin GPIO_PIN_6
#define QSPI_BK1_NCS_GPIO_Port GPIOB
#define SDNCAS_Pin GPIO_PIN_15
#define SDNCAS_GPIO_Port GPIOG
#define MII_TX_EN_Pin GPIO_PIN_11
#define MII_TX_EN_GPIO_Port GPIOG
#define MII_MDC_GPIO_Pin GPIO_PIN_13
#define MII_MDC_GPIO_GPIO_Port GPIOJ
#define MII_MDIO_GPIO_Pin GPIO_PIN_12
#define MII_MDIO_GPIO_GPIO_Port GPIOJ
#define SD2_CLK_Pin GPIO_PIN_6
#define SD2_CLK_GPIO_Port GPIOD
#define D2_Pin GPIO_PIN_0
#define D2_GPIO_Port GPIOD
#define SD1_D3_Pin GPIO_PIN_11
#define SD1_D3_GPIO_Port GPIOC
#define SD1_D2_Pin GPIO_PIN_10
#define SD1_D2_GPIO_Port GPIOC
#define EXPANDER_INT_Pin GPIO_PIN_8
#define EXPANDER_INT_GPIO_Port GPIOI
#define FMC_NBL2_Pin GPIO_PIN_4
#define FMC_NBL2_GPIO_Port GPIOI
#define LCD_DE_Pin GPIO_PIN_7
#define LCD_DE_GPIO_Port GPIOK
#define LCD_B7_Pin GPIO_PIN_6
#define LCD_B7_GPIO_Port GPIOK
#define LCD_B6_Pin GPIO_PIN_5
#define LCD_B6_GPIO_Port GPIOK
#define SPDIF_RX1_Pin GPIO_PIN_12
#define SPDIF_RX1_GPIO_Port GPIOG
#define SD2_D1_Pin GPIO_PIN_10
#define SD2_D1_GPIO_Port GPIOG
#define LCD_B2_Pin GPIO_PIN_14
#define LCD_B2_GPIO_Port GPIOJ
#define FMC_NWE_Pin GPIO_PIN_5
#define FMC_NWE_GPIO_Port GPIOD
#define PAR_D5_Pin GPIO_PIN_3
#define PAR_D5_GPIO_Port GPIOD
#define D3_Pin GPIO_PIN_1
#define D3_GPIO_Port GPIOD
#define D27_Pin GPIO_PIN_3
#define D27_GPIO_Port GPIOI
#define D26_Pin GPIO_PIN_2
#define D26_GPIO_Port GPIOI
#define TAMPER_WKUP_KEY_Pin GPIO_PIN_13
#define TAMPER_WKUP_KEY_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOF
#define FMC_NBL3_Pin GPIO_PIN_5
#define FMC_NBL3_GPIO_Port GPIOI
#define D29_Pin GPIO_PIN_7
#define D29_GPIO_Port GPIOI
#define D31_Pin GPIO_PIN_10
#define D31_GPIO_Port GPIOI
#define D28_Pin GPIO_PIN_6
#define D28_GPIO_Port GPIOI
#define LCD_B5_Pin GPIO_PIN_4
#define LCD_B5_GPIO_Port GPIOK
#define LCD_B4_Pin GPIO_PIN_3
#define LCD_B4_GPIO_Port GPIOK
#define SAI2_FSB_Pin GPIO_PIN_9
#define SAI2_FSB_GPIO_Port GPIOG
#define LCD_B3_Pin GPIO_PIN_15
#define LCD_B3_GPIO_Port GPIOJ
#define FMC_NOE_Pin GPIO_PIN_4
#define FMC_NOE_GPIO_Port GPIOD
#define SD1_CMD_Pin GPIO_PIN_2
#define SD1_CMD_GPIO_Port GPIOD
#define D23_Pin GPIO_PIN_15
#define D23_GPIO_Port GPIOH
#define D25_Pin GPIO_PIN_1
#define D25_GPIO_Port GPIOI
#define RS232_IrDA_RX_Pin GPIO_PIN_10
#define RS232_IrDA_RX_GPIO_Port GPIOA
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOF
#define LCD_HSYNC_Pin GPIO_PIN_12
#define LCD_HSYNC_GPIO_Port GPIOI
#define D30_Pin GPIO_PIN_9
#define D30_GPIO_Port GPIOI
#define D21_Pin GPIO_PIN_13
#define D21_GPIO_Port GPIOH
#define D22_Pin GPIO_PIN_14
#define D22_GPIO_Port GPIOH
#define D24_Pin GPIO_PIN_0
#define D24_GPIO_Port GPIOI
#define RS2320IrDA0T_Pin GPIO_PIN_9
#define RS2320IrDA0T_GPIO_Port GPIOA
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define ULPI_DIR_Pin GPIO_PIN_11
#define ULPI_DIR_GPIO_Port GPIOI
#define SD1_D1_Pin GPIO_PIN_9
#define SD1_D1_GPIO_Port GPIOC
#define MII_MCO_Pin GPIO_PIN_8
#define MII_MCO_GPIO_Port GPIOA
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOF
#define LCD_VSYNC_Pin GPIO_PIN_13
#define LCD_VSYNC_GPIO_Port GPIOI
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOI
#define SD1_D0_Pin GPIO_PIN_8
#define SD1_D0_GPIO_Port GPIOC
#define FMC_NE1_Pin GPIO_PIN_7
#define FMC_NE1_GPIO_Port GPIOC
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOF
#define LCD_CLK_Pin GPIO_PIN_14
#define LCD_CLK_GPIO_Port GPIOI
#define ULPI_NXT_Pin GPIO_PIN_4
#define ULPI_NXT_GPIO_Port GPIOH
#define SDCLK_Pin GPIO_PIN_8
#define SDCLK_GPIO_Port GPIOG
#define FMC_NWAIT_Pin GPIO_PIN_6
#define FMC_NWAIT_GPIO_Port GPIOC
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOF
#define SDNWE_Pin GPIO_PIN_5
#define SDNWE_GPIO_Port GPIOH
#define SDNE0_Pin GPIO_PIN_3
#define SDNE0_GPIO_Port GPIOH
#define NAND_INT_Pin GPIO_PIN_7
#define NAND_INT_GPIO_Port GPIOG
#define FMC_NE3_Pin GPIO_PIN_6
#define FMC_NE3_GPIO_Port GPIOG
#define QSPI_BK1_IO2_Pin GPIO_PIN_7
#define QSPI_BK1_IO2_GPIO_Port GPIOF
#define QSPI_BK1_IO3_Pin GPIO_PIN_6
#define QSPI_BK1_IO3_GPIO_Port GPIOF
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOF
#define SDCKE0_Pin GPIO_PIN_2
#define SDCKE0_GPIO_Port GPIOH
#define D1_Pin GPIO_PIN_15
#define D1_GPIO_Port GPIOD
#define ULPI_D6_Pin GPIO_PIN_13
#define ULPI_D6_GPIO_Port GPIOB
#define D15_Pin GPIO_PIN_10
#define D15_GPIO_Port GPIOD
#define Potentiometer_Pin GPIO_PIN_10
#define Potentiometer_GPIO_Port GPIOF
#define QSPI_BK1_IO1_Pin GPIO_PIN_9
#define QSPI_BK1_IO1_GPIO_Port GPIOF
#define QSPI_BK1_IO0_Pin GPIO_PIN_8
#define QSPI_BK1_IO0_GPIO_Port GPIOF
#define MII_TX_CLK_Pin GPIO_PIN_3
#define MII_TX_CLK_GPIO_Port GPIOC
#define D0_Pin GPIO_PIN_14
#define D0_GPIO_Port GPIOD
#define ULPI_D5_Pin GPIO_PIN_12
#define ULPI_D5_GPIO_Port GPIOB
#define D14_Pin GPIO_PIN_9
#define D14_GPIO_Port GPIOD
#define D13_Pin GPIO_PIN_8
#define D13_GPIO_Port GPIOD
#define ULPI_STP_Pin GPIO_PIN_0
#define ULPI_STP_GPIO_Port GPIOC
#define SAI1_SDA_Pin GPIO_PIN_1
#define SAI1_SDA_GPIO_Port GPIOC
#define MII_TXD2_Pin GPIO_PIN_2
#define MII_TXD2_GPIO_Port GPIOC
#define QSPI_CLK_Pin GPIO_PIN_2
#define QSPI_CLK_GPIO_Port GPIOB
#define A6_Pin GPIO_PIN_12
#define A6_GPIO_Port GPIOF
#define A11_Pin GPIO_PIN_1
#define A11_GPIO_Port GPIOG
#define A9_Pin GPIO_PIN_15
#define A9_GPIO_Port GPIOF
#define LCD_R5_Pin GPIO_PIN_4
#define LCD_R5_GPIO_Port GPIOJ
#define A17_Pin GPIO_PIN_12
#define A17_GPIO_Port GPIOD
#define A18_Pin GPIO_PIN_13
#define A18_GPIO_Port GPIOD
#define A13_Pin GPIO_PIN_3
#define A13_GPIO_Port GPIOG
#define A12_Pin GPIO_PIN_2
#define A12_GPIO_Port GPIOG
#define LCD_R6_Pin GPIO_PIN_5
#define LCD_R6_GPIO_Port GPIOJ
#define D20_Pin GPIO_PIN_12
#define D20_GPIO_Port GPIOH
#define MII_RX_CLK_Pin GPIO_PIN_1
#define MII_RX_CLK_GPIO_Port GPIOA
#define SAI2_SDB_Pin GPIO_PIN_0
#define SAI2_SDB_GPIO_Port GPIOA
#define PAR_HSYNC_Pin GPIO_PIN_4
#define PAR_HSYNC_GPIO_Port GPIOA
#define MII_RXD0_Pin GPIO_PIN_4
#define MII_RXD0_GPIO_Port GPIOC
#define A7_Pin GPIO_PIN_13
#define A7_GPIO_Port GPIOF
#define A10_Pin GPIO_PIN_0
#define A10_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOJ
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define A16_Pin GPIO_PIN_11
#define A16_GPIO_Port GPIOD
#define A15_Pin GPIO_PIN_5
#define A15_GPIO_Port GPIOG
#define A14_Pin GPIO_PIN_4
#define A14_GPIO_Port GPIOG
#define MII_RXD3_Pin GPIO_PIN_7
#define MII_RXD3_GPIO_Port GPIOH
#define D17_Pin GPIO_PIN_9
#define D17_GPIO_Port GPIOH
#define D19_Pin GPIO_PIN_11
#define D19_GPIO_Port GPIOH
#define SAI2_SCKB_Pin GPIO_PIN_2
#define SAI2_SCKB_GPIO_Port GPIOA
#define PAR_PCLK_Pin GPIO_PIN_6
#define PAR_PCLK_GPIO_Port GPIOA
#define ULPI_CK_Pin GPIO_PIN_5
#define ULPI_CK_GPIO_Port GPIOA
#define MII_RXD1_Pin GPIO_PIN_5
#define MII_RXD1_GPIO_Port GPIOC
#define A8_Pin GPIO_PIN_14
#define A8_GPIO_Port GPIOF
#define DSI_TE_Pin GPIO_PIN_2
#define DSI_TE_GPIO_Port GPIOJ
#define SDNRAS_Pin GPIO_PIN_11
#define SDNRAS_GPIO_Port GPIOF
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D8_Pin GPIO_PIN_11
#define D8_GPIO_Port GPIOE
#define D11_Pin GPIO_PIN_14
#define D11_GPIO_Port GPIOE
#define ULPI_D3_Pin GPIO_PIN_10
#define ULPI_D3_GPIO_Port GPIOB
#define MII_RXD2_Pin GPIO_PIN_6
#define MII_RXD2_GPIO_Port GPIOH
#define D16_Pin GPIO_PIN_8
#define D16_GPIO_Port GPIOH
#define D18_Pin GPIO_PIN_10
#define D18_GPIO_Port GPIOH
#define ULPI_D0_Pin GPIO_PIN_3
#define ULPI_D0_GPIO_Port GPIOA
#define MII_RX_DV_Pin GPIO_PIN_7
#define MII_RX_DV_GPIO_Port GPIOA
#define ULPI_D2_Pin GPIO_PIN_1
#define ULPI_D2_GPIO_Port GPIOB
#define ULPI_D1_Pin GPIO_PIN_0
#define ULPI_D1_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOJ
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOJ
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define D9_Pin GPIO_PIN_12
#define D9_GPIO_Port GPIOE
#define D12_Pin GPIO_PIN_15
#define D12_GPIO_Port GPIOE
#define D10_Pin GPIO_PIN_13
#define D10_GPIO_Port GPIOE
#define ULPI_D4_Pin GPIO_PIN_11
#define ULPI_D4_GPIO_Port GPIOB
#define USB_FS2_DM_Pin GPIO_PIN_14
#define USB_FS2_DM_GPIO_Port GPIOB
#define USB_FS2_DP_Pin GPIO_PIN_15
#define USB_FS2_DP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
