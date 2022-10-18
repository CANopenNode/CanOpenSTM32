#ifndef MAIN_HDR_H
#define MAIN_HDR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

/* Exported functions prototypes */
void Error_Handler(void);

/* Private defines */
#define BTN_Pin                 LL_GPIO_PIN_13
#define BTN_GPIO_Port           GPIOC
#define LED_RED_Pin             LL_GPIO_PIN_2
#define LED_RED_GPIO_Port       GPIOC
#define LED_GREEN_Pin           LL_GPIO_PIN_3
#define LED_GREEN_GPIO_Port     GPIOC

#ifdef __cplusplus
}
#endif

#endif /* MAIN_HDR_H */
