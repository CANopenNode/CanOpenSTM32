/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "comm.h"
#include "CANopen.h"
#include "OD.h"
#include "cmsis_os.h"
#include "stm32f769i_eval.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_USART1_UART_Init(void);
static void MX_GPIO_Init(void);

/* Thread functions and attributes */
static void thread_init_entry(void* arg);
static void thread_canopen_entry(void* arg);
static void thread_canopen_periodic_entry(void* arg);
const osThreadAttr_t
thread_init_attr = {
    .name = "init",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t
thread_canopen_attr = {
    .name = "canopen",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t
thread_canopen_periodic_attr = {
    .name = "canopen_periodic",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal1,
};
static osThreadId_t thread_canopen_handle;
static osThreadId_t thread_canopen_periodic_handle;

/* Lwmem buffer for allocation */
static uint8_t lwmem_buffer[0x4000];
const static lwmem_region_t lwmem_default_regions[] = {
        {lwmem_buffer, sizeof(lwmem_buffer)},
        {NULL, 0}
};

/* Local variables */
static CO_t* CO;
static uint32_t co_heap_used;
static CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
static uint8_t LED_red_status, LED_green_status;

/* Default values for CO_CANopenInit() */
#define NMT_CONTROL                     (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define FIRST_HB_TIME                   500
#define SDO_SRV_TIMEOUT_TIME            1000
#define SDO_CLI_TIMEOUT_TIME            500
#define SDO_CLI_BLOCK                   false
#define OD_STATUS_BITS                  NULL

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Configure the MPU attributes */
  MPU_Config();

  /* Enable the CPU Cache */
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  PeriphCommonClock_Config();

  /* Prepare kernel setup */
  osKernelInitialize();
  osThreadNew(thread_init_entry, NULL, &thread_init_attr);
  osKernelStart();

  /* Should not enter here.. */
  while (1) {
      NVIC_SystemReset();
  }
}

/**
  * @brief  Configure the MPU attributes
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Strongly ordered for not defined regions */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x00;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * \brief           Init thread
 * \param[in]       arg: User argument
 */
static void
thread_init_entry(void* arg) {
    /* Initialize all configured peripherals */
    lwmem_assignmem(lwmem_default_regions);
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_BLUE);
    comm_init();
    comm_printf("CANopenNode application running on STM32H735G-DK\r\n");

    /* Start CANopen main task */
    thread_canopen_handle = osThreadNew(thread_canopen_entry, &CanHandle, &thread_canopen_attr);

    /* Add other application tasks... */

    /* Infinite loop */
    while (1) {
        /* Do whatever else is necessary */


        /* Make sure to offload thread or others won't be able to process */
        osDelay(1000);
    }

    /* Self exit */
    osThreadExit();
}

/**
 * \brief           Main CANopen application thread
 * It creates new CANopen instance and sets up CAN peripheral
 *
 * \param[in]       arg: User argument with FDCAN handle
 */
static void
thread_canopen_entry(void* arg) {
    uint32_t max_sleep_time_us;

    comm_printf("CANopen main thread is running\r\n");

    /* Initialize new instance of CANopen */
    if ((CO = CO_new(NULL, &co_heap_used)) == NULL) {
        comm_printf("Error: Could not allocate CO instance\r\n");
        Error_Handler();
    }
    comm_printf("CO allocated, uses %u bytes of heap memory\r\n", (unsigned)co_heap_used);

    /* Set CAN pointer from argument */
    CO->CANmodule->CANptr = arg;

    /* Create OS objects */
    co_drv_create_os_objects();

    /* Get access to mutex before creating periodic thread */
    co_drv_mutex_lock();

    /* Lock access mutex prior creating periodic thread */
    thread_canopen_periodic_handle = osThreadNew(thread_canopen_periodic_entry, NULL, &thread_canopen_periodic_attr);

    /* Start application */
    do {
        uint16_t pendingBitRate = 125;
        uint32_t errInfo = 0, time_old, time_current;
        uint8_t pendingNodeId = 0x12, activeNodeId = 0;
        CO_ReturnError_t err;

        /* Reset normal state */
        CO->CANmodule->CANnormal = false;

        /* Enter CAN configuration. May be NULL, default one is used in driver */
        CO_CANsetConfigurationMode(CO->CANmodule->CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        /* Initialize CANopen */
        if ((err = CO_CANinit(CO, CO->CANmodule->CANptr, pendingBitRate)) != CO_ERROR_NO) {
            comm_printf("Error: CAN initialization failed: %d\n", err);
            Error_Handler();
        }
        comm_printf("CAN initialized\r\n");

        /* Setup LSS block */
        CO_LSS_address_t lssAddress = {
                .identity = {
                .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
            }
        };
        if ((err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate)) != CO_ERROR_NO) {
            comm_printf("Error: LSS slave initialization failed: %d\n", err);
            Error_Handler();
        }
        comm_printf("LSS initialized\r\n");

        /* Initialite core stack */
        activeNodeId = pendingNodeId;
        err = CO_CANopenInit(CO,                /* CANopen object */
                             NULL,              /* alternate NMT */
                             NULL,              /* alternate em */
                             OD,                /* Object dictionary */
                             OD_STATUS_BITS,    /* Optional OD_statusBits */
                             NMT_CONTROL,       /* CO_NMT_control_t */
                             FIRST_HB_TIME,     /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
                             activeNodeId,
                             &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                comm_printf("Error: Object Dictionary entry 0x%X\n", (unsigned)errInfo);
            } else {
                comm_printf("Error: CANopen initialization failed: %d\n", (int)err);
            }
            Error_Handler();
        }
        comm_printf("CANOpen initialized\r\n");

        /* Initialize PDO */
        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if (err != CO_ERROR_NO) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                comm_printf("Error: Object Dictionary entry 0x%X\n", (unsigned)errInfo);
            } else {
                comm_printf("Error: PDO initialization failed: %d\n", (int)err);
            }
            Error_Handler();
        }
        comm_printf("CAN PDO initialized\r\n");

        /* Configure CANopen callbacks, etc */
        if (!CO->nodeIdUnconfigured) {
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            if (storageInitError != 0) {
                CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY,
                               CO_EMC_HARDWARE, storageInitError);
            }
#endif
        } else {
            comm_printf("CANopenNode - Node-id not initialized\n");
            Error_Handler();
        }

        /* Start CAN to receive messages */
        CO_CANsetNormalMode(CO->CANmodule);

        reset = CO_RESET_NOT;
        comm_printf("CANopenNode - Running and ready to communicate...\n");

        /* Release semaphore at this point. We are ready to proceed */
        co_drv_mutex_unlock();

        /* Get current tick time */
        time_old = time_current = osKernelGetTickCount();
        max_sleep_time_us = 0;              /* Get first sleep time */
        while (reset == CO_RESET_NOT) {
            uint32_t timeDifference_us;

            /*
             * This will block this thread for up to maximal time.
             *
             * If new CAN message arrives,
             * thread will be woken-up from CAN RX interrupt,
             * and processing will continue immediately
             */
            CO_WAIT_SYNC_APP_THREAD(max_sleep_time_us / 1000);

            /* Get exclusive access to CANopen core stack. */
            co_drv_mutex_lock();

            /* Set time to max sleep in next iteration */
            max_sleep_time_us = (uint32_t)-1;

            /* Get current kernel tick time */
            time_current = osKernelGetTickCount();
            timeDifference_us = (time_current - time_old) * (1000000 / configTICK_RATE_HZ);
            time_old = time_current;

            /* CANopen process */
            reset = CO_process(CO, false, timeDifference_us, &max_sleep_time_us);

            /* Process LEDs and react only on change */
            LED_red_status = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
            LED_green_status = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

            /* LEDs are active low */
            BSP_LED_Toggle(LED_GREEN);
            BSP_LED_Toggle(LED_BLUE);

            /*
             * We want to wakeup periodic thread,
             * in-charge of other important CANopen tasks
             */
            CO_WAKEUP_PERIODIC_THREAD();

            /* Only now release mutex to allow other tasks accessing CANopen core */
            co_drv_mutex_unlock();
        }
    } while (reset == CO_RESET_NOT);

    comm_printf("CAN finished...expecting to reset the device..\r\n");

    /* Final end loop that never ends */
    while (1) {
        NVIC_SystemReset();
    }
}

/**
 * \brief           Periodic tasks thread entry
 * \param[in]       arg: User argument
 */
static void
thread_canopen_periodic_entry(void* arg) {
    uint32_t time_old, time_current, timeDifference_us, max_sleep_time_us;

    comm_printf("CANopen periodic thread is running\r\n");

    co_drv_mutex_lock();                        /* Get access mutex */
    time_old = time_current = osKernelGetTickCount();
    max_sleep_time_us = 0;                      /* No sleep for very first time */
    while (1) {
        co_drv_mutex_unlock();                  /* Release mutex to allow other tasks to process */

        /*
         * This will block this thread for up to maximal time.
         *
         * This thread is woken up after timeout expires
         * or if main CANopen thread executes
         */
        CO_WAIT_SYNC_PERIODIC_THREAD(max_sleep_time_us / 1000);

        /* Get access mutex */
        co_drv_mutex_lock();

        /* Set time to maximum wait in next loop */
        max_sleep_time_us = (uint32_t)-1;

        /* Verify that everything is set */
        if (CO == NULL || !CO->CANmodule->CANnormal) {
            continue;
        }

        /* Get new time */
        time_current = osKernelGetTickCount();
        timeDifference_us = (time_current - time_old) * (1000000 / configTICK_RATE_HZ);
        time_old = time_current;

        /* For the moment lock interrupts for further processing */
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
            bool_t syncWas = false;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, timeDifference_us, &max_sleep_time_us);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, timeDifference_us, &max_sleep_time_us);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, timeDifference_us, &max_sleep_time_us);
#endif
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 3;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, MII_MDC_GPIO_Pin|MII_MDIO_GPIO_Pin|LED3_Pin|LED1_Pin
                          |LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MII_TXD3_Pin */
  GPIO_InitStruct.Pin = MII_TXD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(MII_TXD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MII_TXD1_Pin MII_TXD0_Pin MII_TX_EN_Pin */
  GPIO_InitStruct.Pin = MII_TXD1_Pin|MII_TXD0_Pin|MII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SD2_D3_Pin SD2_D2_Pin */
  GPIO_InitStruct.Pin = SD2_D3_Pin|SD2_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_SDMMC2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SD2_CMD_Pin SD2_CLK_Pin */
  GPIO_InitStruct.Pin = SD2_CMD_Pin|SD2_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PAR_VSYNC_Pin */
  GPIO_InitStruct.Pin = PAR_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(PAR_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MII_MDC_GPIO_Pin MII_MDIO_GPIO_Pin LED3_Pin LED1_Pin
                           LED2_Pin */
  GPIO_InitStruct.Pin = MII_MDC_GPIO_Pin|MII_MDIO_GPIO_Pin|LED3_Pin|LED1_Pin
                          |LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : EXPANDER_INT_Pin */
  GPIO_InitStruct.Pin = EXPANDER_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXPANDER_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DE_Pin LCD_B7_Pin LCD_B6_Pin LCD_B5_Pin
                           LCD_B4_Pin */
  GPIO_InitStruct.Pin = LCD_DE_Pin|LCD_B7_Pin|LCD_B6_Pin|LCD_B5_Pin
                          |LCD_B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : SD2_D1_Pin */
  GPIO_InitStruct.Pin = SD2_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;
  HAL_GPIO_Init(SD2_D1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_B2_Pin LCD_B3_Pin LCD_R5_Pin LCD_R6_Pin */
  GPIO_InitStruct.Pin = LCD_B2_Pin|LCD_B3_Pin|LCD_R5_Pin|LCD_R6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : PAR_D5_Pin */
  GPIO_InitStruct.Pin = PAR_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(PAR_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TAMPER_WKUP_KEY_Pin */
  GPIO_InitStruct.Pin = TAMPER_WKUP_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TAMPER_WKUP_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_HSYNC_Pin LCD_VSYNC_Pin LCD_CLK_Pin */
  GPIO_InitStruct.Pin = LCD_HSYNC_Pin|LCD_VSYNC_Pin|LCD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MII_MCO_Pin */
  GPIO_InitStruct.Pin = MII_MCO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(MII_MCO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MII_TX_CLK_Pin MII_TXD2_Pin MII_RXD0_Pin MII_RXD1_Pin */
  GPIO_InitStruct.Pin = MII_TX_CLK_Pin|MII_TXD2_Pin|MII_RXD0_Pin|MII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_STP_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_STP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MII_RX_CLK_Pin MII_RX_DV_Pin */
  GPIO_InitStruct.Pin = MII_RX_CLK_Pin|MII_RX_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAR_HSYNC_Pin PAR_PCLK_Pin */
  GPIO_InitStruct.Pin = PAR_HSYNC_Pin|PAR_PCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MII_RXD3_Pin MII_RXD2_Pin */
  GPIO_InitStruct.Pin = MII_RXD3_Pin|MII_RXD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS2_DM_Pin USB_FS2_DP_Pin */
  GPIO_InitStruct.Pin = USB_FS2_DM_Pin|USB_FS2_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
