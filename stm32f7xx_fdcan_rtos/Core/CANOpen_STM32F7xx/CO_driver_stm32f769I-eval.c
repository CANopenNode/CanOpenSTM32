/*
 * CAN module object for STM32H7xx FDCAN peripheral IP.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Implementation Author:               Tilen Majerle <tilen@majerle.eu>
 */
#include "301/CO_driver.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_rcc.h"

/**
 * \brief           Configuration structure for FDCAN
 */
typedef struct {
    uint16_t clk_presc;                         /*!< Clock prescaler. Can be a any even number between `1` and `30`,
                                                        including `1`, indicating no prescaler */
    struct {
        uint16_t clk_presc;                     /*!< Block generator clock prescaler.
                                                        Can be any value between `1` and `512` for nominal
                                                        and any value between `1` and `32` for data */
        uint32_t swj;                           /*!< Number of SWJ Time-Quantas.
                                                        Can be any value between `1` and `128` for nominal
                                                        and any value between `1` and `16` for data */
        uint32_t ts1;                           /*!< Number of TS1 Time-Quantas.
                                                        Can be any value between `2` and `256` for nominal
                                                        and any value between `1` and `32` for data */
        uint32_t ts2;                           /*!< Number of TS2 Time-Quantas.
                                                        Can be any value between `2` and `128` for nominal
                                                        and any value between `1` and `16` for data */
    } br_nominal;
} fdcan_br_cfg_t;

/* Local CAN module object */
static CO_CANmodule_t* CANModule_local = NULL;  /* Local instance of global CAN module */

/* CAN masks for identifiers */
#define CANID_MASK                              0x07FF  /*!< CAN standard ID mask */
#define FLAG_RTR                                0x8000  /*!< RTR flag, part of identifier */

#if defined(USE_OS)
/* Mutex for atomic access */
static osMutexId_t co_mutex;

/* Semaphore for main app thread synchronization */
osSemaphoreId_t co_drv_app_thread_sync_semaphore;

/* Semaphore for periodic thread synchronization */
osSemaphoreId_t co_drv_periodic_thread_sync_semaphore;
#endif /* defined(USE_OS) */

/* CAN handle object */
CAN_HandleTypeDef CanHandle;

/*
 * Setup default config for 125kHz
 *
 * Settings assume FDCAN input kernel clock is 80 MHz.
 * See \ref fdcan_br_cfg_t for global options
 */
const static fdcan_br_cfg_t
fdcan_br_cfg = {
    .clk_presc = 1,                             /* Decrease input clock to 20 MHz */
    .br_nominal = {                             /* Configure br nominal to 125 kHz */
        .clk_presc = 6,                         /* Set nominal tq time to 2.5 MHz */
        .swj = CAN_SJW_1TQ,
        .ts1 = CAN_BS1_7TQ,
        .ts2 = CAN_BS2_1TQ
    },
};

/******************************************************************************/
void
CO_CANsetConfigurationMode(void *CANptr) {
    /* Put CAN module in configuration mode */
    if (CANptr != NULL) {
        HAL_CAN_Stop(CANptr);
    }
}

/******************************************************************************/
void
CO_CANsetNormalMode(CO_CANmodule_t *CANmodule) {
    /* Put CAN module in normal mode */
    if (CANmodule->CANptr != NULL && HAL_CAN_Start(CANmodule->CANptr) == HAL_OK) {
        CANmodule->CANnormal = true;
    }
}

/******************************************************************************/
CO_ReturnError_t
CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    CAN_FilterTypeDef  sFilterConfig;

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /*
     * Application must set CAN pointer to FDCAN handle.
     *
     * Only FDCAN1 is supported in current revision.
     */
    if (CANptr != &CanHandle) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Hold CANModule variable */
    CANModule_local = CANmodule;

    /* Configure object variables */
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;             /* Do not use HW filters */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    /* Reset all variables */
    for (uint16_t i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (uint16_t i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false;
    }

    /***************************************/
    /* STM32H7 FDCAN related configuration */
    /***************************************/

    /* Set instance at the beginning */
    CanHandle.Instance = CAN1;
    HAL_CAN_Stop(&CanHandle);
    HAL_CAN_DeInit(&CanHandle);

    /* Set CAN parameters */
    CanHandle.Init.Mode = CAN_MODE_NORMAL;
    CanHandle.Init.AutoRetransmission = ENABLE;
    CanHandle.Init.TimeTriggeredMode = DISABLE;
    CanHandle.Init.AutoBusOff = DISABLE;
    CanHandle.Init.AutoWakeUp = DISABLE;
    CanHandle.Init.ReceiveFifoLocked = DISABLE;
    CanHandle.Init.TransmitFifoPriority = DISABLE;

    /* Setup nominal bit timing */
    CanHandle.Init.Prescaler = fdcan_br_cfg.br_nominal.clk_presc;
    CanHandle.Init.SyncJumpWidth = fdcan_br_cfg.br_nominal.swj;//CAN_SJW_1TQ;
    CanHandle.Init.TimeSeg1 = fdcan_br_cfg.br_nominal.ts1;//CAN_BS1_7TQ;
    CanHandle.Init.TimeSeg2 = fdcan_br_cfg.br_nominal.ts2;//CAN_BS2_1TQ;

    if (HAL_CAN_Init(&CanHandle) != HAL_OK) {
        /* What should we return here? */
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /*##-2- Configure the CAN Filter ###########################################*/
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
    {
      /* Filter configuration Error */
      return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Enable notifications */
    if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |
                                                 CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR_WARNING |
                                                 CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF) != HAL_OK)
    {
      /* Notification Error */
      return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return CO_ERROR_NO;
}

#if defined(USE_OS)

/**
 * \brief           Create all OS objects for CANopen
 * \return          `1` on success, `0` otherwise
 */
uint8_t
co_drv_create_os_objects(void) {
    /* Create new mutex for OS context */
    if (co_mutex == NULL) {
        const osMutexAttr_t attr = {
            .attr_bits = osMutexRecursive,
            .name = "co"
        };
        co_mutex = osMutexNew(&attr);
    }

    /* Semaphore for main app thread synchronization */
    if (co_drv_app_thread_sync_semaphore == NULL) {
        const osSemaphoreAttr_t attr = {
                .name = "co_app_thread_sync"
        };
        co_drv_app_thread_sync_semaphore = osSemaphoreNew(1, 1, &attr);
    }

    /* Semaphore for periodic thread synchronization */
    if (co_drv_periodic_thread_sync_semaphore == NULL) {
        const osSemaphoreAttr_t attr = {
                .name = "co_periodic_thread_sync"
        };
        co_drv_periodic_thread_sync_semaphore = osSemaphoreNew(1, 1, &attr);
    }

    return 1;
}

/**
 * \brief           Lock mutex or wait to be available
 * \return          `1` on success, `0` otherwise
 */
uint8_t
co_drv_mutex_lock(void) {
    return osMutexAcquire(co_mutex, osWaitForever) == osOK;
}

/**
 * \brief           Release previously locked mutex
 * \return          `1` on success, `0` otherwise
 */
uint8_t
co_drv_mutex_unlock(void) {
    return osMutexRelease(co_mutex) == osOK;
}

#endif /* defined(USE_OS) */

/******************************************************************************/
void
CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
    if (CANmodule != NULL && CANmodule->CANptr != NULL) {
        HAL_CAN_Stop(CANmodule->CANptr);
    }
}

/******************************************************************************/
CO_ReturnError_t
CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if (CANmodule != NULL && object != NULL
            && CANrx_callback != NULL && index < CANmodule->rxSize) {
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /*
         * Configure global identifier, including RTR bit
         *
         * This is later used for RX operation match case
         */
        buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
        buffer->mask = (mask & CANID_MASK) | FLAG_RTR;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {
            __NOP();
        }
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t*
CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if (CANmodule != NULL && index < CANmodule->txSize) {
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
        buffer->ident = ((uint32_t)ident & CANID_MASK)
                        | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }
    return buffer;
}

/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       CANmodule: CAN module instance
 * \param[in]       buffer: Pointer to buffer to transmit
 */
static uint8_t
prv_send_can_message(CO_CANmodule_t* CANmodule, CO_CANtx_t *buffer) {
    static CAN_TxHeaderTypeDef TxHeader;
    static uint32_t TxMailbox;
    uint8_t success = 0;

    /* Check if TX FIFO is ready to accept more messages */
    if (HAL_CAN_GetTxMailboxesFreeLevel(CANModule_local->CANptr) > 0) {
        /*
         * RTR flag is part of identifier value
         * hence it needs to be properly decoded
         */
        TxHeader.StdId = buffer->ident & CANID_MASK;
        TxHeader.ExtId = 0x01;
        TxHeader.RTR = (buffer->ident & FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = buffer->DLC;
        TxHeader.TransmitGlobalTime = DISABLE;

        /* Now add message to FIFO. Should not fail */
        success = HAL_CAN_AddTxMessage(CANmodule->CANptr, &TxHeader, buffer->data, &TxMailbox) == HAL_OK;
    }
    return success;
}

/******************************************************************************/
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    /*
     * Send message to CAN network
     *
     * Lock interrupts for atomic operation
     */
    CO_LOCK_CAN_SEND(CANmodule);
    if (prv_send_can_message(CANmodule, buffer)) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    } else {
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}


/******************************************************************************/
void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule) {
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0) {
        for (uint16_t i = CANmodule->txSize; i > 0U; --i){
            if (CANmodule->txArray[i].bufferFull){
                if (CANmodule->txArray[i].syncFlag){
                    CANmodule->txArray[i].bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    if (tpdoDeleted) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
    * different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void
CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    uint32_t err;

    /* TODO: Get FDCAN protocol status... */

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0) {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }
        CANmodule->CANerrorStatus = status;
    }
}

/**
 * \brief           Read message from RX FIFO
 * \param           hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       fifo: Fifo number to use for read
 * \param[in]       fifo_isrs: List of interrupts for respected FIFO
 */
static void
prv_read_can_received_msg(CAN_HandleTypeDef* hfdcan, uint32_t fifo) {
    static CAN_RxHeaderTypeDef RxHeader;
    CO_CANrxMsg_t rcvMsg;
    CO_CANrx_t *buffer = NULL;              /* receive message buffer from CO_CANmodule_t object. */
    uint16_t index;                         /* index of received message */
    uint32_t rcvMsgIdent;                   /* identifier of the received message */
    uint8_t messageFound = 0;

    /* Read received message from FIFO */
    if (HAL_CAN_GetRxMessage(hfdcan, fifo, &RxHeader, rcvMsg.data) != HAL_OK) {
        return;
    }

    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = ((RxHeader.IDE == CAN_ID_STD) ? RxHeader.StdId : RxHeader.ExtId) | (RxHeader.RTR == CAN_RTR_REMOTE ? FLAG_RTR : 0x00);
    rcvMsg.dlc = RxHeader.DLC;
    rcvMsgIdent = rcvMsg.ident;

    /*
     * Hardware filters are not used for the moment
     * \todo: Implement hardware filters...
     */
    if (CANModule_local->useCANrxFilters) {
        __NOP();
    } else {
        /*
         * We are not using hardware filters, hence it is necessary
         * to manually match received message ID with all buffers
         */
        buffer = CANModule_local->rxArray;
        for (index = CANModule_local->rxSize; index > 0U; --index, ++buffer) {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
                messageFound = 1;
                break;
            }
        }
    }

    /* Call specific function, which will process the message */
    if (messageFound && buffer != NULL && buffer->CANrx_callback != NULL) {
        buffer->CANrx_callback(buffer->object, (void*) &rcvMsg);
    }
}

/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signaled.
 */
void
HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hfdcan) {
  prv_read_can_received_msg(hfdcan, CAN_RX_FIFO0);
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       RxFifo1ITs: indicates which Rx FIFO 0 interrupts are signaled.
 */
void
HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hfdcan) {
  prv_read_can_received_msg(hfdcan, CAN_RX_FIFO1);
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       BufferIndexes: Bits of successfully sent TX buffers
 */
static void TxMailbox0CompleteCallback(CAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
    CANModule_local->firstCANtxMessage = false; /* First CAN message (bootup) was sent successfully */
    CANModule_local->bufferInhibitFlag = false; /* Clear flag from previous message */
    if (CANModule_local->CANtxCount > 0U) {     /* Are there any new messages waiting to be send */
        CO_CANtx_t *buffer = &CANModule_local->txArray[0];  /* Start with first buffer handle */
        uint16_t i;

        /*
         * Try to send more buffers, process all empty ones
         *
         * This function is always called from interrupt,
         * however to make sure no preemption can happen, interrupts are anyway locked
         * (unless you can guarantee no higher priority interrupt will try to access to FDCAN instance and send data,
         *  then no need to lock interrupts..)
         */
        CO_LOCK_CAN_SEND(CANModule_local);
        for (i = CANModule_local->txSize; i > 0U; --i, ++buffer) {
            /* Try to send message */
            if (buffer->bufferFull) {
                if (prv_send_can_message(CANModule_local, buffer)) {
                    buffer->bufferFull = false;
                    CANModule_local->CANtxCount--;
                    CANModule_local->bufferInhibitFlag = buffer->syncFlag;
                }
            }
        }
        /* Clear counter if no more messages */
        if (i == 0U) {
            CANModule_local->CANtxCount = 0U;
        }
        CO_UNLOCK_CAN_SEND(CANModule_local);
    }
}

/**
 * @brief
 *
 * @param hcan
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
  TxMailbox0CompleteCallback(hcan, 0);
}

/**
 * @brief
 *
 * @param hcan
 */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
  TxMailbox0CompleteCallback(hcan, 1);
}

/**
 * @brief
 *
 * @param hcan
 */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
  TxMailbox0CompleteCallback(hcan, 2);
}

/**
 * \brief           Low-level microcontroller support init callback
 * \param[in]       fdcanHandle: FDCAN handle instance
 */
void
HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA12     ------> CAN1_TX
    PA11     ------> CAN1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  }
}

/**
 * \brief           Low-level microcontroller support deinit callback
 * \param[in]       fdcanHandle: FDCAN handle instance
 */
void
HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();

        /*
         * FDCAN1 GPIO Configuration
         *
         * PH13     ------> FDCAN1_TX
         * PH14     ------> FDCAN1_RX
         */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

        /* FDCAN1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    }
}

/**
 * \brief           This function handles FDCAN1 interrupt 0
 */
void
CAN1_TX_IRQHandler(void) {
  HAL_CAN_IRQHandler(&CanHandle);

    /* Wake-up application thread */
    CO_WAKEUP_APP_THREAD();
}

/**
 * \brief           This function handles FDCAN1 interrupt 1
 */
void
CAN1_RX0_IRQHandler(void) {
  HAL_CAN_IRQHandler(&CanHandle);

    /* Wake-up application thread */
    CO_WAKEUP_APP_THREAD();
}

/**
 * \brief           This function handles FDCAN1 interrupt 1
 */
void
CAN1_RX1_IRQHandler(void) {
  HAL_CAN_IRQHandler(&CanHandle);

    /* Wake-up application thread */
    CO_WAKEUP_APP_THREAD();
}
