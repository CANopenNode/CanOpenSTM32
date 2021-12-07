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
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_rcc.h"

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
        uint16_t swj;                           /*!< Number of SWJ Time-Quantas.
                                                        Can be any value between `1` and `128` for nominal
                                                        and any value between `1` and `16` for data */
        uint16_t ts1;                           /*!< Number of TS1 Time-Quantas.
                                                        Can be any value between `2` and `256` for nominal
                                                        and any value between `1` and `32` for data */
        uint16_t ts2;                           /*!< Number of TS2 Time-Quantas.
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

/* FDCAN handle object */
FDCAN_HandleTypeDef hfdcan1;

/*
 * Setup default config for 125kHz
 *
 * Settings assume FDCAN input kernel clock is 80 MHz.
 * See \ref fdcan_br_cfg_t for global options
 */
const static fdcan_br_cfg_t
fdcan_br_cfg = {
    .clk_presc = 4,                             /* Decrease input clock to 20 MHz */
    .br_nominal = {                             /* Configure br nominal to 125 kHz */
        .clk_presc = 8,                         /* Set nominal tq time to 2.5 MHz */
        .swj = 1,
        .ts1 = 16,
        .ts2 = 3
    },
};

/******************************************************************************/
void
CO_CANsetConfigurationMode(void *CANptr) {
    /* Put CAN module in configuration mode */
    if (CANptr != NULL) {
        HAL_FDCAN_Stop(CANptr);
    }
}

/******************************************************************************/
void
CO_CANsetNormalMode(CO_CANmodule_t *CANmodule) {
    /* Put CAN module in normal mode */
    if (CANmodule->CANptr != NULL && HAL_FDCAN_Start(CANmodule->CANptr) == HAL_OK) {
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
    FDCAN_ClkCalUnitTypeDef fdcan_clk = {0};

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /*
     * Application must set CAN pointer to FDCAN handle.
     *
     * Only FDCAN1 is supported in current revision.
     */
    if (CANptr != &hfdcan1) {
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
    hfdcan1.Instance = FDCAN1;
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_DeInit(&hfdcan1);

    /* Set FDCAN parameters */
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = ENABLE;
    hfdcan1.Init.ProtocolException = DISABLE;

    /* Setup nominal bit timing */
    hfdcan1.Init.NominalPrescaler = fdcan_br_cfg.br_nominal.clk_presc;
    hfdcan1.Init.NominalSyncJumpWidth = fdcan_br_cfg.br_nominal.swj;
    hfdcan1.Init.NominalTimeSeg1 = fdcan_br_cfg.br_nominal.ts1;
    hfdcan1.Init.NominalTimeSeg2 = fdcan_br_cfg.br_nominal.ts2;

    /*
     * Setup data bit timing.
     *
     * Used only if FD and BRS modes are globally enabled,
     * which is not the case for CANopen
     */
    hfdcan1.Init.DataPrescaler = hfdcan1.Init.NominalPrescaler;
    hfdcan1.Init.DataSyncJumpWidth = hfdcan1.Init.NominalSyncJumpWidth;
    hfdcan1.Init.DataTimeSeg1 = hfdcan1.Init.NominalTimeSeg1;
    hfdcan1.Init.DataTimeSeg2 = hfdcan1.Init.NominalTimeSeg2;

    /* Number of used filters */
    hfdcan1.Init.StdFiltersNbr = 0;
    hfdcan1.Init.ExtFiltersNbr = 0;

    /* Message part */
    hfdcan1.Init.MessageRAMOffset = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr = 64;          /* FIFO 0 for standard message ID */
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxBuffersNbr = 0;
    hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
    hfdcan1.Init.TxEventsNbr = 0;
    hfdcan1.Init.TxBuffersNbr = 0;              /* Do not use TX buffers */
    hfdcan1.Init.TxFifoQueueElmtsNbr = 32;      /* Max number of TX FIFO size */
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;   /* Max number of bytes per message in TX FIFO */
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        /* What should we return here? */
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Setup prescaler block config for FDCAN input module */
    switch (fdcan_br_cfg.clk_presc) {
        case 0:
        case 1: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV1; break;
        case 2: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV2; break;
        case 4: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV4; break;
        case 6: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV6; break;
        case 8: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV8; break;
        case 10: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV10; break;
        case 12: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV12; break;
        case 14: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV14; break;
        case 16: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV16; break;
        case 18: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV18; break;
        case 20: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV20; break;
        case 22: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV22; break;
        case 24: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV24; break;
        case 26: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV26; break;
        case 28: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV28; break;
        case 30: fdcan_clk.ClockDivider = FDCAN_CLOCK_DIV30; break;
        default: return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    fdcan_clk.ClockCalibration = FDCAN_CLOCK_CALIBRATION_DISABLE;
    if (HAL_FDCAN_ConfigClockCalibration(&hfdcan1, &fdcan_clk) != HAL_OK) {
        HAL_FDCAN_DeInit(&hfdcan1);
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /*
     * Configure global filter that is used as last check if message did not pass any of other filters:
     *
     * We do not rely on hardware filters in this example
     * and are performing software filters instead
     *
     * Accept non-matching standard ID messages
     * Reject non-matching extended ID messages
     */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Enable notifications */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            0
            | FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE
            | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY
            | FDCAN_IT_BUS_OFF
            | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
            | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0xFFFFFFFF) != HAL_OK) {
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
        HAL_FDCAN_Stop(CANmodule->CANptr);
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
    static FDCAN_TxHeaderTypeDef tx_hdr;
    uint8_t success = 0;

    /* Check if TX FIFO is ready to accept more messages */
    if (HAL_FDCAN_GetTxFifoFreeLevel(CANModule_local->CANptr) > 0) {
        /*
         * RTR flag is part of identifier value
         * hence it needs to be properly decoded
         */
        tx_hdr.Identifier = buffer->ident & CANID_MASK;
        tx_hdr.TxFrameType = (buffer->ident & FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
        tx_hdr.IdType = FDCAN_STANDARD_ID;
        tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
        tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
        tx_hdr.MessageMarker = 0;
        tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

        switch (buffer->DLC) {
            case 0: tx_hdr.DataLength = FDCAN_DLC_BYTES_0; break;
            case 1: tx_hdr.DataLength = FDCAN_DLC_BYTES_1; break;
            case 2: tx_hdr.DataLength = FDCAN_DLC_BYTES_2; break;
            case 3: tx_hdr.DataLength = FDCAN_DLC_BYTES_3; break;
            case 4: tx_hdr.DataLength = FDCAN_DLC_BYTES_4; break;
            case 5: tx_hdr.DataLength = FDCAN_DLC_BYTES_5; break;
            case 6: tx_hdr.DataLength = FDCAN_DLC_BYTES_6; break;
            case 7: tx_hdr.DataLength = FDCAN_DLC_BYTES_7; break;
            case 8: tx_hdr.DataLength = FDCAN_DLC_BYTES_8; break;
            default: /* Hard error... */ break;
        }

        /* Now add message to FIFO. Should not fail */
        success = HAL_FDCAN_AddMessageToTxFifoQ(CANmodule->CANptr, &tx_hdr, buffer->data) == HAL_OK;
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
prv_read_can_received_msg(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo, uint32_t fifo_isrs) {
    static FDCAN_RxHeaderTypeDef rx_hdr;
    CO_CANrxMsg_t rcvMsg;
    CO_CANrx_t *buffer = NULL;              /* receive message buffer from CO_CANmodule_t object. */
    uint16_t index;                         /* index of received message */
    uint32_t rcvMsgIdent;                   /* identifier of the received message */
    uint8_t messageFound = 0;

    /* Read received message from FIFO */
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_hdr, rcvMsg.data) != HAL_OK) {
        return;
    }

    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = rx_hdr.Identifier | (rx_hdr.RxFrameType == FDCAN_REMOTE_FRAME ? FLAG_RTR : 0x00);
    switch (rx_hdr.DataLength) {
        case FDCAN_DLC_BYTES_0: rcvMsg.dlc = 0; break;
        case FDCAN_DLC_BYTES_1: rcvMsg.dlc = 1; break;
        case FDCAN_DLC_BYTES_2: rcvMsg.dlc = 2; break;
        case FDCAN_DLC_BYTES_3: rcvMsg.dlc = 3; break;
        case FDCAN_DLC_BYTES_4: rcvMsg.dlc = 4; break;
        case FDCAN_DLC_BYTES_5: rcvMsg.dlc = 5; break;
        case FDCAN_DLC_BYTES_6: rcvMsg.dlc = 6; break;
        case FDCAN_DLC_BYTES_7: rcvMsg.dlc = 7; break;
        case FDCAN_DLC_BYTES_8: rcvMsg.dlc = 8; break;
        default: rcvMsg.dlc = 0; break;     /* Invalid length when more than 8 */
    }
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
HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        prv_read_can_received_msg(hfdcan, FDCAN_RX_FIFO0, RxFifo0ITs);
    }
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       RxFifo1ITs: indicates which Rx FIFO 0 interrupts are signaled.
 */
void
HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs) {
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
        prv_read_can_received_msg(hfdcan, FDCAN_RX_FIFO1, RxFifo1ITs);
    }
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       BufferIndexes: Bits of successfully sent TX buffers
 */
void
HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
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
 * \brief           Low-level microcontroller support init callback
 * \param[in]       fdcanHandle: FDCAN handle instance
 */
void
HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (fdcanHandle->Instance == FDCAN1) {
        /*
         * Use PLL1Q output for FDCAN clock
         *
         * System clock must output 80 MHz at PLL1Q output
         */
        LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PLL1Q);

        /* Peripheral clock enable */
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();

        /*
         * FDCAN1 GPIO Configuration
         *
         * PH13     ------> FDCAN1_TX
         * PH14     ------> FDCAN1_RX
         */
        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

        /* FDCAN1 interrupt Init */
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
    }
}

/**
 * \brief           Low-level microcontroller support deinit callback
 * \param[in]       fdcanHandle: FDCAN handle instance
 */
void
HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle) {
    if (fdcanHandle->Instance == FDCAN1) {
        /* Peripheral clock disable */
        __HAL_RCC_FDCAN_CLK_DISABLE();

        /*
         * FDCAN1 GPIO Configuration
         *
         * PH13     ------> FDCAN1_TX
         * PH14     ------> FDCAN1_RX
         */
        HAL_GPIO_DeInit(GPIOH, GPIO_PIN_13 | GPIO_PIN_14);

        /* FDCAN1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
    }
}

/**
 * \brief           This function handles FDCAN1 interrupt 0
 */
void
FDCAN1_IT0_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&hfdcan1);

    /* Wake-up application thread */
    CO_WAKEUP_APP_THREAD();
}

/**
 * \brief           This function handles FDCAN1 interrupt 1
 */
void
FDCAN1_IT1_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(&hfdcan1);

    /* Wake-up application thread */
    CO_WAKEUP_APP_THREAD();
}
