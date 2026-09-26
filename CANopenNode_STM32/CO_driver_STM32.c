/*
 * CAN module object for STM32 (FD)CAN peripheral IP.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Hamed Jafarzadeh 	2022
 * 				Tilen Marjerle		2021
 * 				Janez Paternoster	2020
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
#include "CO_app_STM32.h"

/**
 * \brief           We use the local can module pointer,
 *                  that is later used in the callbacks.
 *
 * We assume only one canopen instance is used at all times.
 * It is declared as volatile, invoked from callbacks and interrupts
 */
static CO_CANmodule_t* volatile CANModule_local = NULL; /* Local instance of global CAN module */

/* CAN masks for identifiers */
#define CANID_MASK 0x07FF /*!< CAN standard ID mask */
#define FLAG_RTR   0x8000 /*!< RTR flag, part of identifier */

#ifdef CO_STM32_FDCAN_Driver
#ifndef FDCAN_BUFFER_INDEXES
#if defined(FDCAN_TX_BUFFER31)
#define FDCAN_BUFFER_INDEXES 0xFFFFFFFFU
#elif defined(FDCAN_TX_BUFFER2)
#define FDCAN_BUFFER_INDEXES FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2
#else
#define FDCAN_BUFFER_INDEXES 0xFFFFFFFFU
#warning "FDCAN_BUFFER_INDEXES not defined"
#endif
#endif
#endif /* CO_STM32_FDCAN_Driver */

/**
 * \brief           Returns the HAL instance pointer for the [FD]CAN instance for the MCU
 *                  hardware implementation from the CO_CanHandle CanOpen instance
 * \note            It returns the pointer to the hardware peripheral instance,
 *                  of type `can_periph_handle_t*` which is set depending on the CAN or FDCAN availability in the
 *                  product silicon
 */
#define GET_CAN_PERIPH_HANDLE(_co_can_module_instance_)                                                                \
    (((CANopenNodeSTM32*)(_co_can_module_instance_)->CANptr)->CANHandle)

/**
 * \brief           Convert buffer DLC number to the FDCAN peripheral marcos
 * \param           dlc: DLC number between 0 to 8
 */
#define FDCAN_DLC_NUM_TO_PERIPH_VALUE(dlc)                                                                             \
    ((dlc) == 0)                                                                                                       \
        ? FDCAN_DLC_BYTES_0                                                                                            \
        : (((dlc) == 1)                                                                                                \
               ? FDCAN_DLC_BYTES_1                                                                                     \
               : (((dlc) == 2)                                                                                         \
                      ? FDCAN_DLC_BYTES_2                                                                              \
                      : (((dlc) == 3)                                                                                  \
                             ? FDCAN_DLC_BYTES_3                                                                       \
                             : (((dlc) == 4) ? FDCAN_DLC_BYTES_4                                                       \
                                             : (((dlc) == 5) ? FDCAN_DLC_BYTES_5                                       \
                                                             : (((dlc) == 6) ? FDCAN_DLC_BYTES_6                       \
                                                                             : (((dlc) == 7)   ? FDCAN_DLC_BYTES_7     \
                                                                                : ((dlc) == 8) ? FDCAN_DLC_BYTES_8     \
                                                                                               : (0))))))))

/**
 * \brief           Converts an FDCAN peripheral DLC value (FDCAN_DLC_BYTES_x) back into
 *                  the actual number of data bytes it represents
 */
#define FDCAN_DLC_PERIPH_TO_NUM(dlc)                                                                                   \
    ((dlc) == FDCAN_DLC_BYTES_0)                                                                                       \
        ? 0                                                                                                            \
        : (((dlc) == FDCAN_DLC_BYTES_1)                                                                                \
               ? 1                                                                                                     \
               : (((dlc) == FDCAN_DLC_BYTES_2)                                                                         \
                      ? 2                                                                                              \
                      : (((dlc) == FDCAN_DLC_BYTES_3)                                                                  \
                             ? 3                                                                                       \
                             : (((dlc) == FDCAN_DLC_BYTES_4)                                                           \
                                    ? 4                                                                                \
                                    : (((dlc) == FDCAN_DLC_BYTES_5)                                                    \
                                           ? 5                                                                         \
                                           : (((dlc) == FDCAN_DLC_BYTES_6)                                             \
                                                  ? 6                                                                  \
                                                  : (((dlc) == FDCAN_DLC_BYTES_7)   ? 7                                \
                                                     : ((dlc) == FDCAN_DLC_BYTES_8) ? 8                                \
                                                                                    : (0))))))))

#ifdef CO_STM32_FDCAN_Driver
static void prv_fdcan_bus_off_check_reset(FDCAN_HandleTypeDef* hfdcan);
#endif /* CO_STM32_FDCAN_Driver */

/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       CANmodule: CAN module instance
 * \param[in]       buffer: Pointer to buffer to transmit
 * \return          `1` on success, `0` otherwise
 */
static uint8_t
prv_send_can_message(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    uint8_t success = 0;

    /* Check if TX FIFO is ready to accept more messages */
#ifdef CO_STM32_FDCAN_Driver
    static FDCAN_TxHeaderTypeDef tx_hdr;

    if (HAL_FDCAN_GetTxFifoFreeLevel(GET_CAN_PERIPH_HANDLE(CANmodule)) > 0) {
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

        tx_hdr.DataLength = FDCAN_DLC_NUM_TO_PERIPH_VALUE(buffer->DLC);

        /* Now add message to FIFO. Should not fail */
        success = HAL_FDCAN_AddMessageToTxFifoQ(GET_CAN_PERIPH_HANDLE(CANmodule), &tx_hdr, buffer->data) == HAL_OK;
    }
#else
    static CAN_TxHeaderTypeDef tx_hdr;
    /* Check if TX FIFO is ready to accept more messages */
    if (HAL_CAN_GetTxMailboxesFreeLevel(GET_CAN_PERIPH_HANDLE(CANmodule)) > 0) {
        uint32_t TxMailboxNum = 0;

        /*
         * RTR flag is part of identifier value
         * hence it needs to be properly decoded
         */
        tx_hdr.ExtId = 0u;
        tx_hdr.IDE = CAN_ID_STD;
        tx_hdr.DLC = buffer->DLC;
        tx_hdr.StdId = buffer->ident & CANID_MASK;
        tx_hdr.RTR = (buffer->ident & FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;

        /* Now add message to FIFO. Should not fail */
        success =
            HAL_CAN_AddTxMessage(GET_CAN_PERIPH_HANDLE(CANmodule), &tx_hdr, buffer->data, &TxMailboxNum) == HAL_OK;
    }
#endif
    return success;
}

/**
 * \brief           Read message from RX FIFO
 * \param           CANmodule: CAN module instance, typically from the local pointer
 * \param[in]       fifo: Fifo number to use for read
 * \param[in]       fifo_isrs: List of interrupts for respected FIFO
 */
static void
prv_read_can_received_msg(CO_CANmodule_t* CANmodule, uint32_t fifo, uint32_t fifo_isrs) {
    CO_CANrxMsg_t rcvMsg;
    CO_CANrx_t* buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
    uint16_t index;            /* index of received message */
    uint32_t rcvMsgIdent;      /* identifier of the received message */
    uint8_t messageFound = 0;
    can_periph_handle_t* mcu_canhandle = GET_CAN_PERIPH_HANDLE(CANmodule);

#ifdef CO_STM32_FDCAN_Driver
    /*
     * Write received message to the temporary 64-bytes buffer.
     * This is to ensure that the CAN nodes that do not comply with the newer CAN standards
     * don't send wrong message with the wrong DLC value. This is a safety measure to avoid buffer overflow.
     *
     * Check the FDCAN implementation for STM32 in their respective reference manual.
     */
    static FDCAN_RxHeaderTypeDef rx_hdr;
    static uint8_t rx_data[64];

    /* Read received message from FIFO */
    if (HAL_FDCAN_GetRxMessage(mcu_canhandle, fifo, &rx_hdr, rx_data) != HAL_OK) {
        return;
    }

    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = rx_hdr.Identifier | (rx_hdr.RxFrameType == FDCAN_REMOTE_FRAME ? FLAG_RTR : 0x00);
    rcvMsg.dlc = FDCAN_DLC_PERIPH_TO_NUM(rx_hdr.DataLength); /* Invalid length (more than 8) resolves to 0 */
    if (rcvMsg.dlc > 0) {
        memcpy(rcvMsg.data, rx_data, rcvMsg.dlc);
    }
    rcvMsgIdent = rcvMsg.ident;
#else
    static CAN_RxHeaderTypeDef rx_hdr;

    /* Read received message from FIFO */
    if (HAL_CAN_GetRxMessage(mcu_canhandle, fifo, &rx_hdr, rcvMsg.data) != HAL_OK) {
        return;
    }
    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = rx_hdr.StdId | (rx_hdr.RTR == CAN_RTR_REMOTE ? FLAG_RTR : 0x00);
    rcvMsg.dlc = rx_hdr.DLC;
    rcvMsgIdent = rcvMsg.ident;
#endif

    /*
     * Hardware filters are not used for the moment
     * \todo: Implement hardware filters...
     */
    if (CANmodule->useCANrxFilters) {
        __BKPT(0);
    } else {
        /*
         * We are not using hardware filters, hence it is necessary
         * to manually match received message ID with all buffers
         */
        buffer = CANmodule->rxArray;
        for (index = CANmodule->rxSize; index > 0U; --index, ++buffer) {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
                messageFound = 1;
                break;
            }
        }
    }

    /* Call specific function, which will process the message */
    if (messageFound && buffer != NULL && buffer->CANrx_callback != NULL) {
        buffer->CANrx_callback(buffer->object, &rcvMsg);
    }
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 * \param[in]       MailboxNumber: the mailbox number that has been transmitted
 */
static void
prv_process_tx_complete(CO_CANmodule_t* CANmodule, uint32_t MailboxNumber) {
    CANmodule->firstCANtxMessage = false; /* First CAN message (bootup) was sent successfully */
    CANmodule->bufferInhibitFlag = false; /* Clear flag from previous message */

    /*
     * Try to send more buffers, process all empty ones
     *
     * This function is always called from interrupt,
     * however to make sure no preemption can happen, interrupts are anyway locked
     * (unless you can guarantee no higher priority interrupt will try to access to CAN instance and send data,
     *  then no need to lock interrupts..)
     */
    CO_LOCK_CAN_SEND(CANmodule);
    if (CANmodule->CANtxCount > 0U) {                /* Are there any new messages waiting to be send */
        CO_CANtx_t* buffer = &CANmodule->txArray[0]; /* Start with first buffer handle */
        for (size_t idx = CANmodule->txSize; idx > 0U; --idx, ++buffer) {
            /* Try to send message */
            if (buffer->bufferFull) {
                if (prv_send_can_message(CANmodule, buffer)) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                } else {
                    break; // if we could not send the message, break out of the loop (the tx buffers are full)
                }
            }
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
}

/******************************************************************************/
/* CanOpenNode specific implementation functions                              */
/******************************************************************************/

/**
 * \brief           Put the CAN in the configuration mode.
 *
 * \param           CANptr: Custom \ref CANopenNodeSTM32 structure pointer
 */
void
CO_CANsetConfigurationMode(void* CANptr) {
    CANopenNodeSTM32* customhandle = CANptr;

    /* Put CAN module in configuration mode */
    if (customhandle != NULL) {
        can_periph_handle_t* periphhandle = customhandle->CANHandle;
#ifdef CO_STM32_FDCAN_Driver
        HAL_FDCAN_Stop(periphhandle);
#else
        HAL_CAN_Stop(periphhandle);
#endif
    }
}

/******************************************************************************/

/**
 * \brief           Set CAN peripheral to the normal mode
 *
 * \param           CANmodule: CanOpen module, where we store also the pointer
 *                      to our custom STM32 structure
 */
void
CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {
    /* Put CAN module in normal mode */
    if (CANmodule != NULL && CANmodule->CANptr != NULL) {
        can_periph_handle_t* periphhandle = GET_CAN_PERIPH_HANDLE(CANmodule);
        HAL_StatusTypeDef status;

#ifdef CO_STM32_FDCAN_Driver
        status = HAL_FDCAN_Start(periphhandle);
#else
        status = HAL_CAN_Start(periphhandle);
#endif

        if (status == HAL_OK) {
            CANmodule->CANnormal = true;
        }
    }
}

/******************************************************************************/

/**
 * \brief           Initialize the CANmodule object
 *
 * \param           CANmodule
 * \param           CANptr: CANptr is the same variable we pass to the CO_CANinit and should be
 *                      of the type \ref CANopenNodeSTM32 for this implementation
 * \param           rxArray
 * \param           rxSize
 * \param           txArray
 * \param           txSize
 * \param           CANbitRate
 * \return
 */
CO_ReturnError_t
CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate) {
    CANopenNodeSTM32* canstm32handle = CANptr;

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL || canstm32handle == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* CANptr is our STM32 type */
    CANmodule->CANptr = CANptr;

    /* Keep a local copy of CANModule */
    CANModule_local = CANmodule;

    /* Configure object variables */
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false; /* Do not use HW filters */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    /* Reset all variables */
    for (uint16_t idx = 0U; idx < rxSize; idx++) {
        CO_CANrx_t* const msg = &rxArray[idx];

        msg->ident = 0U;
        msg->mask = 0xFFFFU;
        msg->object = NULL;
        msg->CANrx_callback = NULL;
    }
    for (uint16_t idx = 0U; idx < txSize; idx++) {
        CO_CANtx_t* const msg = &txArray[idx];

        msg->bufferFull = false;
    }

    /***************************************/
    /* STM32 related configuration */
    /***************************************/
    if (canstm32handle->HWInitFunction != NULL) {
        canstm32handle->HWInitFunction();
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
#ifdef CO_STM32_FDCAN_Driver
    if (HAL_FDCAN_ConfigGlobalFilter(canstm32handle->CANHandle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
        != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#else /* CO_STM32_FDCAN_Driver */
    CAN_FilterTypeDef FilterConfig;
#if defined(CAN)
    FilterConfig.FilterBank = 0;
#else
    if ((GET_CAN_PERIPH_HANDLE(CANmodule))->Instance == CAN1) {
        FilterConfig.FilterBank = 0;
    } else {
        FilterConfig.FilterBank = 14;
    }
#endif
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterIdHigh = 0x0;
    FilterConfig.FilterIdLow = 0x0;
    FilterConfig.FilterMaskIdHigh = 0x0;
    FilterConfig.FilterMaskIdLow = 0x0;
    FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    FilterConfig.FilterActivation = ENABLE;
    FilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(canstm32handle->CANHandle, &FilterConfig) != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#endif /* !CO_STM32_FDCAN_Driver */

    /* Enable notifications */
    /* Activate the CAN notification interrupts */
#ifdef CO_STM32_FDCAN_Driver
    if (HAL_FDCAN_ActivateNotification(canstm32handle->CANHandle,
                                       0 | FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                       FDCAN_BUFFER_INDEXES)
        != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#else
    if (HAL_CAN_ActivateNotification(canstm32handle->CANHandle, CAN_IT_RX_FIFO0_MSG_PENDING
                                                                    | CAN_IT_RX_FIFO1_MSG_PENDING
                                                                    | CAN_IT_TX_MAILBOX_EMPTY)
        != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#endif

    return CO_ERROR_NO;
}

/******************************************************************************/
void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    /* Disable the module */
    if (CANmodule != NULL && CANmodule->CANptr != NULL) {
        can_periph_handle_t* periphhandle = GET_CAN_PERIPH_HANDLE(CANmodule);
        HAL_StatusTypeDef status;

#ifdef CO_STM32_FDCAN_Driver
        status = HAL_FDCAN_Stop(periphhandle);
#else
        status = HAL_CAN_Stop(periphhandle);
#endif
        if (status != HAL_OK) {
            (void)status; /* might be unused */
        }
    }
}

/******************************************************************************/
CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
                   void (*CANrx_callback)(void* object, void* message)) {
    CO_ReturnError_t ret = CO_ERROR_NO;

    if (CANmodule != NULL && object != NULL && CANrx_callback != NULL && index < CANmodule->rxSize) {
        CO_CANrx_t* buffer = &CANmodule->rxArray[index];

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
CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    CO_CANtx_t* buffer = NULL;

    if (CANmodule != NULL && index < CANmodule->txSize) {
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
        buffer->ident = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }
    return buffer;
}

/******************************************************************************/
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
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
        /* Only increment count if buffer wasn't already full */
        if (!buffer->bufferFull) {
            buffer->bufferFull = true;
            CANmodule->CANtxCount++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

/******************************************************************************/
void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0) {
        CO_CANtx_t* buffer = &CANmodule->txArray[0];
        for (uint16_t idx = CANmodule->txSize; idx > 0U; idx--) {
            if (buffer->bufferFull) {
                if (buffer->syncFlag) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    if (tpdoDeleted) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/******************************************************************************/

void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
    uint32_t err = 0;
    can_periph_handle_t* periphhandle = GET_CAN_PERIPH_HANDLE(CANmodule);

    // CANOpen just care about Bus_off, Warning, Passive and Overflow
    // I didn't find overflow error register in STM32, if you find it please let me know

#ifdef CO_STM32_FDCAN_Driver
    err = periphhandle->Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EW | FDCAN_PSR_EP);
    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;
        if (err & FDCAN_PSR_BO) {
            status |= CO_CAN_ERRTX_BUS_OFF;

            /* FDCAN does not auto start bus-off, start it here */
            prv_fdcan_bus_off_check_reset(periphhandle);
        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);

            if (err & FDCAN_PSR_EW) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }

            if (err & FDCAN_PSR_EP) {
                status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }

            /* If transmitter is not passive, clear also the (non-latching) TX overflow.
             * It is set by CO_CANsend(), when the driver TX buffer was already full. It
             * must stay set while the bus is in a degraded state, so CO_EM_process() can
             * report it, but it must not latch forever - otherwise CO_EM_CAN_TX_OVERFLOW
             * keeps the communication bit of the error register (0x1001) set and, with
             * CO_NMT_ERR_ON_ERR_REG, the node can never enter NMT operational state.
             * Same pattern as CANopenNode/example/CO_driver_blank.c. */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0U) {
                status &= 0xFFFFU ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }
        CANmodule->CANerrorStatus = status;
    }
#else
    err = periphhandle->Instance->ESR & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF);
    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;
        if (err & CAN_ESR_BOFF) {
            status |= CO_CAN_ERRTX_BUS_OFF;
            // In this driver, we assume that auto bus recovery is activated ! so this error will eventually handled
            // automatically. This has to be enabled in the settings
        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);

            if (err & CAN_ESR_EWGF) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }

            if (err & CAN_ESR_EPVF) {
                status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }

            /* If transmitter is not passive, clear also the (non-latching) TX overflow,
             * see the CO_STM32_FDCAN_Driver branch above. */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0U) {
                status &= 0xFFFFU ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        CANmodule->CANerrorStatus = status;
    }
#endif
}

/******************************************************************************/
/* Hardware specific callback functions implemented here                      */
/******************************************************************************/

#ifdef CO_STM32_FDCAN_Driver

/**
 * \brief           Check for BUS-OFF and reinitialize the FDCAN peripheral instance
 *                  IP will then wait for recessive bits before resuming the operation
 *
 * \param           hfdcan
 */
static void
prv_fdcan_bus_off_check_reset(FDCAN_HandleTypeDef* hfdcan) {
    FDCAN_ProtocolStatusTypeDef protocolStatus = {0};

    HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
    if (protocolStatus.BusOff) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
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
        prv_read_can_received_msg(CANModule_local, FDCAN_RX_FIFO0, RxFifo0ITs);
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
        prv_read_can_received_msg(CANModule_local, FDCAN_RX_FIFO1, RxFifo1ITs);
    }
}

/**
 * \brief           Error status callback
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       ErrorStatusITs indicates which Error Status interrupts are signaled.
 *                      This parameter can be any combination of @arg FDCAN_Error_Status_Interrupts.
 *
 * Implements manual FDCAN Bus-Off recovery as described in
 * https://community.st.com/stm32-mcus-60/how-to-recover-from-bus-off-state-with-fdcan-on-stm32-mcus-158678.
 */
void
HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs) {
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT); // Clear INIT bit to recover from Bus-Off
    }
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       BufferIndexes: Bits of successfully sent TX buffers
 */
void
HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t BufferIndexes) {
    prv_process_tx_complete(CANModule_local, BufferIndexes);
}

/**
 * \brief           Error callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN
 */
void
HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan) {
    (void)hfdcan;

    /* Do we do it here or in process only? */
    /* prv_fdcan_bus_off_check_reset(hfdcan); */
}

/**
 * \brief           Error status callback
 *
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       ErrorStatusITs: indicates which Error Status interrupts are signaled.
 *                      This parameter can be any combination of \arg FDCAN_Error_Status_Interrupts
 */
void
HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs) {
    (void)hfdcan;
    (void)ErrorStatusITs;

    /* Do we do it here or in process only? */
    /* prv_fdcan_bus_off_check_reset(hfdcan); */
}

#else /* CO_STM32_FDCAN_Driver */

/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void
HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    prv_read_can_received_msg(CANModule_local, CAN_RX_FIFO0, 0);
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void
HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    prv_read_can_received_msg(CANModule_local, CAN_RX_FIFO1, 0);
}

/**
 * \brief           TX mailbox 0 message sent
 *
 * \param           hcan
 */
void
HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan) {
    prv_process_tx_complete(CANModule_local, CAN_TX_MAILBOX0);
}

/**
 * \brief           TX mailbox 1 message sent
 *
 * \param           hcan
 */
void
HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan) {
    prv_process_tx_complete(CANModule_local, CAN_TX_MAILBOX1);
}

/**
 * \brief           TX mailbox 2 message sent
 *
 * \param           hcan
 */
void
HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan) {
    prv_process_tx_complete(CANModule_local, CAN_TX_MAILBOX2);
}

#endif /* !CO_STM32_FDCAN_Driver */
