/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver_STM32.c
 * @ingroup     CO_driver
 * @author      Hamed Jafarzadeh
 * @copyright   2004 - 2022 Hamed Jafarzadeh
 *
 * This file is the implementation of the CANOpenNode for STM32 Microcontrollers. This file is part of CANopenNode, an opensource CANopen Stack.
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
 */

#include "301/CO_driver.h"
#include "can.h"

static CO_CANmodule_t* DefaultCANModule = NULL;

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;


void prepareTxHeader(CAN_TxHeaderTypeDef *TxHeader, CO_CANtx_t *buffer)
{
	/* Map buffer data to the HAL CAN tx header data*/
	TxHeader->ExtId = 0u;
	TxHeader->IDE = 0;
	TxHeader->DLC = buffer->DLC;
	TxHeader->StdId = ( buffer->ident >> 2 );
	TxHeader->RTR = ( buffer->ident & 0x2 );
}
/* \brief 	Cube MX callbacks for Fifo0 and Fifo1
 * \details It is assumed that only one CANmodule is (CO->CANmodule[0]) is used.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CO_CANinterrupt_RX(DefaultCANModule);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

		CO_CANinterrupt_RX(DefaultCANModule);
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	CO_CANinterrupt_TX(DefaultCANModule);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	CO_CANinterrupt_TX(DefaultCANModule);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	CO_CANinterrupt_TX(DefaultCANModule);
}



/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr) {
	/* Put CAN module in configuration mode */
	/* HAL Responsible for that apparently */
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule) {
	/* Put CAN module in normal mode */

	CANmodule->CANnormal = true;

	CO_ReturnError_t Error = CO_ERROR_NO;
	if (HAL_CAN_Start(CANmodule->CANptr) != HAL_OK) {
		/* Start Error */
		Error = CO_ERROR_ILLEGAL_ARGUMENT;
		printf("Start CAN Error !\n");

	} else{
		printf("Start CAN OK !\n");
	}

	if (HAL_CAN_ActivateNotification(CANmodule->CANptr,
	CAN_IT_RX_FIFO0_MSG_PENDING |
	CAN_IT_RX_FIFO1_MSG_PENDING |
	CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		/* Notification Error */
		Error = CO_ERROR_ILLEGAL_ARGUMENT;
		printf("Setting CAN notifications error !");
	} else{
		printf("CAN Notification active !\n");
	}

	CANmodule->CANnormal = true;
	return Error;

}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule, void      *HALCanObject,
		CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
		uint16_t txSize, uint16_t CANbitRate) {
	uint16_t i;

	/* verify arguments */
	if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	/* Configure object variables */
	CANmodule->CANptr = (CAN_HandleTypeDef *)HALCanObject;
	CANmodule->rxArray = rxArray;
	CANmodule->rxSize = rxSize;
	CANmodule->txArray = txArray;
	CANmodule->txSize = txSize;
	CANmodule->CANerrorStatus = 0;
	CANmodule->CANnormal = false;
	CANmodule->useCANrxFilters = false; /* microcontroller dependent */
	CANmodule->bufferInhibitFlag = false;
	CANmodule->firstCANtxMessage = true;
	CANmodule->CANtxCount = 0U;
	CANmodule->errOld = 0U;

	for (i = 0U; i < rxSize; i++) {
		rxArray[i].ident = 0U;
		rxArray[i].mask = 0xFFFFU;
		rxArray[i].object = NULL;
		rxArray[i].CANrx_callback = NULL;
	}
	for (i = 0U; i < txSize; i++) {
		txArray[i].bufferFull = false;
	}

	/* Configure CAN module registers */
	CO_CANmodule_disable(CANmodule);
	HAL_CAN_MspDeInit(CANmodule->CANptr);
	HAL_CAN_MspInit(CANmodule->CANptr); /* NVIC and GPIO */

	CANmodule->CANptr->Init.Mode = CAN_MODE_NORMAL;
	CANmodule->CANptr->Init.SyncJumpWidth = CAN_SJW_1TQ;
	CANmodule->CANptr->Init.TimeTriggeredMode = DISABLE;
	CANmodule->CANptr->Init.AutoBusOff = DISABLE;
	CANmodule->CANptr->Init.AutoWakeUp = DISABLE;
	CANmodule->CANptr->Init.AutoRetransmission = ENABLE;
	CANmodule->CANptr->Init.ReceiveFifoLocked = DISABLE;
	CANmodule->CANptr->Init.TransmitFifoPriority = DISABLE;
	CANmodule->CANptr->Init.TimeSeg2 = CAN_BS2_1TQ;
	CANmodule->CANptr->Init.TimeSeg1 = CAN_BS1_10TQ;

//	  hcan.Instance = CAN;
//	  hcan.Init.Prescaler = 8;
//	  hcan.Init.Mode = CAN_MODE_NORMAL;
//	  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
//	  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
//	  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
//	  hcan.Init.TimeTriggeredMode = DISABLE;
//	  hcan.Init.AutoBusOff = DISABLE;
//	  hcan.Init.AutoWakeUp = DISABLE;
//	  hcan.Init.AutoRetransmission = DISABLE;
//	  hcan.Init.ReceiveFifoLocked = DISABLE;
//	  hcan.Init.TransmitFifoPriority = DISABLE;

	/* Configure CAN timing */
	uint32_t Prescaler = 500;

	switch (CANbitRate) {
	case 1000:
		Prescaler = 5;
		break;
	case 500:
		Prescaler = 10;
		break;
	case 250:
		Prescaler = 12;
		break;
	case 125:
		Prescaler = 40;
		break;
	case 100:
		Prescaler = 50;
		break;
	case 50:
		Prescaler = 100;
		break;
	case 20:
		Prescaler = 250;
		break;
	case 10:
		Prescaler = 500;
		break;

	default:
		return CO_ERROR_ILLEGAL_BAUDRATE;
	}

	CANmodule->CANptr->Init.Prescaler = Prescaler;

	if (HAL_CAN_Init(CANmodule->CANptr) != HAL_OK) {
		//_Error_Handler(__FILE__, __LINE__);
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	DefaultCANModule = CANmodule;


	/* Configure CAN module hardware filters */
	if (CANmodule->useCANrxFilters) {
		/* CAN module filters are used, they will be configured with */
		/* CO_CANrxBufferInit() functions, called by separate CANopen */
		/* init functions. */
		/* Configure all masks so, that received message must match filter */
		// TODO : [Future] Filter Implemntation
		__asm__ volatile ("BKPT");
		return CO_ERROR_ILLEGAL_ARGUMENT;
	} else {
		/* CAN module filters are not used, all messages with standard 11-bit */
		/* identifier will be received */
		/* Configure mask 0 so, that all messages with standard identifier are accepted */
		CAN_FilterTypeDef FilterConfig;
		FilterConfig.FilterBank = 0;
		FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		FilterConfig.FilterIdHigh = 0x0;
		FilterConfig.FilterIdLow = 0x0;
		FilterConfig.FilterMaskIdHigh = 0x0;
		FilterConfig.FilterMaskIdLow = 0x0;
		FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		FilterConfig.FilterActivation = ENABLE;
		FilterConfig.SlaveStartFilterBank = 14;


		if (HAL_CAN_ConfigFilter(CANmodule->CANptr, &FilterConfig)
				!= HAL_OK) {
			return CO_ERROR_ILLEGAL_ARGUMENT;
			printf("HAL_CAN_ConfigFilter error!\n");
		} else {
			printf("HAL_CAN_ConfigFilter OK!\n");
			; //do nothing
		}
	}

	/* configure CAN interrupt registers */
	// [HJ] Probably handled by CubeMX

	return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
	if (CANmodule != NULL && CANmodule->CANptr != NULL) {
		/* turn off the module */
		HAL_CAN_DeactivateNotification(CANmodule->CANptr,
		CAN_IT_RX_FIFO0_MSG_PENDING |
		CAN_IT_RX_FIFO1_MSG_PENDING |
		CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_Stop(CANmodule->CANptr);
	}
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
	CO_ReturnError_t ret = CO_ERROR_NO;

	if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL)
			&& (index < CANmodule->rxSize)) {
		/* buffer, which will be configured */
		CO_CANrx_t *buffer = &CANmodule->rxArray[index];

		/* Configure object variables */
		buffer->object = object;
		buffer->CANrx_callback = CANrx_callback;

		/* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
		buffer->ident &= 0x7FF;
		buffer->ident = ident << 2;
		if (rtr) buffer->ident |= 0x02;

		buffer->mask = (mask & 0x07FFU) | 0x0002U;

		/* Set CAN hardware module filter and mask. */
		if (CANmodule->useCANrxFilters) {

		} else {
			CAN_FilterTypeDef FilterConfig;

			FilterConfig.FilterBank = 0;
			FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
			FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
			FilterConfig.FilterIdHigh = 0x0;
			FilterConfig.FilterIdLow = 0x0;
			FilterConfig.FilterMaskIdHigh = 0x0;
			FilterConfig.FilterMaskIdLow = 0x0;
			FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
			FilterConfig.FilterActivation = ENABLE;
			FilterConfig.SlaveStartFilterBank = 14;


			if (HAL_CAN_ConfigFilter(CANmodule->CANptr, &FilterConfig)
					!= HAL_OK) {
				return CO_ERROR_ILLEGAL_ARGUMENT;
			} else {
				; //do nothing
			}
		}
	} else {
		ret = CO_ERROR_ILLEGAL_ARGUMENT;
	}

	return ret;
}

/******************************************************************************/
CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index,
		uint16_t ident, bool_t rtr, uint8_t noOfBytes, bool_t syncFlag) {
	CO_CANtx_t *buffer = NULL;

	if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
		/* get specific buffer */
		buffer = &CANmodule->txArray[index];

		/* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
		 * Microcontroller specific. */
		buffer->ident &= 0x7FF;
		buffer->ident = ident << 2;
		if (rtr) buffer->ident |= 0x02;

		buffer->DLC = noOfBytes;
		buffer->bufferFull = false;
		buffer->syncFlag = syncFlag;
	}

	return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer) {
	CO_ReturnError_t err = CO_ERROR_NO;

	/* Verify overflow */
	if (buffer->bufferFull) {
		if (!CANmodule->firstCANtxMessage) {
			/* don't set error, if bootup message is still on buffers */
			CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
		}
		err = CO_ERROR_TX_OVERFLOW;
	}

	uint32_t TxMailboxNum;
	CO_LOCK_CAN_SEND(CANmodule);
	prepareTxHeader(&TxHeader, buffer);
	/* if CAN TX buffer is free, copy message to it */
	if ((1 && CANmodule->CANtxCount == 0)
			&& (HAL_CAN_GetTxMailboxesFreeLevel(CANmodule->CANptr) > 0)) {
		CANmodule->bufferInhibitFlag = buffer->syncFlag;
		/* copy message and txRequest */
		if (HAL_CAN_AddTxMessage(CANmodule->CANptr, &TxHeader,
				&buffer->data[0], &TxMailboxNum) != HAL_OK) {
			err = CO_ERROR_ILLEGAL_ARGUMENT;
			printf("[CAN_Driver] Message transmit error !\n");
		} else {
			;/*do nothing*/
		}
	}
	/* if no buffer is free, message will be sent by interrupt */
	else {
		buffer->bufferFull = true;
		CANmodule->CANtxCount++;
	} CO_UNLOCK_CAN_SEND(CANmodule);

	return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule) {
	uint32_t tpdoDeleted = 0U;

	CO_LOCK_CAN_SEND(CANmodule);
	/* Abort message from CAN module, if there is synchronous TPDO.
	 * Take special care with this functionality. */
	// TODO [HJ] : Read the documentation and implement if necessary
	if (/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag) {
		/* clear TXREQ */
		CANmodule->bufferInhibitFlag = false;
		tpdoDeleted = 1U;
	}
	/* delete also pending synchronous TPDOs in TX buffers */
	if (CANmodule->CANtxCount != 0U) {
		uint16_t i;
		CO_CANtx_t *buffer = &CANmodule->txArray[0];
		for (i = CANmodule->txSize; i > 0U; i--) {
			if (buffer->bufferFull) {
				if (buffer->syncFlag) {
					buffer->bufferFull = false;
					CANmodule->CANtxCount--;
					tpdoDeleted = 2U;
				}
			}
			buffer++;
		}
	} CO_UNLOCK_CAN_SEND(CANmodule);

	if (tpdoDeleted != 0U) {
		CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
	}
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
	// TODO : [HJ] Check this function

	uint32_t HALCANErrorCode = CANmodule->CANptr->ErrorCode;

	if (CANmodule->errOld != HALCANErrorCode) {
		uint16_t status = CANmodule->CANerrorStatus;
		CANmodule->errOld = HALCANErrorCode;

		if (HALCANErrorCode & HAL_CAN_ERROR_BOF) { // BUS if off
			status |= CO_CAN_ERRTX_BUS_OFF;

		} else { // BUS is on
			status &= 0xFFFF
					^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING
							| CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
							| CO_CAN_ERRTX_PASSIVE);

			if (HALCANErrorCode & HAL_CAN_ERROR_NONE) {
				__asm__ volatile ("BKPT");
				// Why this happened?
			}

			if (HALCANErrorCode & HAL_CAN_ERROR_EWG) { /* bus warning */
				status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRRX_WARNING;
			}

			// [HJ] Might need to check if it is the first message or not (Bootup message)
			if (HALCANErrorCode & HAL_CAN_ERROR_RX_FOV0) {
				status |= CO_CAN_ERRTX_PASSIVE | CO_CAN_ERRRX_PASSIVE;
			}

			if ((HALCANErrorCode & HAL_CAN_ERROR_RX_FOV0)
					|| (HALCANErrorCode & HAL_CAN_ERROR_RX_FOV1)) {
				status |= CO_CAN_ERRRX_OVERFLOW;
			}
		}
		CANmodule->CANerrorStatus = status;
	}
}

/******************************************************************************/

void CO_CANinterrupt_RX(CO_CANmodule_t *CANmodule) {

	/* receive interrupt */
	CO_CANrxMsg_t rcvMsg; /* pointer to received message in CAN module */
	uint16_t index; /* index of received message */
	uint32_t rcvMsgIdent; /* identifier of the received message */
	CO_CANrx_t *buffer = CANmodule->rxArray; /* receive message buffer from CO_CANmodule_t object. */
	bool_t msgMatched = false;

	HAL_CAN_GetRxMessage(CANmodule->CANptr, CAN_RX_FIFO0,
			&rcvMsg.RxHeader, &rcvMsg.data[0]);

	rcvMsg.ident = rcvMsg.RxHeader.StdId;
	rcvMsg.DLC = rcvMsg.RxHeader.DLC;

	if (CANmodule->useCANrxFilters) {
		__asm__ volatile ("BKPT");
		// Why this happened? For now useRXFilters shouldn't be activated
		/* CAN module filters are used. Message with known 11-bit identifier has */
		/* been received */
		index = 0; /* get index of the received message here. Or something similar */
		if (index < CANmodule->rxSize) {
			buffer = &CANmodule->rxArray[index];
			/* verify also RTR */
			if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
				msgMatched = true;
			}
		}
	} else {
		// [HJ] Probably there would be an error here
		/* CAN module filters are not used, message with any standard 11-bit identifier */
		/* has been received. Search rxArray form CANmodule for the same CAN-ID. */
		buffer = &CANmodule->rxArray[0];
		for (index = CANmodule->rxSize; index > 0U; index--) {
			uint16_t msgIdent = (rcvMsg.RxHeader.StdId << 2) | (rcvMsg.RxHeader.RTR ? 2 : 0);
			if (((msgIdent  ^ buffer->ident) & buffer->mask) == 0U) {
				msgMatched = true;
				break;
			}
			buffer++;
		}
	}

	/* Call specific function, which will process the message */
	if (msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)) {
		buffer->CANrx_callback(buffer->object, (void*) &rcvMsg);
	}

	/* Clear interrupt flag */
	// CubeMX will do it before calling this function
}

void CO_CANinterrupt_TX(CO_CANmodule_t *CANmodule) {
	/* transmit interrupt */
	/* Clear interrupt flag */
	// CubeMX will do it before calling this function
	if (HAL_CAN_GetTxMailboxesFreeLevel(
			(CAN_HandleTypeDef*) CANmodule->CANptr) > 0) {
		/* First CAN message (bootup) was sent successfully */
		CANmodule->firstCANtxMessage = false;
		/* clear flag from previous message */
		CANmodule->bufferInhibitFlag = false;
		/* Are there any new messages waiting to be send */
		if (CANmodule->CANtxCount > 0U) {
			uint16_t i; /* index of transmitting message */

			/* first buffer */
			CO_CANtx_t *buffer = &CANmodule->txArray[0];
			/* search through whole array of pointers to transmit message buffers. */
			for (i = CANmodule->txSize; i > 0U; i--) {
				/* if message buffer is full, send it. */
				if (buffer->bufferFull) {
					buffer->bufferFull = false;
					CANmodule->CANtxCount--;

					/* Copy message to CAN buffer */
					CANmodule->bufferInhibitFlag = buffer->syncFlag;
					uint32_t TxMailboxNum;
					prepareTxHeader(&TxHeader, buffer);
					if( HAL_CAN_AddTxMessage(CANmodule->CANptr,
							&TxHeader,
							&buffer->data[0],
							&TxMailboxNum) != HAL_OK)
					{
						;//do nothing
						__asm__ volatile ("BKPT");
						// Why this happened?
					}
					else
					{
						buffer->bufferFull = false;
						CANmodule->CANtxCount--;
					}
					break; /* exit for loop */
				}
				buffer++;
			}/* end of for loop */

			/* Clear counter if no more messages */
			if (i == 0U) {
				CANmodule->CANtxCount = 0U;
			}
		}
	}
}

