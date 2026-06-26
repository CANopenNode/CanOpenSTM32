/*
 * CANopen main program file.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        main_generic.c
 * @author      Hamed Jafarzadeh 	2022
 * 				Janez Paternoster	2021
 * @copyright   2021 Janez Paternoster
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
 */
#include "CO_app_STM32.h"
#include "CANopen.h"
#include "main.h"
#include <stdio.h>

#include "CO_storageBlank.h"
#include "OD.h"

CANopenNodeSTM32*
    canopenNodeSTM32; // It will be set by canopen_app_init and will be used across app to get access to CANOpen objects

/* Printf function of CanOpen app */
#define log_printf(macropar_message, ...) printf(macropar_message, ##__VA_ARGS__)

/* default values for CO_CANopenInit() */
#ifndef NMT_CONTROL
#define NMT_CONTROL                                                                                                    \
    CO_NMT_STARTUP_TO_OPERATIONAL                                                                                      \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#endif
#ifndef FIRST_HB_TIME
#define FIRST_HB_TIME        500
#endif
#ifndef SDO_SRV_TIMEOUT_TIME
#define SDO_SRV_TIMEOUT_TIME 1000
#endif
#ifndef SDO_CLI_TIMEOUT_TIME
#define SDO_CLI_TIMEOUT_TIME 500
#endif
#ifndef SDO_CLI_BLOCK
#define SDO_CLI_BLOCK        false
#endif
#ifndef OD_STATUS_BITS
#define OD_STATUS_BITS       NULL
#endif

/* Global variables and objects */
CO_t* CO = NULL; /* CANopen object */

// Global variables
uint32_t time_old, time_current;
CO_ReturnError_t err;

/* This function will basically setup the CANopen node */
int
canopen_app_init(CANopenNodeSTM32* _canopenNodeSTM32) {

    // Keep a copy global reference of canOpenSTM32 Object
    canopenNodeSTM32 = _canopenNodeSTM32;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    static CO_storage_t storage;
    static CO_storage_entry_t storageEntries[] = {{.addr = &OD_PERSIST_COMM,
                                                   .len = sizeof(OD_PERSIST_COMM),
                                                   .subIndexOD = 2,
                                                   .attr = CO_storage_cmd | CO_storage_restore,
                                                   .addrNV = NULL}};
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0;
#endif

    /* Allocate memory */
    CO_config_t* config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    /* example usage of CO_MULTIPLE_OD (but still single OD here) */
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
    co_config.CNT_LEDS = 1;
    co_config.CNT_LSS_SLV = 1;
    config_ptr = &co_config;
#endif /* CO_MULTIPLE_OD */

    uint32_t heapMemoryUsed;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return 1;
    } else {
        log_printf("Allocated %lu bytes for CANopen objects\n", heapMemoryUsed);
    }

    canopenNodeSTM32->canOpenStack = CO;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", storageInitError);
        return 2;
    }
#endif

    canopen_app_resetCommunication();
    return 0;
}

int
canopen_app_resetCommunication() {
    /* CANopen communication reset - initialize CANopen objects *******************/
    log_printf("CANopenNode - Reset communication...\n");

    /* Wait rt_thread. */
    CO->CANmodule->CANnormal = false;

    /* Enter CAN configuration. */
    CO_CANsetConfigurationMode((void*)canopenNodeSTM32);
    CO_CANmodule_disable(CO->CANmodule);

    /* initialize CANopen */
    err = CO_CANinit(CO, canopenNodeSTM32, 0); // Bitrate for STM32 microcontroller is being set in MXCube Settings
    if (err != CO_ERROR_NO) {
        log_printf("Error: CAN initialization failed: %d\n", err);
        return 1;
    }

    CO_LSS_address_t lssAddress = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};
    err = CO_LSSinit(CO, &lssAddress, &canopenNodeSTM32->desiredNodeID, &canopenNodeSTM32->baudrate);
    if (err != CO_ERROR_NO) {
        log_printf("Error: LSS slave initialization failed: %d\n", err);
        return 2;
    }

    canopenNodeSTM32->activeNodeID = canopenNodeSTM32->desiredNodeID;
    uint32_t errInfo = 0;

    err = CO_CANopenInit(CO,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         canopenNodeSTM32->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf("Error: Object Dictionary entry 0x%lX\n", errInfo);
        } else {
            log_printf("Error: CANopen initialization failed: %d\n", err);
        }
        return 3;
    }

    err = CO_CANopenInitPDO(CO, CO->em, OD, canopenNodeSTM32->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf("Error: Object Dictionary entry 0x%lX\n", errInfo);
        } else {
            log_printf("Error: PDO initialization failed: %d\n", err);
        }
        return 4;
    }

    /* Configure Timer interrupt function for execution every 1 millisecond */
    HAL_TIM_Base_Start_IT(canopenNodeSTM32->timerHandle); //1ms interrupt

    /* Configure CAN transmit and receive interrupt */

    /* Configure CANopen callbacks, etc */
    if (!CO->nodeIdUnconfigured) {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        if (storageInitError != 0) {
            CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
        }
#endif
    } else {
        log_printf("CANopenNode - Node-id not initialized\n");
    }

#if (CO_CONFIG_SDO_SRV) & CO_CONFIG_FLAG_CALLBACK_PRE
	CO_InitCallbacks();
#endif

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule);

    log_printf("CANopenNode - Running...\n");
    fflush(stdout);
    time_old = time_current = HAL_GetTick();
    return 0;
}

void debug_irq_state(void)
{
    uint32_t primask   = __get_PRIMASK();
    uint32_t basepri   = __get_BASEPRI();
    uint32_t faultmask = __get_FAULTMASK();
    uint32_t ipsr      = __get_IPSR();
    uint32_t icsr      = SCB->ICSR;

    if (primask || basepri || faultmask) {

        log_printf("IRQ BLOCKED!\r\n");
        log_printf("PRIMASK   = %lu\r\n", primask);
        log_printf("BASEPRI   = %lu\r\n", basepri);
        log_printf("FAULTMASK = %lu\r\n", faultmask);
        log_printf("IPSR      = %lu\r\n", ipsr);
        log_printf("ACTIVE IRQ= %lu\r\n", icsr & 0x1FF);

    }
}


void FDCAN_checkErrors(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t ir  = hfdcan->Instance->IR;
    uint32_t psr = hfdcan->Instance->PSR;
    uint32_t ecr = hfdcan->Instance->ECR;
    uint32_t err = ir & FDCAN_IR_ALL_ERROR_MASK;

    if (err == 0) return;

    log_printf("\r\n--- FDCAN ERROR ---\r\n");

    /* --- Bus State --- */
    if (psr & FDCAN_PSR_BO)
        log_printf("BUS-OFF: Controller ist vom Bus getrennt (TX deaktiviert!)\r\n");

    if (psr & FDCAN_PSR_EP)
        log_printf("ERROR PASSIVE: Controller reduziert Aktivität (verzoegertes TX)\r\n");

    if (psr & FDCAN_PSR_EW)
        log_printf("ERROR WARNING: Fehlerzaehler ueber Schwellwert\r\n");

    /* --- Protocol Errors --- */
    if (err & FDCAN_IR_PED)
        log_printf("PROTOCOL DATA ERROR: Bitfehler in Datenphase (Signalstoerung / Timingproblem)\r\n");

    if (err & FDCAN_IR_PEA)
        log_printf("PROTOCOL ARBITRATION ERROR: Arbitration verloren oder Bitfehler im Identifier\r\n");

    /* --- FIFO / RX Probleme --- */
    if (err & FDCAN_IR_RF0L)
        log_printf("RX FIFO0 OVERFLOW: Nachricht verloren (ISR oder Main zu langsam!)\r\n");

    if (err & FDCAN_IR_RF1L)
        log_printf("RX FIFO1 OVERFLOW: Nachricht verloren (hohe Buslast / schlechtes Timing)\r\n");

    /* --- RAM / Zugriff --- */
    if (err & FDCAN_IR_MRAF)
        log_printf("MESSAGE RAM ACCESS FAILURE: Zugriff zu langsam oder falsche RAM-Konfiguration\r\n");

    if (err & FDCAN_IR_ARA)
        log_printf("ACCESS TO RESERVED ADDRESS: Ungueltiger RAM-Zugriff (Konfigurations- oder Timingfehler!)\r\n");

    /* --- Erweiterte / seltene Fehler --- */
    if (err & FDCAN_IR_ELO)
        log_printf("ERROR LOG OVERFLOW: Zu viele Fehler in kurzer Zeit → Details teilweise verloren\r\n");

    if (err & FDCAN_IR_WDI)
        log_printf("WATCHDOG INTERRUPT: Message RAM wurde nicht rechtzeitig bedient → evtl. CPU/IRQ blockiert\r\n");

    if (err & FDCAN_IR_TOO)
        log_printf("TIMEOUT OCCURRED: Timeout bei TX/RX → Bus blockiert oder Gegenstelle reagiert nicht\r\n");

    /* --- Error Counter --- */
    uint8_t tx_err = (ecr >> 8) & 0xFF;
    uint8_t rx_err = (ecr >> 0) & 0xFF;

    log_printf("Error Counter: TX=%u RX=%u\r\n", tx_err, rx_err);

    /* --- Rohwerte (sehr hilfreich!) --- */
    log_printf("IR=0x%08lX PSR=0x%08lX ECR=0x%08lX\r\n", ir, psr, ecr);

    /* ✅ Fehlerflags löschen */
    hfdcan->Instance->IR = err;
}

typedef struct
{
    /* CANopen */
    uint8_t  nmt_state;
    uint8_t  can_normal;
    uint16_t can_error;

    /* RX FIFO Status */
    uint8_t  f0_fl;
    uint8_t  f0_full;
    uint8_t  f0_fgi;
    uint8_t  f0_fpi;

    /* FDCAN Register */
    uint32_t IE;
    uint32_t ILE;
    uint32_t IR;
    uint32_t PSR;
    uint32_t ECR;

    /* Software */
    int32_t  fifo_cnt;

} fdcan_snapshot_t;


#define SNAPSHOT_SIZE 255
static uint16_t fdcan_count = 0;
static fdcan_snapshot_t fdcan_buffer[SNAPSHOT_SIZE];
static uint16_t fdcan_index = 0;

static inline void fdcan_log_snapshot(int fifo_cnt)
{
    uint32_t rxf0s = FDCAN1->RXF0S;

    fdcan_buffer[fdcan_index].nmt_state  = CO->NMT->operatingState;
    fdcan_buffer[fdcan_index].can_normal = CO->CANmodule->CANnormal;
    fdcan_buffer[fdcan_index].can_error  = CO->CANmodule->CANerrorStatus;

    fdcan_buffer[fdcan_index].f0_fl   =  rxf0s & 0x7F;
    fdcan_buffer[fdcan_index].f0_full = (rxf0s >> 25) & 0x1;
    fdcan_buffer[fdcan_index].f0_fgi  = (rxf0s >> 8)  & 0x3F;
    fdcan_buffer[fdcan_index].f0_fpi  = (rxf0s >> 16) & 0x3F;

    fdcan_buffer[fdcan_index].IE  = FDCAN1->IE;
    fdcan_buffer[fdcan_index].ILE = FDCAN1->ILE;
    fdcan_buffer[fdcan_index].IR  = FDCAN1->IR;
    fdcan_buffer[fdcan_index].PSR = FDCAN1->PSR;
    fdcan_buffer[fdcan_index].ECR = FDCAN1->ECR;

    fdcan_buffer[fdcan_index].fifo_cnt = fifo_cnt;

    if (fdcan_count < SNAPSHOT_SIZE)
        fdcan_count++;

    fdcan_index = (fdcan_index + 1) % SNAPSHOT_SIZE;
}
void fdcan_print_log(void)
{
    printf("\r\n--- FDCAN LOG (%u) ---\r\n", fdcan_count);

    uint16_t idx = fdcan_index;

    for (uint16_t i = 0; i < fdcan_count; i++)
    {
        idx = (idx == 0) ? (SNAPSHOT_SIZE - 1) : (idx - 1);

        printf("%d; %d; 0x%04X; %u; %u; %u; %u; 0x%08lX; 0x%08lX; 0x%08lX; 0x%08lX; 0x%08lX; %ld\r\n",
            fdcan_buffer[idx].nmt_state,
            fdcan_buffer[idx].can_normal,
            fdcan_buffer[idx].can_error,
            fdcan_buffer[idx].f0_fl,
            fdcan_buffer[idx].f0_full,
            fdcan_buffer[idx].f0_fgi,
            fdcan_buffer[idx].f0_fpi,
            fdcan_buffer[idx].IE,
            fdcan_buffer[idx].ILE,
            fdcan_buffer[idx].IR,
            fdcan_buffer[idx].PSR,
            fdcan_buffer[idx].ECR,
            fdcan_buffer[idx].fifo_cnt);
    }
}


void fdcan_print_once(void)
{
    static uint8_t already_printed = 0;

    if (!already_printed)
    {
        fdcan_print_log();
        already_printed = 1;
    }
}

void
canopen_app_process() {




    /* loop for normal program execution ******************************************/
    /* get time difference since last function call */
    time_current = HAL_GetTick();
    static int cnt = 0 ;
    static int fifo_cnt=0;
    if ((time_current - time_old) > 0) { // Make sure more than 1ms elapsed
        /* CANopen process */
        CO_NMT_reset_cmd_t reset_status;
        uint32_t timeDifference_us = (time_current - time_old) * 1000;
        time_old = time_current;

        FDCAN_checkErrors(canopenNodeSTM32->CANHandle);
        debug_irq_state();

        if((CO_nodeGuardingSlave_TimeLeft(CO->NGslave) <100000) ){	// 100 ms vor Ablauf des Guardings, starten wir mal neu
			if( CO_nodeGuardingSlave_TimeLeft(CO->NGslave) >0){
				fdcan_log_snapshot( fifo_cnt);
        	if (cnt<=15){
				if (cnt==0)
						log_printf("NMT state; CANnormal; CANerrorStatus; RXF0 fill; RXF0 lost; RXF0 get_idx; RXF0 put_idx; FDCAN IE; FDCAN ILE ;FDCAN IR; FDCAN PSR; FDCAN ECR\r\n");

				log_printf("%d; %d; 0x%04X; %lu; %lu; %lu; %lu; 0x%08lX; 0x%08lX; 0x%08lX; 0x%08lX; 0x%08lX; %d\r\n"	, CO->NMT->operatingState, CO->CANmodule->CANnormal, CO->CANmodule->CANerrorStatus, \
							FDCAN1->RXF0S & 0x7F, (FDCAN1->RXF0S >> 25) & 1, (FDCAN1->RXF0S >> 8)  & 0x3F, (FDCAN1->RXF0S >> 16)  & 0x3F, FDCAN1->IE, FDCAN1->ILE, FDCAN1->IR, FDCAN1->PSR, FDCAN1->ECR, fifo_cnt );


				}
				cnt++;

				if  (cnt>=10){
//					 canopen_app_resetCommunication();
					clear_all_can_errorFlags(canopenNodeSTM32->CANHandle);
					log_printf("ERRORS CLEARD! %d; %d; 0x%04X; %lu; %lu; %lu; %lu; 0x%08lX; 0x%08lX; 0x%08lX; 0x%08lX; 0x%08lX; %d\r\n"	, CO->NMT->operatingState, CO->CANmodule->CANnormal, CO->CANmodule->CANerrorStatus, \
								FDCAN1->RXF0S & 0x7F, (FDCAN1->RXF0S >> 25) & 1, (FDCAN1->RXF0S >> 8)  & 0x3F, (FDCAN1->RXF0S >> 16)  & 0x3F, FDCAN1->IE, FDCAN1->ILE, FDCAN1->IR, FDCAN1->PSR, FDCAN1->ECR, fifo_cnt);
					__NOP();
					if (cnt<15)
						cnt++;
				}
        	}
        } else
        	cnt=0;

#ifdef CANFIFO
        /* ? 1. Alle RX Messages aus Ringbuffer verarbeiten */
    	CO_CANrxMsg_t msg;
    	while (rb_pop(&msg, &fifo_cnt)) {
    		handle_can_received_msg(&msg);
    	}

//        /* ? RX Overflow zur�cksetzen, wenn stabil */
//        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0 &&
//            HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) == 0) {
//
//            CANModule_local->CANerrorStatus &= ~CO_CAN_ERRRX_OVERFLOW;
//        }
//
#endif

        reset_status = CO_process(CO, false, timeDifference_us, NULL);

        canopenNodeSTM32->outStatusLEDRed = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
        canopenNodeSTM32->outStatusLEDGreen = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

        if (reset_status == CO_RESET_COMM) {
            /* delete objects from memory */
        	HAL_TIM_Base_Stop_IT(canopenNodeSTM32->timerHandle);
            CO_CANsetConfigurationMode((void*)canopenNodeSTM32);
            CO_delete(CO);
            log_printf("CANopenNode Reset Communication request\n");
            canopen_app_init(canopenNodeSTM32); // Reset Communication routine
        } else if (reset_status == CO_RESET_APP) {
            log_printf("CANopenNode Device Reset\n");
            HAL_NVIC_SystemReset(); // Reset the STM32 Microcontroller
        }
    }
}

/* Thread function executes in constant intervals, this function can be called from FreeRTOS tasks or Timers ********/
void
canopen_app_interrupt(void) {
    CO_LOCK_OD(CO->CANmodule);
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
        bool_t syncWas = false;
        /* get time difference since last function call */
        uint32_t timeDifference_us = 1000; // 1ms second

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

        /* Further I/O or nonblocking application code may go here. */
    }
    CO_UNLOCK_OD(CO->CANmodule);
}
