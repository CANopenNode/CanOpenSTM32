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


#include <stdio.h>

#include "CO_storageBlank.h"
#include "OD.h"
#include "CANopen.h"

#include "tim.h"
#include "fdcan.h"


#define log_printf(macropar_message, ...) \
        printf(macropar_message, ##__VA_ARGS__)


/* default values for CO_CANopenInit() */
#define NMT_CONTROL \
            CO_NMT_STARTUP_TO_OPERATIONAL \
          | CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL


/* Global variables and objects */
CO_t *CO = NULL; /* CANopen object */
uint8_t LED_red, LED_green;

/* Function Prototypes */
void CO_delay(uint32_t delay, CO_NMT_reset_cmd_t *reset);
void CO_handle(uint32_t *time_old, CO_NMT_reset_cmd_t *reset);

/* main ***********************************************************************/
int main_canopen (void){
	uint32_t time_old, time_current;
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    void *CANptr = NULL; /* CAN module address */
    CANptr = &hfdcan1;
    uint8_t pendingNodeId = 13; /* read from dip switches or nonvolatile memory, configurable by LSS slave */
    uint8_t activeNodeId = 13; /* Copied from CO_pendingNodeId in the communication reset section */
    uint16_t pendingBitRate = 250;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storage_t storage;
    CO_storage_entry_t storageEntries[] = {
        {
            .addr = &OD_PERSIST_COMM,
            .len = sizeof(OD_PERSIST_COMM),
            .subIndexOD = 2,
            .attr = CO_storage_cmd | CO_storage_restore,
            .addrNV = NULL
        }
    };
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0;
#endif

    /* Configure microcontroller. */


    /* Allocate memory */
    CO_config_t *config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    /* example usage of CO_MULTIPLE_OD (but still single OD here) */
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
    co_config.CNT_LEDS = 1;
    co_config.CNT_LSS_SLV = 1;
    config_ptr = &co_config;
#endif /* CO_MULTIPLE_OD */
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return 0;
    }
    else {
        log_printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
    }


#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage,
                               CO->CANmodule,
                               OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters,
                               storageEntries,
                               storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", storageInitError);
        return 0;
    }
#endif


    while(reset != CO_RESET_APP){
		/* CANopen communication reset - initialize CANopen objects *******************/
        log_printf("CANopenNode - Reset communication...\n");

        /* Wait rt_thread. */
        CO->CANmodule->CANnormal = false;

        /* Enter CAN configuration. */
        CO_CANsetConfigurationMode((void *)&CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        /* initialize CANopen */
        err = CO_CANinit(CO, CANptr, pendingBitRate);
        if (err != CO_ERROR_NO) {
            log_printf("Error: CAN initialization failed: %d\n", err);
            return 0;
        }

        CO_LSS_address_t lssAddress = {.identity = {
            .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
            .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
            .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
            .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
        }};
        err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate);
        if(err != CO_ERROR_NO) {
            log_printf("Error: LSS slave initialization failed: %d\n", err);
            return 0;
        }

        activeNodeId = pendingNodeId;
        uint32_t errInfo = 0;

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
        if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
            }
            else {
                log_printf("Error: CANopen initialization failed: %d\n", err);
            }
            return 0;
        }

        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if(err != CO_ERROR_NO) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
            }
            else {
                log_printf("Error: PDO initialization failed: %d\n", err);
            }
            return 0;
        }


        /* Configure Timer interrupt function for execution every 1 millisecond */
        HAL_TIM_Base_Start_IT(&htim17); //1ms interrupt

        /* Configure CAN transmit and receive interrupt */


        /* Configure CANopen callbacks, etc */
        if(!CO->nodeIdUnconfigured) {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            if(storageInitError != 0) {
                CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY,
                               CO_EMC_HARDWARE, storageInitError);
            }
#endif
        }
        else {
            log_printf("CANopenNode - Node-id not initialized\n");
        }


        /* start CAN */
        CO_CANsetNormalMode(CO->CANmodule);

        reset = CO_RESET_NOT;

        log_printf("CANopenNode - Running...\n");
        fflush(stdout);
        time_old = time_current = HAL_GetTick();
        uint32_t counter = 0;
        while(reset == CO_RESET_NOT){
        	/* loop for normal program execution ******************************************/
            /* get time difference since last function call */
        	time_current = HAL_GetTick();

            if((time_current - time_old) > 0){ // More than 1ms elapsed
                CO_handle(&time_old, &reset);
            }

//        	OD_set_u32(OD_find(OD, 0x6000), 0x00, counter, false);
//        	counter++;
//        	CO_TPDOsendRequest(&CO->TPDO[4]);
//        	HAL_Delay(10);

            /* Nonblocking application code may go here. */

            /* Process automatic storage */

            /* optional sleep for short time */
        }
    }


/* program exit ***************************************************************/
    /* stop threads */


    /* delete objects from memory */
    CO_CANsetConfigurationMode((void *)&CANptr);
    CO_delete(CO);

    log_printf("CANopenNode finished\n");
	HAL_NVIC_SystemReset(); // Reset the microcontroller
    /* reset */
    return 0;
}

/* Handles status LEDs and NMT state in loop ********************************/
// This is the code that must be constantly running when the device is operational
// even during delays
void CO_handle(uint32_t *time_old, CO_NMT_reset_cmd_t *reset) {
    /* CANopen process */
    uint32_t time_current = HAL_GetTick();
    uint32_t timeDifference_us = (time_current - *time_old) * 1000;
    *time_old = time_current;
    *reset = CO_process(CO, false, timeDifference_us, NULL);
    LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
    LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);
    // Turn on/off green pin accordingly
    if (LED_green && !HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)) {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    } else if (!LED_green && HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)) {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    }
    // Turn on/off red pin accordingly (TODO)

}

/* Handles a non-blocking delay, calling CO_handle each ms ********************************/
void CO_delay(uint32_t delay, CO_NMT_reset_cmd_t *reset) {
    uint32_t start_time, curr_time, t_last_hand;
    start_time = curr_time = t_last_hand = HAL_GetTick();

    // Runs until reset or delay has reached
    while(*reset == CO_RESET_NOT && (curr_time - start_time) <= delay) {
        if((curr_time - t_last_hand) > 0){ // More than 1ms elapsed
            CO_handle(&t_last_hand, reset);
        }
        curr_time = HAL_GetTick();

    }
}

/* timer thread executes in constant intervals ********************************/
void tmrTask_thread(void){
        CO_LOCK_OD(CO->CANmodule);
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
            bool_t syncWas = false;
            /* get time difference since last function call */
            uint32_t timeDifference_us = 1000;

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


/* CAN interrupt function executes on received CAN message ********************/
void /* interrupt */ CO_CAN1InterruptHandler(void){

    /* clear interrupt flag */
}

/* Timer interrupt function executes every 1 ms ********************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim17)
		tmrTask_thread();
}

