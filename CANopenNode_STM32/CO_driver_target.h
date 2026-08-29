/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
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
 */
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include "main.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Determining the CANopen Driver

#if defined(FDCAN) || defined(FDCAN1) || defined(FDCAN2) || defined(FDCAN3)
#define CO_STM32_FDCAN_Driver 1
#elif defined(CAN) || defined(CAN1) || defined(CAN2) || defined(CAN3)
#define CO_STM32_CAN_Driver 1
#else
#error This STM32 does not support CAN or FDCAN
#endif

#undef CO_CONFIG_STORAGE_ENABLE // We don't need the Storage option; implement it based on your use case and remove this line

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/**
 * \brief           CAN RX message for platform
 *
 * This is platform specific
 */
typedef struct {
    uint32_t ident;  /*!< Standard identifier */
    uint8_t dlc;     /*!< Data length */
    uint8_t data[8]; /*!< Received data */
} CO_CANrxMsg_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t*)(msg)))->ident)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t*)(msg)))->dlc)
#define CO_CANrxMsg_readData(msg)  ((uint8_t*)(((CO_CANrxMsg_t*)(msg)))->data)

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void* object;
    void (*CANrx_callback)(void* object, void* message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void* CANptr;
    CO_CANrx_t* rxArray;
    uint16_t rxSize;
    CO_CANtx_t* txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;

} CO_CANmodule_t;

/* Data storage object for one entry */
typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    /* Additional variables (target specific) */
    void* addrNV;
} CO_storage_entry_t;

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                                             \
    do {                                                                                                               \
        CO_MemoryBarrier();                                                                                            \
        rxNew = (void*)1L;                                                                                             \
    } while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                                           \
    do {                                                                                                               \
        CO_MemoryBarrier();                                                                                            \
        rxNew = NULL;                                                                                                  \
    } while (0)

/*
 * With the STM32 port, we can lock interrupts in at least 2 ways:
 *
 *  - Global interrupt disable with the PRIMASK register
 *  - Selective interrupt disable with the BASEPRI register (when available in the core),
 *          where all interrupts with a numerically greater priority number than the max priority level become disabled.
 *
 * Imagine the following interrupt priority setup. Higher on the list means higher priority (logical priority number).
 *
 * | prio   | IRQ name  |
 * | 0      | TIM1      |
 * | ..     | ..        |
 * | 4      | FDCAN     |
 * | 5      | ...       |
 * ...
 * | 15     |           |
 *
 * Setting \ref CO_LOCK_BASEPRI_PRIO_LEVEL to 4 will disable all interrupts that
 * have an interrupt priority number of 4 or greater (all lower priority).
 *
 * When the CANopen application needs to block its own CO-dependent interrupts,
 * we may still want to keep, for example, a timer interrupt running (for external signal processing)
 * since it is irrelevant to the CO application.
 *
 * Use the selective BASEPRI interrupt system at your own responsibility.
 * For simplicity, disable BASEPRI and use the global interrupt control with the PRIMASK register instead.
 */

/**
 * \brief           Enable or disable selective interrupt disable with the BASEPRI register
 * \note            This only works for CPUs with BASEPRI enabled, and will throw a compilation error otherwise
 */
#ifndef CO_LOCK_BASEPRI_ENABLE
#define CO_LOCK_BASEPRI_ENABLE 0
#endif

/**
 * \brief           Defines the priority level at which interrupts are disabled with the BASEPRI register.
 *                  Any interrupt whose priority number is equal to or greater than this value will be disabled.
 *                  (Remember, a lower number means a logically higher interrupt priority.)
 *
 * \note            To make sure this operation works properly, the system shall configure the preemptive NVIC config
 *                  correctly at the application level. CO locking is not designed to take this into account.
 *
 *                  Responsibility lies entirely with the user.
 */
#ifndef CO_LOCK_BASEPRI_PRIO_LEVEL
#define CO_LOCK_BASEPRI_PRIO_LEVEL 0
#endif

/**
 * \brief           Number of priority bits the CPU implements, used for correct value alignment.
 *                  If not provided, we try to use the value defined in the CPU header with the ARM Cortex-M defined macro.
 */
#ifndef CO_LOCK_BASEPRI_NVIC_PRIO_BITS
#define CO_LOCK_BASEPRI_NVIC_PRIO_BITS __NVIC_PRIO_BITS
#endif

#if CO_LOCK_BASEPRI_ENABLE

/* Setup the generic interrupt management system */
#define CO_LOCK_GENERIC(localvarname)                                                                                  \
    do {                                                                                                               \
        uint32_t localvarname = __get_BASEPRI();                                                                       \
        __set_BASEPRI_MAX(CO_LOCK_BASEPRI_PRIO_LEVEL << (8UL - CO_LOCK_BASEPRI_NVIC_PRIO_BITS));                       \
        __DSB();                                                                                                       \
        __ISB();
#define CO_UNLOCK_GENERIC(localvarname)                                                                                \
    __set_BASEPRI(localvarname);                                                                                       \
    __DSB();                                                                                                           \
    __ISB();                                                                                                           \
    }                                                                                                                  \
    while (0)
#else
/* Setup the generic interrupt management system */
#define CO_LOCK_GENERIC(localvarname)                                                                                  \
    do {                                                                                                               \
        uint32_t localvarname = __get_PRIMASK();                                                                       \
        __disable_irq();
#define CO_UNLOCK_GENERIC(localvarname)                                                                                \
    __set_PRIMASK(localvarname);                                                                                       \
    }                                                                                                                  \
    while (0)
#endif /* */

/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)   CO_LOCK_GENERIC(primask_send)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) CO_UNLOCK_GENERIC(primask_send)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)       CO_LOCK_GENERIC(primask_emcy)
#define CO_UNLOCK_EMCY(CAN_MODULE)     CO_UNLOCK_GENERIC(primask_emcy)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)         CO_LOCK_GENERIC(primask_od)
#define CO_UNLOCK_OD(CAN_MODULE)       CO_UNLOCK_GENERIC(primask_od)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
