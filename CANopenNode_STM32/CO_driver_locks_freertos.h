/*
 * FreeRTOS critical sections for the CANopenNode CO_LOCK_* macros.
 *
 * Include from CO_driver_custom.h 
 * The defaults in CO_driver_target.h are #ifndef-guarded, so no #undef is needed.
 *
 * @file        CO_driver_locks_freertos.h
 * @author      Danial Keshavarzi    2026
 * @copyright   2026 Danial Keshavarzi
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
#ifndef CO_DRIVER_LOCKS_FREERTOS_H
#define CO_DRIVER_LOCKS_FREERTOS_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline UBaseType_t
CO_STM32_enterCritical(void) {
    if (__get_IPSR() == 0U) {
        taskENTER_CRITICAL();
        return 0U;
    }
    return taskENTER_CRITICAL_FROM_ISR();
}

static inline void
CO_STM32_exitCritical(UBaseType_t saved) {
    if (__get_IPSR() == 0U) {
        taskEXIT_CRITICAL();
    } else {
        taskEXIT_CRITICAL_FROM_ISR(saved);
    }
}

/* LOCK declares the state that UNLOCK consumes; both must be in the same scope. */
#define CO_LOCK_CAN_SEND(CAN_MODULE)   UBaseType_t coLockStateSend = CO_STM32_enterCritical()
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) CO_STM32_exitCritical(coLockStateSend)

#define CO_LOCK_EMCY(CAN_MODULE)       UBaseType_t coLockStateEmcy = CO_STM32_enterCritical()
#define CO_UNLOCK_EMCY(CAN_MODULE)     CO_STM32_exitCritical(coLockStateEmcy)

#define CO_LOCK_OD(CAN_MODULE)         UBaseType_t coLockStateOd = CO_STM32_enterCritical()
#define CO_UNLOCK_OD(CAN_MODULE)       CO_STM32_exitCritical(coLockStateOd)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_LOCKS_FREERTOS_H */
