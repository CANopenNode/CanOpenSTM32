/*
 * eeprom.h
 *
 *  Created on: 11 jun 2026
 *      Author: Marc Vandenhende
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#ifdef __cplusplus
extern "C"
{


#endif

/* USER CODE BEGIN Includes */
#include "main.h"

bool Eeprom_Init();
bool Eeprom_Init_CO();

extern uint16_t device_size;
extern size_t   page_size;
extern uint16_t device_i2c_address;
extern uint32_t device_serial_number;

extern uint16_t data_storage_start;
extern uint16_t data_storage_size;

extern size_t device_start_co_auto;
extern size_t device_start_co_prot;
extern size_t device_size_co;

#ifdef __cplusplus
}
#endif

#endif /* INC_EEPROM_H_ */
