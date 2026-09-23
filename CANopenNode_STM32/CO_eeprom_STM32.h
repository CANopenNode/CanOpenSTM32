/*
 * CO_eeprom_STM32.h
 *
 *  Created on: 23 Nov 2025
 *      Author: Marc Vandenhende
 */

#ifndef CO_EEPROM_STM32_H_
#define CO_EEPROM_STM32_H_

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE) != 0

bool_t primeEeprom();

#endif

#endif /* CO_EEPROM_STM32_H_ */
