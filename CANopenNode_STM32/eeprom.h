/*
 * eeprom.h
 *
 *  Created on: 11 jun 2026
 *      Author: Marc Vandenhende
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "CO_app_STM32.h"
#include "main.h"
#include <stdbool.h>

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE) != 0

/*
 * MICROCHIP 24AA256UID EEPROM STORAGE DEFINITIONS
 */

#define EEPROM_WRITE_TIME_MS 5 // maximum write cycle time [ms]

#define MC_EEP_I2C_ADDR      0x50U // 24AA256UID I2C address
#define MC_EEP_MFR_ID        0x29U // 24AA256UID device ID : manufacturer code
#define MC_EEP_DEVICE_ID     0x48U // 24AA256UID device ID : device code

#define MC_EEP_DEVICE_SIZE   0x8000U // Actual device size is 0x8000 bytes... but top 0x1000 bytes are unavailable
#define MC_EEP_UNAVAILABLE   0x1000U // ... but top 0x1000 bytes are unavailable
#define MC_EEP_AVAILABLE     (MC_EEP_DEVICE_SIZE - MC_EEP_UNAVAILABLE) // effectively available storage
#define MC_EEP_PAGE_SIZE     64U                                       // 64 byte write page size
#define MC_EEP_EUI_48_ADDR   0x7F7AU                                   // 24AA256UID EUI-48 address
#define MC_EEP_EUI_48_SIZE   6U                                        // 24AA256UID EUI-48 size
#define MC_EEP_EUI_64_ADDR   0x7FB8U                                   // 24AA256UID EUI-64 address
#define MC_EEP_EUI_64_SIZE   8U                                        // 24AA256UID EUI-64 size
#define MC_EEP_SERIAL_ADDR   0x7FFAU                                   // 24AA256UID device ID / serial number address
#define MC_EEP_SERIAL_SIZE   6U                                        // 24AA256UID device ID / serial number size

#define CO_EEP_MAX_STORAGE   0x2000U                  // Max number of bytes reserved for canopennode storage
#define CO_EEP_PAGE_SIZE     (CO_EEP_MAX_STORAGE / 2) // split in half for auto and protected storage
#define CO_EEP_START_AUTO    (MC_EEP_AVAILABLE - CO_EEP_MAX_STORAGE) // start address for canopennode auto storage
#define CO_EEP_START_PROT    (CO_EEP_START_AUTO + CO_EEP_PAGE_SIZE)  // start address for canopennode protected storage

#define APP_EEP_START        0U                                      // EEPROM start address for application usage
#define APP_EEP_SIZE         (MC_EEP_AVAILABLE - CO_EEP_MAX_STORAGE) // EEPROM size for application usage

extern uint32_t device_serial_number;

bool Eeprom_Init();

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */

#endif /* INC_EEPROM_H_ */
