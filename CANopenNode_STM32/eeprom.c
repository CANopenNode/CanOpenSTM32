/*
 * eeprom.c
 *
 *  Created on: 11 jun 2026
 *      Author: Marc Vandenhende
 */

#include "eeprom.h"

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE) != 0

uint32_t device_serial_number;

static bool_t eeprom_init_mc();

/*
 * Main EEPROM initialization routine
 * returns "true" if initialization is success
 * returns "false" if initialization fails or no eeprom present
 */
bool
Eeprom_Init() {
    if (!eeprom_init_mc()) {
        // no EEPROM found
        return false;
    }

    /* If eeprom chip is OK, this will return "true" */
    return HAL_I2C_IsDeviceReady(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) == HAL_OK;
}

/*
 * Try to initialize eeprom
 * Check if device is Microchip 24AA256UID
 * read out device serial number for later usage
 * returns "true" if initialization is success
 * returns "false" if no 24AA256UID eeprom found
 */
static bool_t
eeprom_init_mc() {
    uint8_t serial[MC_EEP_SERIAL_SIZE];

    /* If eeprom chip is OK, this will pass, otherwise timeout */
    if (HAL_I2C_IsDeviceReady(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) != HAL_OK) {
        return false; // return "false" if device not ready
    }

    if (HAL_I2C_Mem_Read(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, MC_EEP_SERIAL_ADDR, 2, serial, MC_EEP_SERIAL_SIZE,
                         I2C_TIMEOUT_MS)
        != HAL_OK) {
        return false; // return "false" if device does not respond
    }

    // Check if Microchip 24AA256UID EEPROM device
    if ((serial[0] == MC_EEP_MFR_ID) && (serial[1] == MC_EEP_DEVICE_ID)) {
        // set serial number
        device_serial_number = (uint32_t)serial[2] << 24 | (uint32_t)serial[3] << 16 | (uint32_t)serial[4] << 8
                               | (uint32_t)serial[5];

        return true;
    }

    return false;
}

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */
