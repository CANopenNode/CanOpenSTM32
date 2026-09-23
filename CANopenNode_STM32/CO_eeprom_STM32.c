/*
 * Eeprom interface for use with CO_storageEeprom, STM32 specific
 *
 * @file        CO_eeprom_STM32.c
 * @author      Marc Vandenhende
 * @copyright   2025 Marc Vandenhende
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

#include <string.h>

#include "storage/CO_storage.h"
#include "storage/CO_eeprom.h"
#include "301/crc16-ccitt.h"
#include "CO_app_STM32.h"
#include "OD.h"

#include "eeprom.h"
#include "CO_eeprom_STM32.h"

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE) != 0

/*
 * Eeprom is configured for auto storage variables. Second half of
 * memory locations is used for storage on command. Below are two
 * internal variables, used for indicating next free address in eeprom, one for
 * autonomous storage and one for protected storage
 */
static size_t eepromAddrNextAuto = 0;
static size_t eepromAddrNextProt = 0;

/******************************************************************************/
bool_t
CO_eeprom_init(void* storageModule) {
    // initialize canopen eeprom storage
    if (!Eeprom_Init()) {
        return false;
    }

    eepromAddrNextAuto = CO_EEP_START_AUTO;
    eepromAddrNextProt = CO_EEP_START_PROT;
    OD_PERSIST_COMM.x1018_identity.serialNumber = device_serial_number;

    /* If eeprom chip is OK, this will return "true" */
    return (HAL_I2C_IsDeviceReady(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) == HAL_OK);
}

/******************************************************************************/
size_t
CO_eeprom_getAddr(void* storageModule, bool_t isAuto, size_t len, bool_t* overflow) {
    size_t addr;

    if (isAuto) {
        /* auto storage is processed byte by byte, no alignment necessary */
        addr = eepromAddrNextAuto;
        eepromAddrNextAuto += len;
        if (eepromAddrNextAuto >= CO_EEP_START_AUTO + CO_EEP_PAGE_SIZE) {
            *overflow = true;
        }
    } else {
        /* addresses for storage on command must be page aligned */
        addr = eepromAddrNextProt;
        size_t lenAligned = len & (~(MC_EEP_PAGE_SIZE - 1));
        if (lenAligned < len) {
            lenAligned += MC_EEP_PAGE_SIZE;
        }
        eepromAddrNextProt += lenAligned;
        if (eepromAddrNextProt >= CO_EEP_START_PROT + CO_EEP_PAGE_SIZE) {
            *overflow = true;
        }
    }

    return addr;
}

/******************************************************************************/
void
CO_eeprom_readBlock(void* storageModule, uint8_t* data, size_t eepromAddr, size_t len) {
#if defined(CAN_CFG_GPIO_Port) && defined(CAN_CFG_Pin)
    /*
     * Return empty data buffer (all bytes 0xff) when if CFG mode
     * -> Only if CFG input pin defined!
     */
    if (!HAL_GPIO_ReadPin(CAN_CFG_GPIO_Port, CAN_CFG_Pin)) {
        memset(data, 0xff, len);
    } else
#endif
        HAL_I2C_Mem_Read(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, eepromAddr, 2, data, len, I2C_TIMEOUT_MS);
}

/******************************************************************************/
bool_t
CO_eeprom_writeBlock(void* storageModule, uint8_t* data, size_t eepromAddr, size_t len) {
    size_t idx = 0;
    uint32_t eep_write_timer = 0;

    if (HAL_I2C_IsDeviceReady(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) != HAL_OK) {
        // device not ready
        return false;
    }

    while (len > 0) {
        size_t len_x = len;
        if (len_x > MC_EEP_PAGE_SIZE) {
            len_x = MC_EEP_PAGE_SIZE;
        }

        if (HAL_I2C_Mem_Write(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, eepromAddr, 2, data + idx, len_x, I2C_TIMEOUT_MS)
            != HAL_OK) {
            // write command error
            return false;
        }

        eepromAddr += MC_EEP_PAGE_SIZE;
        idx += MC_EEP_PAGE_SIZE;

        if (len > MC_EEP_PAGE_SIZE) {
            len -= MC_EEP_PAGE_SIZE;
        } else {
            len = 0;
        }

        /*  wait for completion of the write operation */
        eep_write_timer = HAL_GetTick();
        while (HAL_I2C_IsDeviceReady(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) != HAL_OK) {
            // device not ready yet
            if (HAL_GetTick() - eep_write_timer >= EEPROM_WRITE_TIME_MS + 1)
            // + 1 to compensate for HAL_GetTick() millisecond jitter
            {
                // time out
                return false;
            }
        }
    }
    return true;
}

/******************************************************************************/
uint16_t
CO_eeprom_getCrcBlock(void* storageModule, size_t eepromAddr, size_t len) {
#define BUF_SIZE 250

    uint16_t crc = 0;
    uint8_t buf[BUF_SIZE];
    uint8_t subLen;

    while (len > 0) {
        if (len <= BUF_SIZE) {
            subLen = len;
        } else {
            subLen = BUF_SIZE;
        }

        /* update crc from data part */
        HAL_I2C_Mem_Read(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, eepromAddr, 2, buf, subLen, I2C_TIMEOUT_MS);
        crc = crc16_ccitt(buf, subLen, crc);
        eepromAddr += subLen;
        len -= subLen;
    }

    return crc;
}

/******************************************************************************/
bool_t
CO_eeprom_updateByte(void* storageModule, uint8_t data, size_t eepromAddr) {
    uint8_t buf;

    if (HAL_I2C_IsDeviceReady(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    /* read data byte from eeprom */
    if (HAL_I2C_Mem_Read(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, eepromAddr, 2, &buf, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    /* If data in EEPROM differs, then write it to EEPROM.
     * Don't wait for write to complete */
    if (buf != data) {
        if (HAL_I2C_Mem_Write(HI2C_EEPROM, MC_EEP_I2C_ADDR << 1, eepromAddr, 2, &data, 1, I2C_TIMEOUT_MS) != HAL_OK) {
            return false;
        }
    }

    return true;
}

bool_t
primeEeprom() {
    /* save default PERSIST_COMM data to the eeprom */
    bool_t writeOk = CO_eeprom_writeBlock(NULL, (uint8_t*)&OD_PERSIST_COMM, CO_EEP_START_PROT + MC_EEP_PAGE_SIZE,
                                          sizeof(OD_PERSIST_COMM_t));
    uint16_t crc = crc16_ccitt((uint8_t*)&OD_PERSIST_COMM, sizeof(OD_PERSIST_COMM_t), 0);

    /* Verify, if data in eeprom are equal */
    uint16_t crc_read = CO_eeprom_getCrcBlock(NULL, CO_EEP_START_PROT + MC_EEP_PAGE_SIZE, sizeof(OD_PERSIST_COMM_t));
    if ((crc != crc_read) || !writeOk) {
        return false;
    }

    /* Write signature (see CO_storageEeprom_init() for info) */
    uint16_t signatureOfEntry = sizeof(OD_PERSIST_COMM_t);
    uint32_t signature = (((uint32_t)crc) << 16) | signatureOfEntry;
    writeOk = CO_eeprom_writeBlock(NULL, (uint8_t*)&signature, CO_EEP_START_PROT, sizeof(signature));

    /* verify signature and write */
    uint32_t signatureRead;
    CO_eeprom_readBlock(NULL, (uint8_t*)&signatureRead, CO_EEP_START_PROT, sizeof(signatureRead));
    if ((signature != signatureRead) || !writeOk) {
        return false;
    }

    return true;
}

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */
