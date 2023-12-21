/*
 * Eeprom interface for use with CO_storageEeprom, STM32 specific
 *
 * @file        CO_eepromSTM32.c
 * @author      Vegard Sømliøy
 * @copyright   2023 Vegard Sømliøy
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

//#include "cmsis_os.h"
#include "301/crc16-ccitt.h"
#include "i2c.h"
#include "storage/CO_eeprom.h"

#include <stdlib.h>

/*
 * Hardware definition
 */

#ifndef EEPROM
#define EEPROM
#define EEPROM_ADDRESS     0xA0
#define EEPROM_BUFFER_SIZE 0x2000 // 64 kbit = 0x2000, 128 = 0x4000, 256 = 0x8000
#define EEPROM_PAGE_SIZE   32     // Page size
#define EEPROM_WRITE_TIME  5      // Page write time in ms
#define EEPROM_TIMEOUT     200    // timeout for write
#endif

/*
 * Eeprom is configured so, that first half of memory locations is not write
 * protected, so it is suitable for auto storage variables. Second half of
 * memory locations is write protected. It is used for storage on command, which
 * disables the protection for the short time when writing. Below are two
 * internal variables, used for indicating next free address in eeprom, one for
 * autonomous storage and one for protected storage
 */
static size_t eepromAddrNextAuto = 0;
static size_t eepromAddrNextProt = EEPROM_BUFFER_SIZE / 2;

I2C_HandleTypeDef* eeprom_i2c = &hi2c2;

/******************************************************************************/
bool_t
CO_eeprom_init(void* storageModule) {

    eepromAddrNextAuto = 0;
    eepromAddrNextProt = EEPROM_BUFFER_SIZE / 2;

    return HAL_I2C_IsDeviceReady(eeprom_i2c, EEPROM_ADDRESS, 10, EEPROM_TIMEOUT) == HAL_OK;
}

/******************************************************************************/
size_t
CO_eeprom_getAddr(void* storageModule, bool_t isAuto, size_t len, bool_t* overflow) {
    size_t addr;

    if (isAuto) {
        /* auto storage is processed byte by byte, no alignment necessary */
        addr = eepromAddrNextAuto;
        eepromAddrNextAuto += len;
        if (eepromAddrNextAuto > (EEPROM_BUFFER_SIZE / 2)) {
            *overflow = true;
        }
    } else {
        /* addresses for storage on command */
        addr = eepromAddrNextProt;
        eepromAddrNextProt += len;

        if (eepromAddrNextProt > EEPROM_BUFFER_SIZE) {
            *overflow = true;
        }
    }

    return addr;
}

/******************************************************************************/
void
CO_eeprom_readBlock(void* storageModule, uint8_t* data, size_t eepromAddr, size_t len) {
    HAL_StatusTypeDef Result = HAL_OK;
    uint16_t u16ByteCounter = 0;

    while (HAL_I2C_IsDeviceReady(eeprom_i2c, EEPROM_ADDRESS, 10, EEPROM_TIMEOUT) != HAL_OK);

    while (u16ByteCounter < len && Result == HAL_OK) {

        uint16_t u16BytesToRead = len - u16ByteCounter;

        if (u16BytesToRead >= EEPROM_PAGE_SIZE) {
            // More Pages
            Result = HAL_I2C_Mem_Read(eeprom_i2c, EEPROM_ADDRESS, eepromAddr + u16ByteCounter, I2C_MEMADD_SIZE_16BIT,
                                      &data[u16ByteCounter], EEPROM_PAGE_SIZE, EEPROM_TIMEOUT);
            u16ByteCounter += EEPROM_PAGE_SIZE;
        } else {
            // Less than one page
            Result = HAL_I2C_Mem_Read(eeprom_i2c, EEPROM_ADDRESS, eepromAddr + u16ByteCounter, I2C_MEMADD_SIZE_16BIT,
                                      &data[u16ByteCounter], u16BytesToRead, EEPROM_TIMEOUT);
            u16ByteCounter += u16BytesToRead;
        }

        if (Result != HAL_OK) {
            break;
        }
    }
}

/******************************************************************************/
bool_t
CO_eeprom_writeBlock(void* storageModule, uint8_t* data, size_t eepromAddr, size_t len) {

    HAL_StatusTypeDef Result = HAL_OK;
    uint16_t u16ByteCounter = 0;

    while (u16ByteCounter < len && Result == HAL_OK) {
        while (HAL_I2C_IsDeviceReady(eeprom_i2c, EEPROM_ADDRESS, 10, EEPROM_TIMEOUT) != HAL_OK);

        uint16_t u16BytesToWrite = EEPROM_PAGE_SIZE - (eepromAddr + u16ByteCounter) % EEPROM_PAGE_SIZE;

        if ((u16BytesToWrite + u16ByteCounter) > len) {
            u16BytesToWrite = len - u16ByteCounter;
        }

        Result = HAL_I2C_Mem_Write(eeprom_i2c, EEPROM_ADDRESS, eepromAddr + u16ByteCounter, I2C_MEMADD_SIZE_16BIT,
                                   &data[u16ByteCounter], u16BytesToWrite, EEPROM_TIMEOUT);
        u16ByteCounter += u16BytesToWrite;

        if (Result != HAL_OK) {
            break;
        }
    }

    return Result == HAL_OK;
}

/******************************************************************************/
uint16_t
CO_eeprom_getCrcBlock(void* storageModule, size_t eepromAddr, size_t len) {
    uint16_t crc = 0;
    uint8_t data[EEPROM_PAGE_SIZE];
    size_t addr = eepromAddr;

    while (len > 0) {
        size_t subLen = len < sizeof(data) ? len : sizeof(data);

        /* update crc from data part */
        CO_eeprom_readBlock(storageModule, data, addr, subLen);

        crc = crc16_ccitt(data, subLen, crc);
        len -= subLen;
        addr += subLen;
    }

    return crc;
}

/******************************************************************************/
bool_t
CO_eeprom_updateBlock(void* storageModule, uint8_t* data, size_t eepromAddr, size_t len) {
    bool_t Result = false;

    /* Verify, if data in eeprom are equal*/
    uint16_t data_rx_crc = crc16_ccitt(data, len, 0);
    uint16_t data_crc = CO_eeprom_getCrcBlock(storageModule, eepromAddr, len);

    /* If data in EEPROM differs, then write it to EEPROM. */
    if (data_rx_crc != data_crc) {
        Result = CO_eeprom_writeBlock(storageModule, data, eepromAddr, len);
    }

    return Result;
}

bool_t
CO_eeprom_updateByte(void* storageModule, uint8_t data, size_t eepromAddr) {
    /* read data byte from eeprom */
    uint8_t data_rx;
    bool_t Result = false;

    CO_eeprom_readBlock(storageModule, &data_rx, eepromAddr, 1);

    /* If data in EEPROM differs, then write it to EEPROM. */
    if (data_rx != data) {
        Result = CO_eeprom_writeBlock(storageModule, &data, eepromAddr, 1);
    }

    return Result;
}
