/*
 * eeprom.c
 *
 *  Created on: 11 jun 2026
 *      Author: Marc Vandenhende
 */

#include "eeprom.h"
#include "i2c.h"
#include "CO_app_STM32.h"
#include "OD.h"

/*
 * Hardware definition
 */

#define CO_EEP_MAX_STORAGE  0x2000  // Max number of bytes reserved for CanOpen storage

#define	EEP_MEM_I2C_ADDR	0x50	// I2C address of eeprom device
#define	EEP_UID_I2C_ADDR	0x58    // I2C address of ST eeprom UID

#define	ST_MFR_CODE	        0x20	// ST manufacturer code
#define	ST_BUS_PROTOCOL	    0xE0	// ST bus protocol
#define ST_UID_SIZE 	    16		// ST UID size in #bytes

#define MC_SERIAL_ADDR      0x7ffa	// memory location of MC serial number
#define	MC_MFR_CODE	        0x29	// MC manufacturer code
#define	MC_DEVICE_CODE	    0x48	// MC device code
#define MC_EEP_SERIAL_SIZE	6		// MC Serial size in #bytes

typedef struct
{
    uint8_t density;
    uint32_t  storage;
    uint16_t  page_size;
} eeprom_st_t;

/*
 * Eeprom device table
 * For each recognized type, the table contains the maximum
 * device capacity and the page size for eeprom writing
 */
const eeprom_st_t eeprom_st[] =
{
    {0x0c, 0x1000, 32},   // m24c32-u
    {0x0d, 0x2000, 32},   // m24c64-u
    {0x0e, 0x4000, 64},   // m24128-u
    {0x0f, 0x8000, 64},   // m24256-u
    {0x10, 0x10000, 128}, // m24512-u
};

bool eeprom_initialized = false;

uint32_t device_size          = 0;
uint16_t page_size            = 0;
uint16_t device_i2c_address   = EEP_MEM_I2C_ADDR;
uint32_t device_serial_number = 0;

uint32_t data_storage_start = 0;
uint32_t data_storage_size  = 0;

uint32_t device_start_co_auto = 0;
uint32_t device_start_co_prot = 0;
uint32_t device_size_co       = 0;

static bool_t eeprom_init_mc();
static bool_t eeprom_init_st();

/*
 * Main EEPROM initialization routine
 * returns "true" if initialization is success
 * returns "false" if initialization fails or no eeprom present
 */
bool Eeprom_Init()
{
    if (!eeprom_init_mc())
    {
        if (!eeprom_init_st())
        {
            // no EEPROM found
            return false;
        }
    }

    /* If eeprom chip is OK, this will pass, otherwise timeout */
    eeprom_initialized = HAL_I2C_IsDeviceReady(HI2C_EEPROM, EEP_MEM_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) == HAL_OK;

    return eeprom_initialized;
}

/*
 * Canopen EEPROM initialization routine
 * Assumes eeprom_init() was already called before.
 * returns "true" if initialization is success
 * returns "false" if initialization fails or no eeprom present
 */
bool Eeprom_Init_CO()
{
    if (!eeprom_initialized)
        return false;

    OD_PERSIST_COMM.x1018_identity.serialNumber = device_serial_number;
    /* If eeprom chip is OK, this will pass, otherwise timeout */
    return (HAL_I2C_IsDeviceReady(HI2C_EEPROM, EEP_MEM_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) == HAL_OK);
    // return "true" if device ready
}

/*
 * Try to initialize ST eeprom
 * Looks for the right device and sets parameters accordingly
 * returns "true" if initialization is success
 * returns "false" if no ST eeprom found
 */
static bool_t eeprom_init_st()
{
    uint8_t uid[ST_UID_SIZE];

    /* If eeprom chip is OK, this will pass, otherwise timeout */
    if (HAL_I2C_IsDeviceReady(HI2C_EEPROM, EEP_UID_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) != HAL_OK)
        return false; // return "false" if device not ready

    if (HAL_I2C_Mem_Read(HI2C_EEPROM, EEP_UID_I2C_ADDR << 1, 0, 2, uid, ST_UID_SIZE, I2C_TIMEOUT_MS) != HAL_OK)
        return false; // return "false" if device does not respond

    // Check if STM EEPROM device
    if ((uid[0] == ST_MFR_CODE) && (uid[1] == ST_BUS_PROTOCOL))
    {
        // loop through table, set parameters when match found
        for (int i = 0; i < sizeof(eeprom_st) / sizeof(eeprom_st_t); i++)
        // loop through table, set parameters when match found
        {
            if (eeprom_st[i].density == uid[2])
            {
                device_size = eeprom_st[i].storage;
                page_size   = eeprom_st[i].page_size;

                // set serial number
                device_serial_number = (uint32_t) uid[12] << 24
                                       | (uint32_t) uid[13] << 16
                                       | (uint32_t) uid[14] << 8
                                       | (uint32_t) uid[15];

                // If device > 8 kbytes, use first 8 kbytes of storage for CanOpen
                if (device_size > CO_EEP_MAX_STORAGE)
                {
                    // canopen device size values
                    device_size_co       = CO_EEP_MAX_STORAGE;
                    device_start_co_auto = 0x0000;
                    device_start_co_prot = CO_EEP_MAX_STORAGE / 2;

                    // general storage values
                    data_storage_start = CO_EEP_MAX_STORAGE;
                    data_storage_size  = device_size - CO_EEP_MAX_STORAGE;
                }
                else
                {
                    // canopen device size values
                    device_size_co       = device_size;
                    device_start_co_auto = 0x0000;
                    device_start_co_prot = device_size_co / 2;

                    // general storage values -> no storage available
                    data_storage_start = device_size_co;
                    data_storage_size  = 0x0000;
                }

                return true;
            }
        }
    }

    return false;
}

/*
 * Try to initialize Microchip eeprom
 * Check if device is 24AA256UID and sets parameters accordingly
 * returns "true" if initialization is success
 * returns "false" if no MC eeprom found
 */
static bool_t eeprom_init_mc()
{
    uint8_t serial[MC_EEP_SERIAL_SIZE];

    /* If eeprom chip is OK, this will pass, otherwise timeout */
    if (HAL_I2C_IsDeviceReady(HI2C_EEPROM, EEP_MEM_I2C_ADDR << 1, 3, I2C_TIMEOUT_MS) != HAL_OK)
        return false; // return "false" if device not ready

    if (HAL_I2C_Mem_Read(HI2C_EEPROM, EEP_MEM_I2C_ADDR << 1, MC_SERIAL_ADDR, 2, serial, MC_EEP_SERIAL_SIZE, I2C_TIMEOUT_MS) !=
        HAL_OK)
        return false; // return "false" if device does not respond

    // Check if STM EEPROM device
    if ((serial[0] == MC_MFR_CODE) && (serial[1] == MC_DEVICE_CODE))
    {
        // set serial number
        device_serial_number = (uint32_t) serial[2] << 24
                               | (uint32_t) serial[3] << 16
                               | (uint32_t) serial[4] << 8
                               | (uint32_t) serial[5];

        // device size is 32 kbytes (0x8000)
        // but range 0x7000 -> 0x7fff not available for storage
        // using first 8 kbytes of storage for CanOpen
        device_size = 0x7000;
        page_size   = 64;

        // canopen device size values
        device_size_co       = CO_EEP_MAX_STORAGE;
        device_start_co_auto = 0x0000;
        device_start_co_prot = CO_EEP_MAX_STORAGE / 2;

        // general storage values
        data_storage_start = CO_EEP_MAX_STORAGE;
        data_storage_size  = device_size - CO_EEP_MAX_STORAGE;

        return true;
    }

    return false;
}
