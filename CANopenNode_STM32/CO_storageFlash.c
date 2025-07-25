/**
 * To use this driver:
 * - Define USE_CO_STORAGE_FLASH in your build config, this will enable the following defines in CO_driver_target.h:
#define CO_CONFIG_STORAGE CO_CONFIG_STORAGE_ENABLE
#define CO_CONFIG_CRC16 CO_CONFIG_CRC16_ENABLE

 * - Exclude CANopenNode/storage/CO_storageEeprom.c from the build, if it causes build errors.
 * - Make sure Object Dictionary entries for 0x1010 "Store Parameters" and 0x1011 "Restore default parameters" are enabled
 * - Call CO_storageFlash_init in canopen_app_init (where CO_storageBlank_init is called in the example code)
 * - Modify your linker script to have a NVM section of at least 2KB and a _co_storage_start symbol (see examples/stm32f3xx_can_rtos/STM32F303VCTX_FLASH.ld)
 */
#ifdef USE_CO_STORAGE_FLASH
#include "CO_storageFlash.h"
#include "301/crc16-ccitt.h"
#include "stm32f3xx_hal_flash.h"
#include "stm32f3xx_hal_flash_ex.h"
#include "stm32f3xx.h"

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE

#define SIZE_CRC 2
#define SIZE_FIRST_ADDRESS 2 // First uint16_t is used to signal whether the flash was written by writing a '1'

static ODR_t restoreFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule);
static ODR_t storeFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule);
static ODR_t writePageBuffer(void);
static ODR_t erasePage(void);

static uint8_t flashPageBuffer[SIZE_OF_PAGE];

/**
 * @brief Function for writing data on "Store parameters" command - OD object 1010. For more information see file
 * CO_storage.h, CO_storage_entry_t.
 * @note This is implemented as read, modify write as multiple storage groups are stored on one page
 * @param entry storage entry
 * @param CANmodule pointer to CO_CANmodule_t structure
 * @return ODR_OK on success
 */
static ODR_t storeFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
  (void)CANmodule;
  // Round up the offset to the nearest even integer
  // This is because we write 16 bytes and length could a odd amount of bytes
  size_t roundedLength = (entry->len + 1) & ~1;

  // Out of memory
  if (roundedLength + SIZE_CRC + SIZE_FIRST_ADDRESS > entry->reservedSpace)
  {
    return ODR_OUT_OF_MEM;
  }

  uint8_t *flash_pointer = (uint8_t *)&_co_storage_start;

  memcpy(flashPageBuffer, flash_pointer, SIZE_OF_PAGE);

  // Mark the storage entry as written
  flashPageBuffer[entry->offset] = 1;
  flashPageBuffer[entry->offset + 1] = 0;

  memcpy(&flashPageBuffer[entry->offset + SIZE_FIRST_ADDRESS], (uint8_t *)entry->addr, entry->len);

  // Calculate CRC
  entry->crc = crc16_ccitt(entry->addr, entry->len, 0);

  // Write the CRC after the data
  size_t crcAddress = entry->offset + roundedLength + SIZE_FIRST_ADDRESS;
  flashPageBuffer[crcAddress] = (uint8_t)(entry->crc & 0xFF);
  flashPageBuffer[crcAddress + 1] = (uint8_t)((entry->crc >> 8) & 0xFF);

  // Write the modified page back to flash
  ODR_t ret = writePageBuffer();
  if (ret != ODR_OK)
  {
    return ret;
  }

  uint16_t flashCRC = flash_pointer[crcAddress] | (flash_pointer[crcAddress + 1] << 8);
  // Compare CRC values
  if (flashCRC != entry->crc)
  {
    ret = ODR_HW; // CRC mismatch, data not written successfully
  }

  return ret;
}

/**
 * @brief Write the page buffer to flash
 * @return ODR_OK on success
 */
static ODR_t writePageBuffer(void)
{
  if (erasePage() != ODR_OK)
  {
    return ODR_HW;
  }

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return ODR_NO_RESOURCE;
  }

  uint16_t *flash_pointer = (uint16_t *)&_co_storage_start;

  for (size_t i = 0; i < SIZE_OF_PAGE; i += 2)
  {
    uint64_t toWrite = (flashPageBuffer[i + 1] << 8) | flashPageBuffer[i];
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)flash_pointer++, (uint64_t)toWrite) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return ODR_HW;
    }
  }

  HAL_FLASH_Lock();

  return ODR_OK;
}

/**
 * @brief Function for restoring data on "Restore default parameters" command - OD 1011. For more information see file
 * CO_storage.h, CO_storage_entry_t.
 * @note Only restores to default after reset
 * @param entry storage entry
 * @param CANmodule pointer to CO_CANmodule_t structure
 * @return ODR_OK on success
 */
static ODR_t restoreFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
  (void)CANmodule;
  // Read page from flash
  uint8_t *flash_pointer = (uint8_t *)&_co_storage_start;
  memcpy(flashPageBuffer, flash_pointer, SIZE_OF_PAGE);

  // Set entry as not written to
  flashPageBuffer[entry->offset] = 0;

  // Write the modified page back to flash
  return writePageBuffer();
}

static ODR_t erasePage(void)
{
  FLASH_EraseInitTypeDef eraseIfo;
  eraseIfo.PageAddress = (uint32_t)&_co_storage_start;
  eraseIfo.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseIfo.NbPages = 1;

  uint32_t eraseError = 0;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return ODR_NO_RESOURCE;
  }

  // Returns 0xFFFFFFFF on success
  HAL_FLASHEx_Erase(&eraseIfo, &eraseError);
  HAL_FLASH_Lock();

  if (eraseError == 0xFFFFFFFF)
  {
    return ODR_OK;
  }

  return ODR_HW;
}

/**
 * @brief Init the storage flash module
 * @param storage CO_storage_t pointer
 * @param CANmodule CO_CANmodule_t CO_CANmodule_t
 * @param OD_1010_StoreParameters pointer to OD entry for store parameters
 * @param OD_1011_RestoreDefaultParam pointer to OD entry for restore parameters
 * @param entries pointer to CO storage entries
 * @param entriesCount number of CO storage entries
 * @param storageInitError output parametet to indicate error
 * @return CO_ERROR_NO on success
 */
CO_ReturnError_t CO_storageFlash_init(CO_storage_t *storage, CO_CANmodule_t *CANmodule,
                                      OD_entry_t *OD_1010_StoreParameters, OD_entry_t *OD_1011_RestoreDefaultParam,
                                      CO_storage_entry_t *entries, uint8_t entriesCount, uint32_t *storageInitError)
{
  CO_ReturnError_t ret = CO_ERROR_NO;

  /* verify arguments */
  if (storage == NULL || entries == NULL || entriesCount == 0 || storageInitError == NULL)
  {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  storage->enabled = false;

  /* initialize storage and OD extensions */
  ret = CO_storage_init(storage, CANmodule, OD_1010_StoreParameters, OD_1011_RestoreDefaultParam, storeFlash,
                        restoreFlash, entries, entriesCount);

  if (ret != CO_ERROR_NO)
  {
    return ret;
  }

  /* initialize entries */
  *storageInitError = 0;

  for (uint8_t i = 0; i < entriesCount; i++)
  {
    CO_storage_entry_t *entry = &entries[i];

    /* verify arguments */
    if (entry->addr == NULL || entry->len == 0 || entry->subIndexOD < 2)
    {
      *storageInitError = i;
      return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    // Round up the offset to the nearest even integer
    // This is because we write 16 bytes and offset could a odd amount of bytes
    size_t roundedLength = (entry->len + 1) & ~1;

    if (roundedLength + SIZE_CRC + SIZE_FIRST_ADDRESS > entry->reservedSpace)
    {
      *storageInitError = i;
      return CO_ERROR_OUT_OF_MEMORY;
    }

    uint16_t *flash_pointer = ((uint16_t *)&_co_storage_start) + (entry->offset / sizeof(uint16_t));

    // If a 1 is at the first address than something is written in Flash
    // Else keep the defaults
    if (*flash_pointer == 1U)
    {
      // Move the pointer to the second address
      flash_pointer++;

      // Read the data from flash memory
      memcpy((uint8_t *)entry->addr, (uint8_t *)flash_pointer, entry->len);

      // Calculate CRC
      entry->crc = crc16_ccitt(entry->addr, entry->len, 0);

      // Read the CRC to verify
      uint16_t read_crc = *(flash_pointer + (roundedLength) / sizeof(uint16_t));

      if (read_crc != entry->crc)
      {
        uint32_t errorBit = entry->subIndexOD;
        if (errorBit > 31)
          errorBit = 31;
        *storageInitError |= (1U) << errorBit;
        return CO_ERROR_DATA_CORRUPT;
      }
    }
  }
  storage->enabled = true;

  return ret;
}

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */

#endif
