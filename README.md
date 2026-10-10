# CANopenNode STM32 with EEPROM storage extension

The EEPROM storage extension is designed to work with a Microchip 24AA256UID i2c EEPROM device.

The storage_blank.c/h files have been replaced by "CO_eeprom_STM32.c/h" with 2 additional files "eeprom.c/h" which contain some low level functions.
CO_app_* and CO_driver_* had a couple of modifications to allow storage/readback of the eeprom data.

The use of the 24AA256UID EEPROM requires an i2c port. Additionally, a digital input "CAN_CFG" can be used to trick the EEPROM routines into thinking that the EEPROM is uninitialized, which can be handy during development.
The CAN_CFG input is active "low" and is to be defined in Cube_MX as a GPIO input with pullup resistor active. It is best to connect the input to a small pushbutton switch on the PCB.

The example application "stm32g0xx_fdcan" was modified to use the EEPROM storage. The EEPROM should be connected to the in this project already existing I2C bus (I2C1) with all address pins tied to GND. A CAN_CFG gpio input pin was defined with active pullup resistor at PB4.
At first run, the EEPROM does not yet contain a valid configuration. The EEPROM will automatically be primed with the default persist_comm OD parameters. This function is disabled when input CAN_CFG is active.

The Microchip 24AA256UID contains a built-in serial number. This serial number is used to fill the serial number object (index 0x1018 subindex 0x04) in the object dictionary.

Modified persist_comm objects are stored into EEPROM by writing value UNSIGNED32 0x65766173 to object 0x1010 subindex 0x01.


Only changes required to user code:

in main.h:
If the application uses a separate pair of .c/.h files per peripheral, add...
#include "i2c.h"
...to main.h. If your application only consists of a main.c/main.h, this is not needed.

in main.h add:

#ifndef CO_CONFIG_STORAGE
#define CO_CONFIG_STORAGE (CO_CONFIG_STORAGE_ENABLE)
#endif

in eeprom.h:
Make sure that HI2C_EEPROM matches the I2C port handle used for the EEPROM.
#define HI2C_EEPROM          &hi2c1
