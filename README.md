# CANopenNode STM32 with EEPROM storage

The EEPROM storage extension is designed to work with a Microchip 24AA256UID i2c EEPROM device.

The use of the 24AA256UID requires an i2c port, which was not defined in some of the example projects. In those specific projects, an i2c port was added in the STM32CubeMX design of each project. The port handle of the chosen EEPROM i2c port should be defined in file i2c.h. In projects that do not contain a i2c.h file, the definition should be made in main.h. If i2c.h is present, it should be added as an include in main.h.

There is also a GPIO input port defined "CAN_CFG" with internal pullup resistor activated. Externally pulling this input to ground at startup
causes the firmware to skip readout of the EEPROM and use the default persist_comm OD parameters.

At first run, the EEPROM does not yet contain a valid configuration. The EEPROM will automatically be primed with the default persist_comm OD parameters.

The Microchip 24AA256UID contains a built-in serial number. This serial number is used to fill the serial number object (index 0x1018 subindex 0x04) in the object dictionary.

Modified persist_comm objects are stored into EEPROM by writing value UNSIGNED32 0x65766173 to object 0x1010 subindex 0x01.
