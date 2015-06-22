/*
    EvvGC-PLUS - Copyright (C) 2013-2015 Sarunas Vaitekonis

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _EEPROM_H_
#define _EEPROM_H_

/* Address of the 24C02 EEPROM chip: 1 0 1 0 1 1 1; */
#define EEPROM_24C02_ADDR       0x57
/* Size of the chip is 256 bytes (2048 bits or 2kbit); */
#define EEPROM_24C02_SIZE       0x0100
/* 8 Bytes per page; */
#define EEPROM_24C02_PAGE_SIZE  0x08
/* I2C read transaction time-out in milliseconds. */
#define EEPROM_READ_TIMEOUT_MS  0x05
/* I2C write transaction time-out in milliseconds. */
#define EEPROM_WRITE_TIMEOUT_MS 0x01
/* The beginning of the EEPROM. */
#define EEPROM_START_ADDR       0x00

#ifdef __cplusplus
extern "C" {
#endif
uint8_t eepromLoadSettings(void);
uint8_t eepromSaveSettings(void);
uint8_t eepromContinueSaving(void);
uint8_t eepromIsDataLeft(void);
uint8_t eepromWriteBlock(uint16_t addr, const void *data, size_t dataSize);
uint8_t eepromReadBlock(void *data, uint16_t addr, size_t dataSize);
#ifdef __cplusplus
}
#endif

#endif /* _EEPROM_H_ */
