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
/**
 * ATTENTION!!!
 * - EEPROM chip is accessible only if MPU6050 sensor is connected to the I2C bus.
 *   Otherwise pull-up resistors are missing on SDA and SCL lines, therefore
 *   communication with EEPROM chip is impossible.
 */

#include "ch.h"
#include "hal.h"

#include "mpu6050.h"
#include "pwmio.h"
#include "telemetry.h"
#include "misc.h"
#include "attitude.h"
#include "eeprom.h"

/* C libraries: */
#include <string.h>


typedef struct tagEEPROMStruct {
  uint8_t addr;
  uint8_t *data;
  size_t dataSize;
} __attribute__((packed)) EEPROMStruct, *PEEPROMStruct;

/**
 * Global variables
 */
/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;

/**
 * Local variables
 */
static EEPROMStruct eeprom;

/**
 * @brief  Writes data buffer to specified EEPROM chip address.
 * @detail EEPROM accepts buffer write operations with only one page
 *         in size at a time.
 * @param  pEEPROM - pointer to EEPROM data structure.
 * @return 1 - if write operation is successful;
 *         0 - if write operation fails.
 */
static uint8_t eepromWriteData(const PEEPROMStruct pEEPROM) {
  uint8_t addrOffset;
  size_t numBytes;
  uint8_t eepromTXBuf[EEPROM_24C02_PAGE_SIZE + 1];
  msg_t status = RDY_OK;

  eepromTXBuf[0] = pEEPROM->addr;
  addrOffset = pEEPROM->addr % EEPROM_24C02_PAGE_SIZE;
  if (addrOffset) { /* Write address is not page aligned; */
    if (pEEPROM->dataSize < (size_t)(EEPROM_24C02_PAGE_SIZE - addrOffset)) {
      numBytes = pEEPROM->dataSize;
    } else {
      numBytes = EEPROM_24C02_PAGE_SIZE - addrOffset;
    }
    memcpy((void *)&eepromTXBuf[1], (void *)pEEPROM->data, numBytes);
    i2cAcquireBus(&I2CD2);
    status = i2cMasterTransmitTimeout(&I2CD2, EEPROM_24C02_ADDR, eepromTXBuf, numBytes + 1,
      NULL, 0, MS2ST(EEPROM_WRITE_TIMEOUT_MS));
    i2cReleaseBus(&I2CD2);
    if (status != RDY_OK) {
      g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
      if (g_i2cErrorInfo.last_i2c_error) {
        g_i2cErrorInfo.i2c_error_counter++;
        debugLog("E:eeprom-wr1");
      }
      return 0;
    }
    /* If write is successful, update variables for the next write operation. */
    pEEPROM->addr     += numBytes;
    pEEPROM->data     += numBytes;
    pEEPROM->dataSize -= numBytes;
  } else { /* Write address is page aligned; */
    if (pEEPROM->dataSize > EEPROM_24C02_PAGE_SIZE) {
      memcpy((void *)&eepromTXBuf[1], (void *)pEEPROM->data, EEPROM_24C02_PAGE_SIZE);
      i2cAcquireBus(&I2CD2);
      status = i2cMasterTransmitTimeout(&I2CD2, EEPROM_24C02_ADDR, eepromTXBuf, EEPROM_24C02_PAGE_SIZE + 1,
        NULL, 0, MS2ST(EEPROM_WRITE_TIMEOUT_MS));
      i2cReleaseBus(&I2CD2);
      if (status != RDY_OK) {
        g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
        if (g_i2cErrorInfo.last_i2c_error) {
          g_i2cErrorInfo.i2c_error_counter++;
          debugLog("E:eeprom-wr2");
        }
        return 0;
      }
      /* If write is successful, update variables for the next write operation. */
      pEEPROM->addr     += EEPROM_24C02_PAGE_SIZE;
      pEEPROM->data     += EEPROM_24C02_PAGE_SIZE;
      pEEPROM->dataSize -= EEPROM_24C02_PAGE_SIZE;
    } else {
      memcpy((void *)&eepromTXBuf[1], (void *)pEEPROM->data, pEEPROM->dataSize);
      i2cAcquireBus(&I2CD2);
      status = i2cMasterTransmitTimeout(&I2CD2, EEPROM_24C02_ADDR, eepromTXBuf, pEEPROM->dataSize + 1,
        NULL, 0, MS2ST(EEPROM_WRITE_TIMEOUT_MS));
      i2cReleaseBus(&I2CD2);
      if (status != RDY_OK) {
        g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
        if (g_i2cErrorInfo.last_i2c_error) {
          g_i2cErrorInfo.i2c_error_counter++;
          debugLog("E:eeprom-wr3");
        }
        return 0;
      }
      pEEPROM->dataSize = 0;
    }
  }
  return 1;
}

uint8_t eepromWriteBlock(uint16_t addr, const void *data, size_t dataSize) {
   while(eepromIsDataLeft()) {
      chThdSleepMilliseconds(10);
   }
   /*uint8_t buff[EEPROM_24C02_SIZE] = {0};
   eeprom.addr = 0x00;
   eeprom.data = buff;
   eeprom.dataSize = EEPROM_24C02_SIZE;
   uint8_t dump[EEPROM_24C02_SIZE];
   i2cAcquireBus(&I2CD2);
   i2cMasterTransmitTimeout(&I2CD2, EEPROM_24C02_ADDR, 0x00, 1, (uint8_t *)dump, EEPROM_24C02_SIZE, MS2ST(20));
   i2cReleaseBus(&I2CD2);*/
   eeprom.addr     = (uint8_t)addr;
   eeprom.data     = (uint8_t *)data;
   eeprom.dataSize = dataSize;
   return eepromWriteData(&eeprom);
}

uint8_t eepromReadBlock(void *data, uint16_t addr, size_t dataSize) {
   while(eepromIsDataLeft()) { //don't read if we haven't finished writing
        chThdSleepMilliseconds(10);
     }
   msg_t status = RDY_OK;
   uint8_t address = (uint8_t)addr;
   i2cAcquireBus(&I2CD2);
   status = i2cMasterTransmitTimeout(&I2CD2, EEPROM_24C02_ADDR, &address, 1, (uint8_t *)data, dataSize, MS2ST(EEPROM_READ_TIMEOUT_MS));
   i2cReleaseBus(&I2CD2);

   if (status != RDY_OK) {
      g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
      if (g_i2cErrorInfo.last_i2c_error) {
        debugLog("E:eeprom-load");
        g_i2cErrorInfo.i2c_error_counter++;
      }
      return 0;
    }
   return 1;
}

/**
 * @brief  Continues saving all user defined settings to external EEPROM chip.
 * @return 1 - if write operation is successful;
 *         0 - if write operation fails.
 */
uint8_t eepromContinueSaving(void) {
  return eepromWriteData(&eeprom);
}

/**
 * @brief
 * @return 1 - if there is data left to write;
 *         0 - if there is no data left to write.
 */
uint8_t eepromIsDataLeft(void) {
  return (eeprom.dataSize > 0);
}
