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

typedef struct tagEEPROMStruct {
  PIDSettings pidSettings[3];       /*  9 bytes */
  PWMOutputStruct pwmOutput[3];     /* 12 bytes */
  MixedInputStruct mixedInput[3];   /* 21 byte  */
  InputModeStruct modeSettings[3];  /* 24 bytes */
  uint8_t sensorSettings[3];        /*  3 bytes */
  float accelBias[3];               /* 12 bytes */
  float gyroBias[3];                /* 12 bytes */
  uint32_t crc32;                   /*  4 bytes */
/* TOTAL:                              97 bytes */
/* Bytes left:                        159 bytes */
} __attribute__((packed)) EEPROMStruct, *PEEPROMStruct;

/**
 * Global variables
 */
/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;

/**
 * Local variables
 */
static size_t dataSizeLeft = 0;
static uint8_t newAddr;
static uint8_t *pDataAddr;
static uint8_t fSkipContinue = 0;

static EEPROMStruct eepromData;
static uint8_t eepromTXBuf[EEPROM_24C02_PAGE_SIZE + 1];

/**
 * @brief  Writes data buffer to specified EEPROM chip address.
 * @detail EEPROM accepts buffer write operations with only one page
 *         in size at a time.
 * @param  addr - EEPROM memory address.
 * @param  pData - pointer to data buffer.
 * @param  size - size of data buffer.
 * @return 1 - if write operation is successful;
 *         0 - if write operation fails.
 */
static uint8_t eepromWriteData(uint8_t addr, uint8_t *pData, size_t size) {
  uint8_t addrOffset;
  size_t numBytes;
  msg_t status = RDY_OK;

  eepromTXBuf[0] = addr;
  addrOffset = addr % EEPROM_24C02_PAGE_SIZE;
  if (addrOffset) { /* Write address is not page aligned; */
    if (size < (size_t)(EEPROM_24C02_PAGE_SIZE - addrOffset)) {
      numBytes = size;
    } else {
      numBytes = EEPROM_24C02_PAGE_SIZE - addrOffset;
    }
    memcpy((void *)&eepromTXBuf[1], (void *)pData, numBytes);
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
    /* If write is successful, update variables for next write operation. */
    dataSizeLeft = size - numBytes;
    newAddr = addr + numBytes;
    pDataAddr = pData + numBytes;
  } else { /* Write address is page aligned; */
    if (size > EEPROM_24C02_PAGE_SIZE) {
      memcpy((void *)&eepromTXBuf[1], (void *)pData, EEPROM_24C02_PAGE_SIZE);
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
      /* If write is successful, update variables for next write operation. */
      dataSizeLeft = size - EEPROM_24C02_PAGE_SIZE;
      newAddr = addr + EEPROM_24C02_PAGE_SIZE;
      pDataAddr = pData + EEPROM_24C02_PAGE_SIZE;
    } else {
      memcpy((void *)&eepromTXBuf[1], (void *)pData, size);
      i2cAcquireBus(&I2CD2);
      status = i2cMasterTransmitTimeout(&I2CD2, EEPROM_24C02_ADDR, eepromTXBuf, size + 1,
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
      dataSizeLeft = 0;
    }
  }
  return 1;
}

/**
 * @brief  Loads all user defined settings from external EEPROM chip.
 * @return 1 - if write operation is successful;
 *         0 - if write operation fails.
 */
uint8_t eepromLoadSettings(void) {
  msg_t status = RDY_OK;
  uint8_t addr = EEPROM_START_ADDR;

  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmitTimeout(&I2CD2, EEPROM_24C02_ADDR, &addr, 1,
    (uint8_t *)&eepromData, sizeof(eepromData), MS2ST(EEPROM_READ_TIMEOUT_MS));
  i2cReleaseBus(&I2CD2);

  if (status != RDY_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      debugLog("E:eeprom-load");
      g_i2cErrorInfo.i2c_error_counter++;
    }
    return 0;
  }

  if (eepromData.crc32 != crcCRC32((uint32_t *)&eepromData, sizeof(eepromData) / sizeof(uint32_t) - 1)) {
    /* Fill with default settings. */
    return eepromSaveSettings();
  } else {
    pidSettingsUpdate(eepromData.pidSettings);
    pwmOutputSettingsUpdate(eepromData.pwmOutput);
    mixedInputSettingsUpdate(eepromData.mixedInput);
    inputModeSettingsUpdate(eepromData.modeSettings);
    sensorSettingsUpdate(eepromData.sensorSettings);
    accelBiasUpdate(&g_IMU1, eepromData.accelBias);
    gyroBiasUpdate(&g_IMU1, eepromData.gyroBias);
  }

  return 1;
}

/**
 * @brief  Starts saving all user defined settings to external EEPROM chip.
 * @return 1 - if write operation is successful;
 *         0 - if write operation fails.
 */
uint8_t eepromSaveSettings(void) {
  memcpy((void *)eepromData.pidSettings, (void *)g_pidSettings, sizeof(g_pidSettings));
  memcpy((void *)eepromData.pwmOutput, (void *)g_pwmOutput, sizeof(g_pwmOutput));
  memcpy((void *)eepromData.mixedInput, (void *)g_mixedInput, sizeof(g_mixedInput));
  memcpy((void *)eepromData.modeSettings, (void *)g_modeSettings, sizeof(g_modeSettings));
  memcpy((void *)eepromData.sensorSettings, (void *)g_sensorSettings, sizeof(g_sensorSettings));
  memcpy((void *)eepromData.accelBias, (void *)g_IMU1.accelBias, sizeof(g_IMU1.accelBias));
  memcpy((void *)eepromData.gyroBias, (void *)g_IMU1.gyroBias, sizeof(g_IMU1.gyroBias));
  eepromData.crc32 = crcCRC32((uint32_t *)&eepromData, sizeof(eepromData) / sizeof(uint32_t) - 1);
  fSkipContinue = 1;
  return eepromWriteData(EEPROM_START_ADDR, (uint8_t *)&eepromData, sizeof(eepromData));
}

/**
 * @brief  Continues saving all user defined settings to external EEPROM chip.
 * @return 1 - if write operation is successful;
 *         0 - if write operation fails.
 */
uint8_t eepromContinueSaving(void) {
  if (fSkipContinue) {
    fSkipContinue = 0;
    return 1;
  }
  return eepromWriteData(newAddr, pDataAddr, dataSizeLeft);
}

/**
 * @brief
 * @return 1 - if there is data left to write;
 *         0 - if there is no data left to write.
 */
uint8_t eepromIsDataLeft(void) {
  return (dataSizeLeft > 0);
}