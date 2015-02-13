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

#include "ch.h"
#include "hal.h"

/* C libraries: */
#include <string.h>

#include "telemetry.h"
#include "attitude.h"
#include "pwmio.h"
#include "mpu6050.h"
#include "eeprom.h"
#include "misc.h"

/* Telemetry buffer size in bytes.  */
#define TELEMETRY_BUFFER_SIZE   32

/* Board status variable. */
extern uint32_t g_boardStatus;

/* Console input/output handle. */
BaseChannel *g_chnp;
/* Telemetry data header. */
static DataHdr dataHdr;
/* Data buffer for various telemetry IO operations. */
static uint8_t dataBuf[TELEMETRY_BUFFER_SIZE];

size_t bytesRequired = 0;

/**
 * @brief  Calculates CRC32 checksum of received data buffer.
 * @param  pHdr - pointer to data header structure.
 * @return CRC32 checksum of received zero-padded data buffer.
 */
static uint32_t telemetryGetCRC32Checksum(const PDataHdr pHdr) {
  uint32_t crc_length = pHdr->size / sizeof(uint32_t);
  if (pHdr->size % sizeof(uint32_t)) {
    crc_length++;
  }
  return crcCRC32((const uint32_t *)pHdr->data, crc_length);
}

/**
 * @brief
 * @details
 */
static void telemetrySendSerialData(const PDataHdr pHdr) {
  /* Sends command ID and data size. */
  chnWrite(g_chnp, (const uint8_t *)pHdr, 2);
  if (pHdr->size) {
    /* Sends actual data. */
    chnWrite(g_chnp, pHdr->data, pHdr->size);
    /* Sends cyclic redundancy check data. */
    chnWrite(g_chnp, (const uint8_t *)&pHdr->crc, sizeof(uint32_t));
  }
}

/**
 * @brief  Command processor.
 * @param  pHdr - pointer to data header structure to be processed.
 * @return none.
 */
static void telemetryProcessCommand(const PDataHdr pHdr) {
  switch (pHdr->cmd_id) {
  case 'D': /* Reads new sensor settings; */
    if ((pHdr->size == sizeof(g_sensorSettings)) && (pHdr->crc == telemetryGetCRC32Checksum(pHdr))) {
      sensorSettingsUpdate((PSensorStruct)pHdr->data);
    }
    break;
  case 'I': /* Reads new mixed input settings; */
    if ((pHdr->size == sizeof(g_mixedInput)) && (pHdr->crc == telemetryGetCRC32Checksum(pHdr))) {
      mixedInputSettingsUpdate((PMixedInputStruct)pHdr->data);
    }
    break;
  case 'M': /* Reads new input mode settings; */
    if ((pHdr->size == sizeof(g_modeSettings)) && (pHdr->crc == telemetryGetCRC32Checksum(pHdr))) {
      inputModeSettingsUpdate((PInputModeStruct)pHdr->data);
    }
    break;
  case 'O': /* Reads new output settings; */
    if ((pHdr->size == sizeof(g_pwmOutput)) && (pHdr->crc == telemetryGetCRC32Checksum(pHdr))) {
      pwmOutputSettingsUpdate((PPWMOutputStruct)pHdr->data);
    }
    break;
  case 'S': /* Reads new PID values; */
    if ((pHdr->size == sizeof(g_pidSettings)) && (pHdr->crc == telemetryGetCRC32Checksum(pHdr))) {
      pidSettingsUpdate((PPIDSettings)pHdr->data);
    }
    break;
  case 'a': /* Outputs accelerometer data; */
    memcpy((void *)dataBuf, (void *)g_accelData, sizeof(g_accelData));
    pHdr->size = sizeof(g_accelData);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'b': /* Outputs board status; */
    memcpy((void *)dataBuf, (void *)&g_boardStatus, sizeof(g_boardStatus));
    pHdr->size = sizeof(g_boardStatus);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'c': /* Saves settings to EEPROM; */
    if (eepromSaveSettings()) {
    }
    break;
  case 'd': /* Outputs sensor settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)dataBuf, 0, sizeof(dataBuf));
    memcpy((void *)dataBuf, (void *)g_sensorSettings, sizeof(g_sensorSettings));
    pHdr->size = sizeof(g_sensorSettings);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'g': /* Outputs gyroscope data; */
    memcpy((void *)dataBuf, (void *)g_gyroData, sizeof(g_gyroData));
    pHdr->size = sizeof(g_gyroData);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'h': /* Outputs motor offset data; */
    memcpy((void *)dataBuf, (void *)g_motorOffset, sizeof(g_motorOffset));
    pHdr->size = sizeof(g_motorOffset);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'i': /* Outputs input data values; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)dataBuf, 0, sizeof(dataBuf));
    memcpy((void *)dataBuf, (void *)g_inputValues, sizeof(g_inputValues));
    pHdr->size = sizeof(g_inputValues);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'm': /* Outputs input mode settings; */
    memcpy((void *)dataBuf, (void *)g_modeSettings, sizeof(g_modeSettings));
    pHdr->size = sizeof(g_modeSettings);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'o': /* Outputs PWM output settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)dataBuf, 0, sizeof(dataBuf));
    memcpy((void *)dataBuf, (void *)g_pwmOutput, sizeof(g_pwmOutput));
    pHdr->size = sizeof(g_pwmOutput);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'p': /* Outputs mixed input settings. */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)dataBuf, 0, sizeof(dataBuf));
    memcpy((void *)dataBuf, (void *)g_mixedInput, sizeof(g_mixedInput));
    pHdr->size = sizeof(g_mixedInput);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 'r': /* Outputs camera attitude data; */
    memcpy((void *)dataBuf, (void *)g_qIMU, sizeof(g_qIMU));
    pHdr->size = sizeof(g_qIMU);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  case 's': /* Outputs PID settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)dataBuf, 0, sizeof(dataBuf));
    memcpy((void *)dataBuf, (void *)g_pidSettings, sizeof(g_pidSettings));
    pHdr->size = sizeof(g_pidSettings);
    pHdr->data = dataBuf;
    pHdr->crc  = telemetryGetCRC32Checksum(pHdr);
    telemetrySendSerialData(pHdr);
    break;
  default:;
  }
}

/**
 * @brief  Reads chunks of data from generic serial interface driver.
 * @return none.
 */
void telemetryReadSerialData(void) {
  chSysLock();
  /* The following function must be called from within a system lock zone. */
  size_t bytesAvailable = chnBytesAvailable(g_chnp);
  chSysUnlock();
  if (bytesRequired) { /* Continue with previous command. */
    if (bytesAvailable >= bytesRequired) {
      if (dataHdr.size % sizeof(uint32_t)) {
        /* Clean data buffer for zero-padded crc32 checksum calculation. */
        memset((void *)dataBuf, 0, sizeof(dataBuf));
      }
      chnRead(g_chnp, dataBuf, dataHdr.size);
      chnRead(g_chnp, (uint8_t *)&dataHdr.crc, sizeof(dataHdr.crc));
      dataHdr.data = dataBuf;
      bytesRequired = 0;
      telemetryProcessCommand(&dataHdr);
      bytesAvailable -= dataHdr.size + sizeof(dataHdr.crc);
      if (bytesAvailable) {
        telemetryReadSerialData();
      }
    }
  } else { /* Read next command from the queue. */
    if (bytesAvailable >= (sizeof(dataHdr.cmd_id) + sizeof(dataHdr.size))) {
      chnRead(g_chnp, (uint8_t *)&dataHdr, sizeof(dataHdr.cmd_id) + sizeof(dataHdr.size));
      bytesAvailable -= sizeof(dataHdr.cmd_id) + sizeof(dataHdr.size);
      if (dataHdr.size) {
        if (bytesAvailable >= (dataHdr.size + sizeof(dataHdr.crc))) {
          if (dataHdr.size % sizeof(uint32_t)) {
            /* Clean data buffer for zero-padded crc32 checksum calculation. */
            memset((void *)dataBuf, 0, sizeof(dataBuf));
          }
          chnRead(g_chnp, dataBuf, dataHdr.size);
          chnRead(g_chnp, (uint8_t *)&dataHdr.crc, sizeof(dataHdr.crc));
          dataHdr.data = dataBuf;
          telemetryProcessCommand(&dataHdr);
          bytesAvailable -= dataHdr.size + sizeof(dataHdr.crc);
          if (bytesAvailable) {
            telemetryReadSerialData();
          }
        } else {
          bytesRequired = dataHdr.size + sizeof(dataHdr.crc);
        }
      } else {
        telemetryProcessCommand(&dataHdr);
        if (bytesAvailable) {
          telemetryReadSerialData();
        }
      }
    }
  }
}
