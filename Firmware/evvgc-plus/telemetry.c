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

/* Predefined telemetry responses. */
#define TELEMETRY_RESP_OK         "_OK_"
#define TELEMETRY_RESP_FAIL       "FAIL"

/* Telemetry start-of-frame signature. */
#define TELEMETRY_MSG_SOF         0xBD
/* Empty message ID. */
#define TELEMETRY_MSG_NOMSG       0x00
/* Telemetry buffer size in bytes.  */
#define TELEMETRY_BUFFER_SIZE     0x20
/* Telemetry message header size in bytes.  */
#define TELEMETRY_MSG_HDR_SIZE    0x04
/* Telemetry message checksum size in bytes.  */
#define TELEMETRY_MSG_CRC_SIZE    0x04
/* Telemetry message header + crc size in bytes.  */
#define TELEMETRY_MSG_SIZE        ( TELEMETRY_MSG_HDR_SIZE + TELEMETRY_MSG_CRC_SIZE )

/* Telemetry message structure. */
typedef struct tagMessage {
  uint8_t sof;      /* Start of frame signature. */
  uint8_t msg_id;   /* Telemetry message ID. */
  uint8_t size;     /* Size of whole telemetry message including header and additional data. */
  uint8_t res;      /* Reserved. Set to 0. */
  uint8_t data[TELEMETRY_BUFFER_SIZE];
  uint32_t crc;
} __attribute__((packed)) Message, *PMessage;

/**
 * Global variables
 */
/* Board status variable. */
extern uint32_t g_boardStatus;
/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;
/* Console input/output handle. */
BaseChannel *g_chnp;

/**
 * Local variables
 */
/* Telemetry message. */
static Message msg = {
  TELEMETRY_MSG_SOF,
  TELEMETRY_MSG_NOMSG,
  TELEMETRY_MSG_SIZE,
  0,
  {0},
  0xFFFFFFFF
};

static size_t bytesRequired = 0;

/**
 * @brief  Calculates CRC32 checksum of received data buffer.
 * @param  pMsg - pointer to telemetry message structure.
 * @return CRC32 checksum of received zero-padded data buffer.
 */
static uint32_t telemetryGetCRC32Checksum(const PMessage pMsg) {
  uint32_t crc_length = (pMsg->size - TELEMETRY_MSG_CRC_SIZE) / sizeof(uint32_t);
  if ((pMsg->size - TELEMETRY_MSG_CRC_SIZE) % sizeof(uint32_t)) {
    crc_length++;
  }
  return crcCRC32((const uint32_t *)pMsg, crc_length);
}

/**
 * @brief  Sends data to selected serial port.
 * @param  pMsg - pointer to telemetry message structure to be sent.
 * @return none.
 */
static void telemetrySendSerialData(const PMessage pMsg) {
  /* Sends message header and actual data if any. */
  chnWrite(g_chnp, (const uint8_t *)pMsg, pMsg->size - TELEMETRY_MSG_CRC_SIZE);
  /* Sends cyclic redundancy checksum. */
  chnWrite(g_chnp, (const uint8_t *)&pMsg->crc, TELEMETRY_MSG_CRC_SIZE);
}

/**
 * @brief  Prepares positive telemetry response.
 * @param  pMsg - pointer to telemetry message structure.
 * @return none.
 */
static void telemetryPositiveResponse(const PMessage pMsg) {
  memcpy((void *)pMsg->data, (void *)TELEMETRY_RESP_OK, sizeof(TELEMETRY_RESP_OK) - 1);
  pMsg->size = sizeof(TELEMETRY_RESP_OK) + TELEMETRY_MSG_SIZE - 1;
  pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
}

/**
 * @brief  Prepares negative telemetry response.
 * @param  pMsg - pointer to telemetry message structure.
 * @return none.
 */
static void telemetryNegativeResponse(const PMessage pMsg) {
  memcpy((void *)pMsg->data, (void *)TELEMETRY_RESP_FAIL, sizeof(TELEMETRY_RESP_FAIL) - 1);
  pMsg->size = sizeof(TELEMETRY_RESP_FAIL) + TELEMETRY_MSG_SIZE - 1;
  pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
}

/**
 * @brief  Command processor.
 * @param  pMsg - pointer to telemetry message structure to be processed.
 * @return none.
 */
static void telemetryProcessCommand(const PMessage pMsg) {
  switch (pMsg->msg_id) {
  case 'D': /* Reads new sensor settings; */
    if ((pMsg->size - TELEMETRY_MSG_SIZE) == sizeof(g_sensorSettings)) {
      sensorSettingsUpdate((uint8_t *)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'I': /* Reads new mixed input settings; */
    if ((pMsg->size - TELEMETRY_MSG_SIZE) == sizeof(g_mixedInput)) {
      mixedInputSettingsUpdate((PMixedInputStruct)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'M': /* Reads new input mode settings; */
    if ((pMsg->size - TELEMETRY_MSG_SIZE) == sizeof(g_modeSettings)) {
      inputModeSettingsUpdate((PInputModeStruct)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'O': /* Reads new output settings; */
    if ((pMsg->size - TELEMETRY_MSG_SIZE) == sizeof(g_pwmOutput)) {
      pwmOutputSettingsUpdate((PPWMOutputStruct)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'S': /* Reads new PID values; */
    if ((pMsg->size - TELEMETRY_MSG_SIZE) == sizeof(g_pidSettings)) {
      pidSettingsUpdate((PPIDSettings)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'a': /* Outputs accelerometer data; */
    memcpy((void *)pMsg->data, (void *)g_IMU1.accelData, sizeof(g_IMU1.accelData));
    pMsg->size = sizeof(g_IMU1.accelData) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'b': /* Outputs board status; */
    memcpy((void *)pMsg->data, (void *)&g_boardStatus, sizeof(g_boardStatus));
    pMsg->size = sizeof(g_boardStatus) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'c': /* Saves settings to EEPROM; */
    if (eepromSaveSettings()) {
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'd': /* Outputs sensor settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)g_sensorSettings, sizeof(g_sensorSettings));
    pMsg->size = sizeof(g_sensorSettings) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'e': /* Outputs I2C error info structure; */
    memcpy((void *)pMsg->data, (void *)&g_i2cErrorInfo, sizeof(g_i2cErrorInfo));
    pMsg->size = sizeof(g_i2cErrorInfo) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'g': /* Outputs gyroscope data; */
    memcpy((void *)pMsg->data, (void *)g_IMU1.gyroData, sizeof(g_IMU1.gyroData));
    pMsg->size = sizeof(g_IMU1.gyroData) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'h': /* Outputs motor offset data; */
    memcpy((void *)pMsg->data, (void *)g_motorOffset, sizeof(g_motorOffset));
    pMsg->size = sizeof(g_motorOffset) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'i': /* Outputs input data values; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)g_inputValues, sizeof(g_inputValues));
    pMsg->size = sizeof(g_inputValues) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'm': /* Outputs input mode settings; */
    memcpy((void *)pMsg->data, (void *)g_modeSettings, sizeof(g_modeSettings));
    pMsg->size = sizeof(g_modeSettings) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'o': /* Outputs PWM output settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)g_pwmOutput, sizeof(g_pwmOutput));
    pMsg->size = sizeof(g_pwmOutput) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'p': /* Outputs mixed input settings. */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)g_mixedInput, sizeof(g_mixedInput));
    pMsg->size = sizeof(g_mixedInput) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'r': /* Outputs camera attitude data; */
    memcpy((void *)pMsg->data, (void *)g_IMU1.qIMU, sizeof(g_IMU1.qIMU));
    pMsg->size = sizeof(g_IMU1.qIMU) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 's': /* Outputs PID settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)g_pidSettings, sizeof(g_pidSettings));
    pMsg->size = sizeof(g_pidSettings) + TELEMETRY_MSG_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case '[': /* Calibrate gyroscope. */
    imuCalibrationStart(&g_IMU1, IMU_CALIBRATE_GYRO);
    telemetryPositiveResponse(pMsg);
    break;
  case ']': /* Calibrate accelerometer. */
    imuCalibrationStart(&g_IMU1, IMU_CALIBRATE_ACCEL);
    telemetryPositiveResponse(pMsg);
    break;
  case '{': /* Calibrate gyroscope. */
    imuCalibrationStart(&g_IMU2, IMU_CALIBRATE_GYRO);
    telemetryPositiveResponse(pMsg);
    break;
  case '}': /* Calibrate accelerometer. */
    imuCalibrationStart(&g_IMU2, IMU_CALIBRATE_ACCEL);
    telemetryPositiveResponse(pMsg);
    break;
  default: /* Unknown command. */
    telemetryNegativeResponse(pMsg);
  }
  pMsg->sof = TELEMETRY_MSG_SOF;
  pMsg->res = 0;
  telemetrySendSerialData(pMsg);
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
      if ((msg.size - TELEMETRY_MSG_SIZE) % sizeof(uint32_t)) {
        /* Clean data buffer for zero-padded crc32 checksum calculation. */
        memset((void *)msg.data, 0, TELEMETRY_BUFFER_SIZE);
      }
      if (msg.size - TELEMETRY_MSG_SIZE) {
        chnRead(g_chnp, (uint8_t *)msg.data, msg.size - TELEMETRY_MSG_SIZE);
      }
      chnRead(g_chnp, (uint8_t *)&msg.crc, TELEMETRY_MSG_CRC_SIZE);
      if (msg.crc == telemetryGetCRC32Checksum(&msg)) {
        telemetryProcessCommand(&msg);
      }
      bytesRequired = 0;
      bytesAvailable -= msg.size - TELEMETRY_MSG_HDR_SIZE;
      if (bytesAvailable) {
        telemetryReadSerialData();
      }
    }
  } else if (bytesAvailable >= TELEMETRY_MSG_HDR_SIZE) {
    /* Read next command from the queue. */
    chnRead(g_chnp, (uint8_t *)&msg, TELEMETRY_MSG_HDR_SIZE);
    bytesAvailable -= TELEMETRY_MSG_HDR_SIZE;
    /* Check if message header is not corrupted. */
    if ((msg.sof == TELEMETRY_MSG_SOF) &&
        (msg.size >= TELEMETRY_MSG_SIZE) &&
        (msg.size <= TELEMETRY_BUFFER_SIZE)) {
      /* Read message data. */
      if (bytesAvailable >= (size_t)(msg.size - TELEMETRY_MSG_HDR_SIZE)) {
        if ((msg.size - TELEMETRY_MSG_SIZE) % sizeof(uint32_t)) {
          /* Clean data buffer for zero-padded crc32 checksum calculation. */
          memset((void *)msg.data, 0, TELEMETRY_BUFFER_SIZE);
        }
        if (msg.size - TELEMETRY_MSG_SIZE) {
          chnRead(g_chnp, (uint8_t *)msg.data, msg.size - TELEMETRY_MSG_SIZE);
        }
        chnRead(g_chnp, (uint8_t *)&msg.crc, TELEMETRY_MSG_CRC_SIZE);
        if (msg.crc == telemetryGetCRC32Checksum(&msg)) {
          telemetryProcessCommand(&msg);
        }
        bytesAvailable -= msg.size - TELEMETRY_MSG_HDR_SIZE;
        if (bytesAvailable) {
          telemetryReadSerialData();
        }
      } else {
        bytesRequired = msg.size - TELEMETRY_MSG_HDR_SIZE;
      }
    } else if (bytesAvailable) {
      telemetryReadSerialData();
    }
  }
}
