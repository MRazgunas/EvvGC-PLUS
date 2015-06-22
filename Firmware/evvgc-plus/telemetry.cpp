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

#include "mpu6050.h"
#include "pwmio.h"
#include "eeprom.h"
#include "misc.h"
#include "attitude.h"
#include "usbcfg.h"
#include "telemetry.h"
#include "parameters.h"

extern Parameters g;

/* C libraries: */
#include <string.h>

/* Predefined telemetry responses. */
#define TELEMETRY_RESP_OK         "_OK_"
#define TELEMETRY_RESP_FAIL       "FAIL"

/* Telemetry start-of-frame signature. */
#define TELEMETRY_MSG_SOF         0xBD
/* Empty message ID. */
#define TELEMETRY_MSG_NOMSG       0x00
/* Telemetry buffer size in bytes.  */
#define TELEMETRY_BUFFER_SIZE     0x80
/* Telemetry message header size in bytes.  */
#define TELEMETRY_MSG_HDR_SIZE    0x04
/* Telemetry message checksum size in bytes.  */
#define TELEMETRY_MSG_CRC_SIZE    0x04
/* Telemetry message header + crc size in bytes.  */
#define TELEMETRY_MSG_SVC_SIZE    ( TELEMETRY_MSG_HDR_SIZE + TELEMETRY_MSG_CRC_SIZE )

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
/* Main thread termination flag. */
extern bool_t g_runMain;
/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;
/* Stream data id. */
extern uint8_t g_streamDataID;
/* Data streaming index. */
extern uint8_t g_streamIdx;
/* Console input/output handle. */
BaseChannel *g_chnp = NULL;

/**
 * Local variables
 */
/* Telemetry message. */
static Message msg = {
  TELEMETRY_MSG_SOF,
  TELEMETRY_MSG_NOMSG,
  TELEMETRY_MSG_SVC_SIZE,
  0,
  {0},
  0xFFFFFFFF
};
static uint8_t *msgPos = (uint8_t *)&msg;
static Message debugMsg;

static size_t bytesRequired = TELEMETRY_MSG_HDR_SIZE;

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
  pMsg->size = sizeof(TELEMETRY_RESP_OK) + TELEMETRY_MSG_SVC_SIZE - 1;
  pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
}

/**
 * @brief  Prepares negative telemetry response.
 * @param  pMsg - pointer to telemetry message structure.
 * @return none.
 */
static void telemetryNegativeResponse(const PMessage pMsg) {
  memcpy((void *)pMsg->data, (void *)TELEMETRY_RESP_FAIL, sizeof(TELEMETRY_RESP_FAIL) - 1);
  pMsg->size = sizeof(TELEMETRY_RESP_FAIL) + TELEMETRY_MSG_SVC_SIZE - 1;
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
    if ((pMsg->size - TELEMETRY_MSG_SVC_SIZE) == sizeof(g_sensorSettings)) {
      sensorSettingsUpdate((uint8_t *)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'F': /* Reads new complementary filter settings; */
    if ((pMsg->size - TELEMETRY_MSG_SVC_SIZE) == sizeof(uint16_t) * 2) {
      cfSettingsUpdate((uint16_t *)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'I': /* Reads new mixed input settings; */
    if ((pMsg->size - TELEMETRY_MSG_SVC_SIZE) == sizeof(MixedInputStruct)*3) {
      mixedInputSettingsUpdate((PMixedInputStruct)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'M': /* Reads new input mode settings; */
    if ((pMsg->size - TELEMETRY_MSG_SVC_SIZE) == sizeof(InputModeStruct)*3) {
      inputModeSettingsUpdate((PInputModeStruct)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'O': /* Reads new output settings; */
    if ((pMsg->size - TELEMETRY_MSG_SVC_SIZE) == sizeof(PWMOutputStruct)*3) {
      pwmOutputSettingsUpdate((PPWMOutputStruct)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'S': /* Reads new PID values; */
    if ((pMsg->size - TELEMETRY_MSG_SVC_SIZE) == sizeof(PIDSettin)*3) {
      pidSettingsUpdate((PPIDSettings)pMsg->data);
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'Z': /* Reads new streaming data id; */
    if ((pMsg->size - TELEMETRY_MSG_SVC_SIZE) == sizeof(uint8_t)) {
      g_streamDataID = pMsg->data[0];
      g_streamIdx    = 0; /* Reset index. */
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'a': /* Outputs accelerometer data; */
    memcpy((void *)pMsg->data, (void *)g_IMU1.accelData, sizeof(g_IMU1.accelData));
    pMsg->size = sizeof(g_IMU1.accelData) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'b': /* Outputs board status; */
    memcpy((void *)pMsg->data, (void *)&g_boardStatus, sizeof(g_boardStatus));
    pMsg->size = sizeof(g_boardStatus) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'c': /* Saves settings to EEPROM; */
    if (/*eepromSaveSettings()*/false) {
      telemetryPositiveResponse(pMsg);
    } else {
      telemetryNegativeResponse(pMsg);
    }
    break;
  case 'd': /* Outputs sensor settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)g_sensorSettings, sizeof(g_sensorSettings));
    pMsg->size = sizeof(g_sensorSettings) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'e': /* Outputs I2C error info structure; */
    memcpy((void *)pMsg->data, (void *)&g_i2cErrorInfo, sizeof(g_i2cErrorInfo));
    pMsg->size = sizeof(g_i2cErrorInfo) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'f': /* Outputs complementary filter settings; */
    uint16_t tmp[2];
    tmp[0] = g.cf_2kp;
    tmp[1] = g.cf_2ki;
    memcpy((void *)pMsg->data, (void *)&tmp, sizeof(tmp));
    pMsg->size = sizeof(tmp) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'g': /* Outputs gyroscope data; */
    memcpy((void *)pMsg->data, (void *)g_IMU1.gyroData, sizeof(g_IMU1.gyroData));
    pMsg->size = sizeof(g_IMU1.gyroData) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'h': /* Outputs motor offset data; */
    memcpy((void *)pMsg->data, (void *)g_motorOffset, sizeof(g_motorOffset));
    pMsg->size = sizeof(g_motorOffset) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'i': /* Outputs input data values; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)g_inputValues, sizeof(g_inputValues));
    pMsg->size = sizeof(g_inputValues) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'm': /* Outputs input mode settings; */
    InputModeStruct tmp1[3];
    tmp1[0].min_angle = g_modeSettings[0].min_angle; tmp1[0].max_angle = g_modeSettings[0].max_angle; tmp1[0].mode_id = g_modeSettings[0].mode_id;
    tmp1[0].offset = g_modeSettings[0].offset; tmp1[0].speed = g_modeSettings[0].speed;
    tmp1[1].min_angle = g_modeSettings[1].min_angle; tmp1[1].max_angle = g_modeSettings[1].max_angle; tmp1[1].mode_id = g_modeSettings[1].mode_id;
    tmp1[1].offset = g_modeSettings[1].offset; tmp1[1].speed = g_modeSettings[1].speed;
    tmp1[2].min_angle = g_modeSettings[2].min_angle; tmp1[2].max_angle = g_modeSettings[2].max_angle; tmp1[2].mode_id = g_modeSettings[2].mode_id;
    tmp1[2].offset = g_modeSettings[2].offset; tmp1[2].speed = g_modeSettings[2].speed;
    memcpy((void *)pMsg->data, (void *)tmp1, sizeof(tmp1));
    pMsg->size = sizeof(tmp1) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'o': /* Outputs PWM output settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    PWMOutputStruct tmp2[3];
    tmp2[0].dt_cmd_id = g_pwmOutput[0].dt_cmd_id; tmp2[0].flags = g_pwmOutput[0].flags; tmp2[0].num_poles = g_pwmOutput[0].num_poles; tmp2[0].power = g_pwmOutput[0].power;
    tmp2[1].dt_cmd_id = g_pwmOutput[1].dt_cmd_id; tmp2[1].flags = g_pwmOutput[1].flags; tmp2[1].num_poles = g_pwmOutput[1].num_poles; tmp2[1].power = g_pwmOutput[1].power;
    tmp2[2].dt_cmd_id = g_pwmOutput[2].dt_cmd_id; tmp2[2].flags = g_pwmOutput[2].flags; tmp2[2].num_poles = g_pwmOutput[2].num_poles; tmp2[2].power = g_pwmOutput[2].power;
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)tmp2, sizeof(tmp2));
    pMsg->size = sizeof(tmp2) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'p': /* Outputs mixed input settings. */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    MixedInputStruct tmp3[3];
    tmp3[0].channel_id = g_mixedInput[0].channel_id; tmp3[0].max_val = g_mixedInput[0].max_val; tmp3[0].mid_val = g_mixedInput[0].mid_val; tmp3[0].min_val = g_mixedInput[0].min_val;
    tmp3[1].channel_id = g_mixedInput[1].channel_id; tmp3[1].max_val = g_mixedInput[1].max_val; tmp3[1].mid_val = g_mixedInput[1].mid_val; tmp3[1].min_val = g_mixedInput[1].min_val;
    tmp3[2].channel_id = g_mixedInput[2].channel_id; tmp3[2].max_val = g_mixedInput[2].max_val; tmp3[2].mid_val = g_mixedInput[2].mid_val; tmp3[2].min_val = g_mixedInput[2].min_val;
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)tmp3, sizeof(tmp3));
    pMsg->size = sizeof(tmp3) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 'r': /* Outputs camera attitude data; */
    memcpy((void *)pMsg->data, (void *)g_IMU1.qIMU, sizeof(g_IMU1.qIMU));
    pMsg->size = sizeof(g_IMU1.qIMU) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case 's': /* Outputs PID settings; */
    /* Clean data buffer for zero-padded crc32 checksum calculation. */
    PIDSettin tmp4[3];
    tmp4[0].D = g_pidSettings[0].D; tmp4[0].I = g_pidSettings[0].I; tmp4[0].P = g_pidSettings[0].P;
    tmp4[1].D = g_pidSettings[1].D; tmp4[1].I = g_pidSettings[1].I; tmp4[1].P = g_pidSettings[1].P;
    tmp4[2].D = g_pidSettings[2].D; tmp4[2].I = g_pidSettings[2].I; tmp4[2].P = g_pidSettings[2].P;
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)tmp4, sizeof(tmp4));
    pMsg->size = sizeof(tmp4) + TELEMETRY_MSG_SVC_SIZE;
    pMsg->crc  = telemetryGetCRC32Checksum(pMsg);
    break;
  case '[': /* Calibrate gyroscope. */
    imuCalibrationSet(IMU1_CALIBRATE_GYRO);
    telemetryPositiveResponse(pMsg);
    break;
  case ']': /* Calibrate accelerometer. */
    imuCalibrationSet(IMU1_CALIBRATE_ACCEL);
    telemetryPositiveResponse(pMsg);
    break;
  case 'l': /* Outputs last debug message. */
    memset((void *)pMsg->data, 0, TELEMETRY_BUFFER_SIZE);
    memcpy((void *)pMsg->data, (void *)debugMsg.data, debugMsg.size);
    pMsg->size = debugMsg.size;
    pMsg->crc  = telemetryGetCRC32Checksum(&debugMsg);
    debugMsg.size = 0;
    break;
  case 'X': /* Hard reset the board. */
    telemetryPositiveResponse(pMsg);
    g_runMain = FALSE;
    break;
  default: /* Unknown command. */
    telemetryNegativeResponse(pMsg);
  }
  pMsg->sof = TELEMETRY_MSG_SOF;
  pMsg->res = 0;
  telemetrySendSerialData(pMsg);
}

#define IS_MSG_VALID() \
  ((msg.sof == TELEMETRY_MSG_SOF) &&\
  (msg.size >= TELEMETRY_MSG_SVC_SIZE) &&\
  (msg.size <= TELEMETRY_BUFFER_SIZE))

/**
 * @brief: Try to recover from bad input data stream
 * @note: Author of this expects that most of the time, this will be caused
 *        by some junk stuff coming from OS (e.g. tty settings). We'll just
 *        sync to next SOF and throw away any after packet. This means the next
 *        (few) packet(s) may be dropped - still better than no comm...
 */
static void telemetryReadSerialDataResync(uint8_t len) {
  uint8_t i;

  while (len) {
    for (i = 1; i < len; i++) {
      if (((uint8_t *)&msg)[i] == TELEMETRY_MSG_SOF) {
        break;
      }
    }

    if (len - i > 0) {
      memmove(&msg, &((uint8_t *)&msg)[i], len - i);
    }
    len -= i;
    msgPos = (uint8_t *)&msg + len;

    if (len < TELEMETRY_MSG_HDR_SIZE) {
      /* ...wait for header to get completed */
      bytesRequired = TELEMETRY_MSG_HDR_SIZE - len;
      break;
    } else {
      if (IS_MSG_VALID()) {
        if (msg.size <= len) {
          /* This may throw away some data at the tail of buffer...*/
          bytesRequired = 0;
        } else {
          bytesRequired = msg.size - len;
        }
        break;
      }
    }
  }
}

/**
 * @brief  Process messages received from generic serial interface driver.
 * @return none.
 */
void telemetryReadSerialData(void) {
  chSysLock();
  /* The following function must be called from within a system lock zone. */
  size_t bytesAvailable = chnBytesAvailable(g_chnp);
  chSysUnlock();

  while (bytesAvailable) {
    if (bytesAvailable >= bytesRequired) {
      if (bytesRequired > 0) {
        palTogglePad(GPIOA, GPIOA_LED_B);
        chnRead(g_chnp, msgPos, bytesRequired);
        msgPos += bytesRequired;
        bytesAvailable -= bytesRequired;
        bytesRequired = 0;
      }
    } else {
      chnRead(g_chnp, msgPos, bytesAvailable);
      msgPos += bytesAvailable;
      bytesRequired -= bytesAvailable;
      break;
    }

    size_t curReadLen = msgPos - (uint8_t *)&msg;
    if (!IS_MSG_VALID()) {
      telemetryReadSerialDataResync(curReadLen);
    } else if (curReadLen == TELEMETRY_MSG_HDR_SIZE) {
      bytesRequired = msg.size - TELEMETRY_MSG_HDR_SIZE;
    } else if (bytesRequired == 0) {
      /* Whole packet is read, check and process it... */
      /* Move CRC */
      memmove(&msg.crc, (uint8_t *)&msg + msg.size - TELEMETRY_MSG_CRC_SIZE,
        TELEMETRY_MSG_CRC_SIZE);
      /* Zero-out unused data for crc32 calculation. */
      memset(&msg.data[msg.size - TELEMETRY_MSG_SVC_SIZE], 0,
        TELEMETRY_BUFFER_SIZE - msg.size + TELEMETRY_MSG_SVC_SIZE);

      if (msg.crc == telemetryGetCRC32Checksum(&msg)) {
        telemetryProcessCommand(&msg);
      } else {
        uint8_t i;
        for (i =0; i < 50; i++) {
          palTogglePad(GPIOA, GPIOA_LED_B);
          chThdSleepMilliseconds(US2ST(10500));
        }
      }

      /* Read another packet...*/
      bytesRequired = TELEMETRY_MSG_HDR_SIZE;
      msgPos = (uint8_t*)&msg;
    }
  }
}

/**
 * @brief
 */
void telemetryWriteStream(const float *pData, size_t size) {
  msg.sof = TELEMETRY_MSG_SOF;
  msg.msg_id = 'z';
  msg.size = size + TELEMETRY_MSG_SVC_SIZE;
  msg.res = 0;
  memcpy((void *)msg.data, (void *)pData, size);
  msg.crc = telemetryGetCRC32Checksum(&msg);
  telemetrySendSerialData(&msg);
}

/**
 * @brief
 */
void debugLog(const char *str) {
  uint8_t l = strlen(str) + 1;
  if (l > sizeof(debugMsg.data)) {
    l = sizeof(debugMsg.data);
  }

  memcpy(debugMsg.data, str, l);
  debugMsg.size = l + TELEMETRY_MSG_SVC_SIZE;
}
