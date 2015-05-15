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
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include "ch.h"
#include "hal.h"

#include "misc.h"
#include "telemetry.h"
#include "mpu6050.h"

/* C libraries: */
#include <string.h>

#define MPU6050_RX_BUF_SIZE   0x0E
#define MPU6050_TX_BUF_SIZE   0x05

/* I2C read transaction time-out in milliseconds. */
#define MPU6050_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6050_WRITE_TIMEOUT_MS  0x01

#define CALIBRATION_COUNTER_MAX   5000

/**
 * Global variables
 */
/* Default packed sensor settings.
 * Structure of the sensor settings is:
 * D2|2I2|1I2|0I2|D1|2I1|1I1|0I1
 * where Dx  - axis direction of the sensor x;
 *       nIx - n-th bit of axis ID of the sensor x.
 */
uint8_t g_sensorSettings[3] = {
  0x00 |
  IMU1_AXIS_DIR_POS |
  IMU2_AXIS_DIR_POS, /* Pitch (X) */
  0x11,              /* Roll  (Y) */
  0x22,              /* Yaw   (Z) */
};

/* IMU data structure. */
IMUStruct g_IMU1;

/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;
extern uint32_t g_boardStatus;

/**
 * Local variables
 */
/* Data buffers */
static uint8_t mpu6050RXData[MPU6050_RX_BUF_SIZE];
static uint8_t mpu6050TXData[MPU6050_TX_BUF_SIZE];

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fAddrLow - IMU address pin A0 is pulled low flag.
 */
void imuStructureInit(PIMUStruct pIMU, uint8_t fAddrHigh) {
  uint8_t i;
  /* Initialize to zero. */
  memset((void *)pIMU, 0, sizeof(IMUStruct));
  /* Initialize to unity quaternion. */
  pIMU->qIMU[0] = 1.0f;

  if (fAddrHigh) {
    pIMU->addr = MPU6050_I2C_ADDR_A0_HIGH;
    for (i = 0; i < 3; i++) {
      pIMU->axes_conf[i] = g_sensorSettings[i] >> 4;
    }
  } else {
    pIMU->addr = MPU6050_I2C_ADDR_A0_LOW;
    for (i = 0; i < 3; i++) {
      pIMU->axes_conf[i] = g_sensorSettings[i] & IMU1_CONF_MASK;
    }
  }
}

/**
 * @brief
 */
void imuCalibrationSet(uint8_t flags) {
  g_boardStatus |= flags & IMU_CALIBRATION_MASK;
}

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fCalibrateAcc - accelerometer calibration flag.
 * @return 0 - if calibration is not finished;
 *         1 - if calibration is finished.
 */
uint8_t imuCalibrate(PIMUStruct pIMU, uint8_t fCalibrateAcc) {
  if (fCalibrateAcc) {
    if (pIMU->clbrCounter == 0) {
      /* Reset accelerometer bias. */
      pIMU->accelBias[0] = pIMU->accelData[0];
      pIMU->accelBias[1] = pIMU->accelData[0];
      pIMU->accelBias[2] = pIMU->accelData[0] + GRAV;
      pIMU->clbrCounter++;
      return 0;
    } else if (pIMU->clbrCounter < CALIBRATION_COUNTER_MAX) {
      /* Accumulate accelerometer bias. */
      pIMU->accelBias[0] += pIMU->accelData[0];
      pIMU->accelBias[1] += pIMU->accelData[1];
      pIMU->accelBias[2] += pIMU->accelData[2] + GRAV;
      pIMU->clbrCounter++;
      return 0;
    } else {
      /* Update accelerometer bias. */
      pIMU->accelBias[0] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->accelBias[1] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->accelBias[2] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->clbrCounter = 0;
    }
  } else {
    if (pIMU->clbrCounter == 0) {
      /* Reset gyroscope bias. */
      pIMU->gyroBias[0] = pIMU->gyroData[0];
      pIMU->gyroBias[1] = pIMU->gyroData[0];
      pIMU->gyroBias[2] = pIMU->gyroData[0];
      pIMU->clbrCounter++;
      return 0;
    } else if (pIMU->clbrCounter < CALIBRATION_COUNTER_MAX) {
      /* Accumulate gyroscope bias. */
      pIMU->gyroBias[0] += pIMU->gyroData[0];
      pIMU->gyroBias[1] += pIMU->gyroData[1];
      pIMU->gyroBias[2] += pIMU->gyroData[2];
      pIMU->clbrCounter++;
      return 0;
    } else {
      /* Update gyroscope bias. */
      pIMU->gyroBias[0] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->gyroBias[1] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->gyroBias[2] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->clbrCounter = 0;
    }
  }
  return 1;
}

/**
 * @brief  Initialization function for the MPU6050 sensor.
 * @param  addr - I2C address of MPU6050 chip.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 */
uint8_t mpu6050Init(uint8_t addr) {
  msg_t status = RDY_OK;

  /* Reset all MPU6050 registers to their default values */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0b11000000;          // Register value;

  i2cAcquireBus(&I2CD2);

  status = i2cMasterTransmitTimeout(&I2CD2, addr, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != RDY_OK) {
    i2cReleaseBus(&I2CD2);
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
      debugLog("E:mpu6050i-tx1");
    }
    return 0;
  }

  /* Wait 100 ms for the MPU6050 to reset */
  chThdSleepMilliseconds(100);

  /* Clear the SLEEP flag, set the clock and start measuring. */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0b00000011;          // Register value CLKSEL = PLL_Z;

  status = i2cMasterTransmitTimeout(&I2CD2, addr, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != RDY_OK) {
    i2cReleaseBus(&I2CD2);
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
      debugLog("E:mpu6050i-rst");
    }
    return 0;
  }

  /* Configure the MPU6050 sensor        */
  /* NOTE:                               */
  /* - SLEEP flag must be cleared before */
  /*   configuring the sensor.           */
  mpu6050TXData[0] = MPU6050_SMPLRT_DIV;  // Start register address;
  mpu6050TXData[1] = 11;                  // SMPLRT_DIV register value (8000 / (11 + 1) = 666 Hz);
  mpu6050TXData[2] = 0b00000000;          // CONFIG register value DLPF_CFG = 0 (256-260 Hz);
  mpu6050TXData[3] = 0b00010000;          // GYRO_CONFIG register value FS_SEL = +-1000 deg/s;
  mpu6050TXData[4] = 0b00010000;          // ACCEL_CONFIG register value AFS_SEL = +-8G;

  status = i2cMasterTransmitTimeout(&I2CD2, addr, mpu6050TXData, 5,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  i2cReleaseBus(&I2CD2);

  if (status != RDY_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
      debugLog("E:mpu6050i-cfg");
    }
    return 0;
  }

  return 1;
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t mpu6050GetNewData(PIMUStruct pIMU) {
  msg_t status = RDY_OK;
  uint8_t id;

  /* Set the start register address for bulk data transfer. */
  mpu6050TXData[0] = MPU6050_ACCEL_XOUT_H;
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmitTimeout(&I2CD2, pIMU->addr, mpu6050TXData, 1,
    mpu6050RXData, 14, MS2ST(MPU6050_READ_TIMEOUT_MS));
  i2cReleaseBus(&I2CD2);

  if (status != RDY_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
      debugLog("E:mpu6050gnd");
    }
    return 0;
  }

  id = pIMU->axes_conf[0] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[0] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[id] = ((int16_t)((mpu6050RXData[0]<<8) | mpu6050RXData[1]))*MPU6050_ACCEL_SCALE; /* Accel X */
    pIMU->gyroData[id]  = ((int16_t)((mpu6050RXData[8]<<8) | mpu6050RXData[9]))*MPU6050_GYRO_SCALE;  /* Gyro X  */
  } else {
    pIMU->accelData[id] = (-1 - (int16_t)((mpu6050RXData[0]<<8) | mpu6050RXData[1]))*MPU6050_ACCEL_SCALE; /* Accel X */
    pIMU->gyroData[id]  = (-1 - (int16_t)((mpu6050RXData[8]<<8) | mpu6050RXData[9]))*MPU6050_GYRO_SCALE;  /* Gyro X  */
  }

  id = pIMU->axes_conf[1] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[1] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[id] = ((int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]))*MPU6050_ACCEL_SCALE; /* Accel Y */
    pIMU->gyroData[id]  = ((int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]))*MPU6050_GYRO_SCALE;  /* Gyro Y  */
  } else {
    pIMU->accelData[id] = (-1 - (int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]))*MPU6050_ACCEL_SCALE; /* Accel Y */
    pIMU->gyroData[id]  = (-1 - (int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]))*MPU6050_GYRO_SCALE;  /* Gyro Y  */
  }

  id = pIMU->axes_conf[2] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[2] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[id] = ((int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]))*MPU6050_ACCEL_SCALE; /* Accel Z */
    pIMU->gyroData[id]  = ((int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]))*MPU6050_GYRO_SCALE;  /* Gyro Z  */
  } else {
    pIMU->accelData[id] = (-1 - (int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]))*MPU6050_ACCEL_SCALE; /* Accel Z */
    pIMU->gyroData[id]  = (-1 - (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]))*MPU6050_GYRO_SCALE;  /* Gyro Z  */
  }

  return 1;
}

/**
 * @brief
 */
void sensorSettingsUpdate(const uint8_t *pNewSettings) {
  uint8_t i;
  memcpy((void *)g_sensorSettings, (void *)pNewSettings, sizeof(g_sensorSettings));
  for (i = 0; i < 3; i++) {
    g_IMU1.axes_conf[i] = g_sensorSettings[i] & IMU1_CONF_MASK;
  }
}

/**
 * @brief
 */
void accelBiasUpdate(PIMUStruct pIMU, const float *pNewSettings) {
  memcpy((void *)pIMU->accelBias, (void *)pNewSettings, sizeof(pIMU->accelBias));
}

/**
 * @brief
 */
void gyroBiasUpdate(PIMUStruct pIMU, const float *pNewSettings) {
	memcpy((void *)pIMU->gyroBias, (void *)pNewSettings, sizeof(pIMU->gyroBias));
}
