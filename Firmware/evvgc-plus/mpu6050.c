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

/* C libraries: */
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "mpu6050.h"
#include "misc.h"

#define MPU6050_RX_BUF_SIZE   0x0E
#define MPU6050_TX_BUF_SIZE   0x05

/* I2C read transaction time-out in milliseconds. */
#define MPU6050_READ_TIMEOUT_MS   0x02
/* I2C write transaction time-out in milliseconds. */
#define MPU6050_WRITE_TIMEOUT_MS  0x02

/**
 * Global variables
 */
/* Default sensor settings. */
SensorStruct g_sensorSettings[3] = {
/* ID, DIR */
  {0,  1}, /* Pitch (X) */
  {1, -1}, /* Roll  (Y) */
  {2, -1}  /* Yaw   (Z) */
};
/* Scaled accelerometer data. */
float g_accelData[3] = {0.0f};
/* Scaled gyroscope data. */
float g_gyroData[3] = {0.0f};
/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;

/**
 * Local variables
 */
/* Data buffers */
static uint8_t mpu6050RXData[MPU6050_RX_BUF_SIZE];
static uint8_t mpu6050TXData[MPU6050_TX_BUF_SIZE];

/**
 * Initialization function for the MPU6050 sensor.
 */
uint8_t mpu6050Init(void) {
  msg_t status = RDY_OK;

  /* Reset all MPU6050 registers to their default values */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0b11000000;          // Register value;

  i2cAcquireBus(&I2CD2);

  status = i2cMasterTransmitTimeout(&I2CD2, MPU6050_I2C_ADDR_A0_LOW, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != RDY_OK) {
    i2cReleaseBus(&I2CD2);
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
    }
    return 0;
  }

  /* Wait 100 ms for the MPU6050 to reset */
  chThdSleepMilliseconds(100);

  /* Clear the SLEEP flag, set the clock and start measuring. */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0b00000011;          // Register value CLKSEL = PLL_Z;

  status = i2cMasterTransmitTimeout(&I2CD2, MPU6050_I2C_ADDR_A0_LOW, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != RDY_OK) {
    i2cReleaseBus(&I2CD2);
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
    }
    return 0;
  }

  /* Configure the MPU6050 sensor        */
  /* NOTE:                               */
  /* - SLEEP flag must be cleared before */
  /*   configuring the sensor.           */
  mpu6050TXData[0] = MPU6050_SMPLRT_DIV;  // Start register address;
  mpu6050TXData[1] = 15;                  // SMPLRT_DIV register value (8000 / (15 + 1) = 500 Hz);
  mpu6050TXData[2] = 0b00000000;          // CONFIG register value DLPF_CFG = 0 (256-260 Hz);
  mpu6050TXData[3] = 0b00010000;          // GYRO_CONFIG register value FS_SEL = +-1000 deg/s;
  mpu6050TXData[4] = 0b00010000;          // ACCEL_CONFIG register value AFS_SEL = +-8G;

  status = i2cMasterTransmitTimeout(&I2CD2, MPU6050_I2C_ADDR_A0_LOW, mpu6050TXData, 5,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  i2cReleaseBus(&I2CD2);

  if (status != RDY_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
    }
    return 0;
  }

  return 1;
}

/**
 *
 */
uint8_t mpu6050GetNewData(void) {
  msg_t status = RDY_OK;
  uint8_t i;

  /* Set the start register address for bulk data transfer. */
  mpu6050TXData[0] = MPU6050_ACCEL_XOUT_H;
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmitTimeout(&I2CD2, MPU6050_I2C_ADDR_A0_LOW, mpu6050TXData, 1,
    mpu6050RXData, 14, MS2ST(MPU6050_READ_TIMEOUT_MS));
  i2cReleaseBus(&I2CD2);

  if (status != RDY_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(&I2CD2);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
    }
    return 0;
  }

  i = g_sensorSettings[0].axis_id;
  if (g_sensorSettings[0].axis_dir > 0) {
    g_accelData[i] = ((int16_t)((mpu6050RXData[0]<<8) | mpu6050RXData[1]))*MPU6050_ACCEL_SCALE; /* Accel X */
    g_gyroData[i]  = ((int16_t)((mpu6050RXData[8]<<8) | mpu6050RXData[9]))*MPU6050_GYRO_SCALE;  /* Gyro X  */
  } else {
    g_accelData[i] = (-1 - (int16_t)((mpu6050RXData[0]<<8) | mpu6050RXData[1]))*MPU6050_ACCEL_SCALE; /* Accel X */
    g_gyroData[i]  = (-1 - (int16_t)((mpu6050RXData[8]<<8) | mpu6050RXData[9]))*MPU6050_GYRO_SCALE;  /* Gyro X  */
  }

  i = g_sensorSettings[1].axis_id;
  if (g_sensorSettings[1].axis_dir > 0) {
    g_accelData[i] = ((int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]))*MPU6050_ACCEL_SCALE; /* Accel Y */
    g_gyroData[i]  = ((int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]))*MPU6050_GYRO_SCALE;  /* Gyro Y  */
  } else {
    g_accelData[i] = (-1 - (int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]))*MPU6050_ACCEL_SCALE; /* Accel Y */
    g_gyroData[i]  = (-1 - (int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]))*MPU6050_GYRO_SCALE;  /* Gyro Y  */
  }

  i = g_sensorSettings[2].axis_id;
  if (g_sensorSettings[2].axis_dir > 0) {
    g_accelData[i] = ((int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]))*MPU6050_ACCEL_SCALE; /* Accel Z */
    g_gyroData[i]  = ((int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]))*MPU6050_GYRO_SCALE;  /* Gyro Z  */
  } else {
    g_accelData[i] = (-1 - (int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]))*MPU6050_ACCEL_SCALE; /* Accel Z */
    g_gyroData[i]  = (-1 - (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]))*MPU6050_GYRO_SCALE;  /* Gyro Z  */
  }

  return 1;
}

/**
 * @brief
 */
void sensorSettingsUpdate(const PSensorStruct pNewSettings) {
  memcpy((void *)&g_sensorSettings, (void *)pNewSettings, sizeof(g_sensorSettings));
}
