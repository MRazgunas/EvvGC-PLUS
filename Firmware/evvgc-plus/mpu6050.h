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

#ifndef _MPU6050_H_
#define _MPU6050_H_

#define IMU1_CALIBRATE_GYRO         0x00000008
#define IMU1_CALIBRATE_ACCEL        0x00000010
#define IMU1_CALIBRATION_MASK       0x00000018
#define IMU2_CALIBRATE_GYRO         0x00000020
#define IMU2_CALIBRATE_ACCEL        0x00000040
#define IMU2_CALIBRATION_MASK       0x00000060
#define IMU_CALIBRATION_MASK        0x00000078

#define IMU_AXIS_DIR_POS          0x08
#define IMU_AXIS_ID_MASK          0x07

#define IMU1_AXIS_DIR_POS         0x08
#define IMU1_AXIS_ID_MASK         0x07
#define IMU1_CONF_MASK            0x0F

#define IMU2_AXIS_DIR_POS         0x80
#define IMU2_AXIS_ID_MASK         0x70
#define IMU2_CONF_MASK            0xF0


typedef struct tagIMUStruct {
  float accelData[3];     /* Accelerometer data.             */
  float gyroData[3];      /* Gyroscope data.                 */
  float accelBias[3];     /* Accelerometer bias.             */
  float gyroBias[3];      /* Gyroscope bias.                 */
  float accelFiltered[3]; /* Filtered accelerometer data.    */
  float v2Filtered[3];    /* Filtered direction of gravity.  */
  float qIMU[4];          /* Attitude quaternion of the IMU. */
  float rpy[3];           /* Attitude in Euler angles.       */
  uint32_t clbrCounter;   /* Calibration counter             */
  uint8_t axes_conf[3];   /* Configuration of IMU axes.      */
  uint8_t addr;           /* I2C address of the chip.        */
} __attribute__((packed)) IMUStruct, *PIMUStruct;

/* IMU data structure. */
extern IMUStruct g_IMU1;
/* Packed sensor settings. */
extern uint8_t g_sensorSettings[3];

#ifdef __cplusplus
extern "C" {
#endif
  void imuStructureInit(PIMUStruct pIMU, uint8_t fAddrHigh);
  void imuCalibrationSet(uint8_t flags);
  uint8_t imuCalibrate(PIMUStruct pIMU, uint8_t fCalibrateAcc);
  uint8_t mpu6050Init(uint8_t addr);
  uint8_t mpu6050GetNewData(PIMUStruct pIMU);
  void accelBiasUpdate(void);
  void gyroBiasUpdate(void);
  void sensorSettingsUpdate(const uint8_t *pNewSettings);
#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */
