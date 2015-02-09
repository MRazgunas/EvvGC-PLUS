/*
    EvvGC-PLUS - Copyright (C) 2013-2014 Sarunas Vaitekonis

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

#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_I2C_ADDR_A0_LOW     0x68
#define MPU6050_I2C_ADDR_A0_HIGH    0x69

/* MPU6050 useful registers */
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_PWR_MGMT_1          0x6B

/* Sensor scales */
//#define MPU6050_GYRO_SCALE              (1.0f / 131.0f) //  250 deg/s
#define MPU6050_GYRO_SCALE              (1.0f /  65.5f) //  500 deg/s
//#define MPU6050_GYRO_SCALE              (1.0f /  32.8f) // 1000 deg/s
//#define MPU6050_GYRO_SCALE              (1.0f /  16.4f) // 2000 deg/s

#define GRAV                            9.81
//#define MPU6050_ACCEL_SCALE             (GRAV / 16384.0f) //  2G
#define MPU6050_ACCEL_SCALE             (GRAV /  8192.0f) //  4G
//#define MPU6050_ACCEL_SCALE             (GRAV /  4096.0f) //  8G
//#define MPU6050_ACCEL_SCALE             (GRAV /  2048.0f) // 16G

/* Scaled accelerometer data. */
extern float g_accelData[3];
/* Scaled gyroscope data. */
extern float g_gyroData[3];

#ifdef __cplusplus
extern "C" {
#endif
  inline uint8_t mpu6050Init(void);
  inline uint8_t mpu6050GetNewData(void);
#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H_ */
