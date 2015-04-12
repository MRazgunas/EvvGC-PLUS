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

#include "usbcfg.h"
#include "mpu6050.h"
#include "attitude.h"
#include "pwmio.h"
#include "misc.h"
#include "telemetry.h"
#include "eeprom.h"

/* Telemetry operation time out in milliseconds. */
#define TELEMETRY_SLEEP_MS      20

#define MPU6050_LOW_DETECTED    0x00000001
#define MPU6050_HIGH_DETECTED   0x00000002
#define EEPROM_24C02_DETECTED   0x00000004

uint32_t g_boardStatus = 0;
I2CErrorStruct g_i2cErrorInfo = {0, 0};

/* I2C2 configuration for I2C driver 2 */
static const I2CConfig i2cfg_d2 = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2,
};

/* Virtual serial port over USB. */
SerialUSBDriver SDU1;

/* Binary semaphore indicating that new data is ready to be processed. */
static BinarySemaphore bsemIMU1DataReady;
static BinarySemaphore bsemIMU2DataReady;

/**
 * Red LED blinker thread. Times are in milliseconds.
 */
static WORKING_AREA(waBlinkerThread, 64);
static msg_t BlinkerThread(void *arg) {
  (void)arg;
  while (TRUE) {
    systime_t time;
    if (g_boardStatus & IMU_CALIBRATION_MASK) {
      time = 50;
    } else {
      time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 500;
    }
    palTogglePad(GPIOB, GPIOB_LED_A);
    chThdSleepMilliseconds(time);
  }
  /* This point should never be reached. */
  return 0;
}

/**
 * MPU6050 data polling thread. Times are in milliseconds.
 * This thread requests a new data from MPU6050 every 1.5 ms (@666 Hz).
 */
static WORKING_AREA(waPollMPU6050Thread, 128);
static msg_t PollMPU6050Thread(void *arg) {
  systime_t time;

  (void)arg;
  time = chTimeNow();
  while (TRUE) {
    if (mpu6050GetNewData(&g_IMU1)) {
      chBSemSignal(&bsemIMU1DataReady);
    } else {
      /* Restart I2C2 bus in case of an error. */
      i2cStop(&I2CD2);
      i2cStart(&I2CD2, &i2cfg_d2);
    }
    if ((g_boardStatus & MPU6050_HIGH_DETECTED) && mpu6050GetNewData(&g_IMU2)) {
      chBSemSignal(&bsemIMU2DataReady);
    } else {
      /* Restart I2C2 bus in case of an error. */
      i2cStop(&I2CD2);
      i2cStart(&I2CD2, &i2cfg_d2);
    }
    /* Wait until the next 1.5 milliseconds passes. */
    chThdSleepUntil(time += US2ST(1500));
  }
  /* This point should never be reached. */
  return 0;
}

/**
 * Attitude calculation thread.
 * - This thread works in conjunction with PollMPU6050Thread thread.
 * - This thread is synchronized by PollMPU6050Thread thread.
 * - This thread has the highest priority level.
 */
static WORKING_AREA(waAttitudeThread, 2048);
static msg_t AttitudeThread(void *arg) {
  (void)arg;
  attitudeInit();
  while (TRUE) {
    /* Process IMU1 new data ready event. */
    if (chBSemWait(&bsemIMU1DataReady) == RDY_OK) {
      if (g_boardStatus & IMU1_CALIBRATION_MASK) {
        if (imuCalibrate(&g_IMU1, g_boardStatus & IMU1_CALIBRATE_ACCEL)) {
          g_boardStatus &= ~IMU1_CALIBRATION_MASK;
        }
      } else {
        attitudeUpdate(&g_IMU1);
      }
    }
    /* Process IMU2 new data ready event. */
    if ((g_boardStatus & MPU6050_HIGH_DETECTED) && (chBSemWait(&bsemIMU2DataReady) == RDY_OK)) {
      if (g_boardStatus & IMU2_CALIBRATION_MASK) {
        if (imuCalibrate(&g_IMU2, g_boardStatus & IMU2_CALIBRATE_ACCEL)) {
          g_boardStatus &= ~IMU2_CALIBRATION_MASK;
        }
      } else {
        attitudeUpdate(&g_IMU2);
      }
    }
    if (g_boardStatus & IMU_CALIBRATION_MASK) {
      pwmOutputDisableAll();
    } else {
      cameraRotationUpdate();
      actuatorsUpdate();
    }
  }
  /* This point should never be reached. */
  return 0;
}

/**
 * @brief   Application entry point.
 * @details
 */
int main(void) {
  /* System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Initializes a serial-over-USB CDC driver. */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /* Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbStop(serusbcfg.usbp);
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(500);
  usbConnectBus(serusbcfg.usbp);
  usbStart(serusbcfg.usbp, &usbcfg);

  /* Activates the serial driver 4 using the driver's default configuration. */
  sdStart(&SD4, NULL);

  /* Activates the I2C driver 2. */
  i2cStart(&I2CD2, &i2cfg_d2);

  /* Enables the CRC peripheral clock. */
  rccEnableCRC(FALSE);

  /* Initialize IMU data structure. */
  imuStructureInit(&g_IMU1, FALSE); // IMU1 on low address.
  imuStructureInit(&g_IMU2, TRUE);  // IMU2 on high address.

  /* Loads settings from external EEPROM chip.
     WARNING! If MPU6050 sensor is not connected to the I2C bus, there
     aren't pull-up resistors on SDA and SCL lines, therefore it is
     impossible to communicate with EEPROM without the sensor connected. */
  if (eepromLoadSettings()) {
    g_boardStatus |= EEPROM_24C02_DETECTED;
  }

  /* Initializes the MPU6050 sensor1. */
  if (mpu6050Init(g_IMU1.addr)) {
    g_boardStatus |= MPU6050_LOW_DETECTED;
    g_boardStatus |= IMU1_CALIBRATE_GYRO;
  }
#if 0
  /* Initializes the MPU6050 sensor2. */
  if (mpu6050Init(g_IMU2.addr)) {
    g_boardStatus |= MPU6050_HIGH_DETECTED;
    g_boardStatus |= IMU2_CALIBRATE_GYRO;
  } else if (g_i2cErrorInfo.last_i2c_error == I2CD_ACK_FAILURE) {
    /* Device not found. */
    g_i2cErrorInfo.last_i2c_error = I2CD_NO_ERROR;
    g_i2cErrorInfo.i2c_error_counter--;
  }
#endif
  if (g_boardStatus & MPU6050_LOW_DETECTED) {
    /* Creates a taken binary semaphore. */
    chBSemInit(&bsemIMU1DataReady, TRUE);
    chBSemInit(&bsemIMU2DataReady, TRUE);

    /* Creates the MPU6050 polling thread and attitude calculation thread. */
    chThdCreateStatic(waPollMPU6050Thread, sizeof(waPollMPU6050Thread),
      NORMALPRIO + 1, PollMPU6050Thread, NULL);
    chThdCreateStatic(waAttitudeThread, sizeof(waAttitudeThread),
      HIGHPRIO, AttitudeThread, NULL);

    /* Starts motor drivers. */
    pwmOutputStart();

    /* Starts ADC and ICU input drivers. */
    mixedInputStart();
  }

  /* Creates the blinker thread. */
  chThdCreateStatic(waBlinkerThread, sizeof(waBlinkerThread),
    LOWPRIO, BlinkerThread, NULL);

  /* Normal main() thread activity. */
  while (TRUE) {
    g_chnp = serusbcfg.usbp->state == USB_ACTIVE ? (BaseChannel *)&SDU1 : (BaseChannel *)&SD4;
    telemetryReadSerialData();
    if ((g_boardStatus & EEPROM_24C02_DETECTED) && eepromIsDataLeft()) {
      eepromContinueSaving();
    }
    chThdSleepMilliseconds(TELEMETRY_SLEEP_MS);
  }
  /* This point should never be reached. */
  return 0;
}
