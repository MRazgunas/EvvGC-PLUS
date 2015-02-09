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

uint32_t g_boardStatus = 0;

/* I2C2 configuration for I2C driver 2 */
static const I2CConfig i2cfg_d2 = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2,
};

/* Virtual serial port over USB. */
SerialUSBDriver SDU1;

/* Binary semaphore indicating that new data is ready to be processed. */
static BinarySemaphore bsemNewDataReady;

/**
 * Red LED blinker thread. Times are in milliseconds.
 */
static WORKING_AREA(waBlinkerThread, 64);
static msg_t BlinkerThread(void *arg) {
  (void)arg;
  while (TRUE) {
    systime_t time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 500;
    palTogglePad(GPIOB, GPIOB_LED_RED);
    chThdSleepMilliseconds(time);
  }
  /* This point should never be reached. */
  return 0;
}

/**
 * MPU6050 data polling thread. Times are in milliseconds.
 * This thread requests a new data from MPU6050 every 2 ms (@500 Hz).
 */
static WORKING_AREA(waPollMPU6050Thread, 128);
static msg_t PollMPU6050Thread(void *arg) {
  systime_t time;
  (void)arg;
  time = chTimeNow();
  while (TRUE) {
    if (mpu6050GetNewData()) {
      chBSemSignal(&bsemNewDataReady);
    }
    /* Wait until the next 2 milliseconds passes. */
    chThdSleepUntil(time += MS2ST(2));
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
    if (chBSemWait(&bsemNewDataReady) == RDY_OK) {
      attitudeUpdate();
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
  i2cInit();
  i2cStart(&I2CD2, &i2cfg_d2);
    
  /* Enables the CRC peripheral clock. */
  rccEnableCRC(FALSE);

  /* Loads settings from external EEPROM chip. */
  if (eepromLoadSettings()) {
    g_boardStatus |= 1;
  }

  /* Initializes the MPU6050 sensor. */
  if (mpu6050Init()) {
    g_boardStatus |= 2;

    /* Creates a taken binary semaphore. */
    chBSemInit(&bsemNewDataReady, TRUE);

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
    if (eepromIsDataLeft()) {
      g_boardStatus |= 4;
      eepromContinueSaving();  
    }
    chThdSleepMilliseconds(TELEMETRY_SLEEP_MS);
  }
  /* This point should never be reached. */
  return 0;
}
