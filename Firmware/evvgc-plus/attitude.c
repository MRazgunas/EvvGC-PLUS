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

/*
  References:
  [1] Mahony, R.; Hamel, T.; Pflimlin, Jean-Michel, "Nonlinear Complementary
      Filters on the Special Orthogonal Group", Automatic Control,
      IEEE Transactions on, vol.53, no.5, pp.1203,1218, June 2008
  [2] Euston, M.; Coote, P.; Mahony, R.; Jonghyuk Kim; Hamel, T.,
      "A complementary filter for attitude estimation of a fixed-wing UAV",
      Intelligent Robots and Systems, 2008. IROS 2008. IEEE/RSJ International
      Conference on, vol., no., pp.340,345, 22-26 Sept. 2008
*/

/*
  Note:
  - the order of rotation of the standard 3D-gimbal system is:
    Pitch (X) then Roll (Y) and then Yaw (Z).
*/

/* C libraries: */
#include <string.h>

#include "misc.h"
#include "attitude.h"
#include "mpu6050.h"
#include "pwmio.h"

#define CALIBRATION_COUNTER_MAX   5000
#define FIXED_DT_STEP             0.002f

/* PID integral error limit: 1 mechanical degree per
   iteration or ~1.4 mechanical rotations per second. */
#define PID_INTEGRAL_ERROR_MAX    M_PI / 180.0f
#define PID_INTEGRAL_ERROR_MIN    -PID_INTEGRAL_ERROR_MAX

#define STEP_LIMIT                M_PI / 6.0f

#define INPUT_SIGNAL_ALPHA        200.0f
#define MODE_FOLLOW_DEAD_BAND     M_PI / 36.0f

/* PID controller structure. */
typedef struct tagPIDStruct {
  float P;
  float I;
  float D;
  float prevErr;
  float prevCmd;
} __attribute__((packed)) PIDStruct, *PPIDStruct;

/**
 * Global variables.
 */
/* Attitude quaternion of the IMU. */
float g_qIMU[4] = {1.0f, 0.0f, 0.0f, 0.0f};
/* Electrical offset of the motors. */
float g_motorOffset[3] = {0.0f};

/**
 * Default PID settings.
 */
PIDSettings g_pidSettings[3] = {
/* P, I, D */
  {0, 0, 0}, /* Pitch PID */
  {0, 0, 0}, /* Roll  PID */
  {0, 0, 0}, /* Yaw   PID */
};

/**
 * Default input mode settings.
 */
InputModeStruct g_modeSettings[3] = {
  {-60,               /* Min angle */
   60,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_ANGLE}, /* Mode ID   */
  {-60,               /* Min angle */
   60,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_ANGLE}, /* Mode ID   */
  {-90,               /* Min angle */
   90,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_SPEED}, /* Mode ID   */
};

/**
 * Local variables
 */
static float camAtti[3] = {0.0f};
static float camRot[3] = {0.0f};
static float camErr[3] = {0.0f};
static float camRotSpeedPrev[3] = {0.0f};

static float gyroBias[3] = {0.0f};
static float accelBias[3] = {0.0f};

static float accelKp = 0.02f;
static float accelKi = 0.0002;

/* Calibration related variables */
static uint8_t fCalibrateAccel = 0;
static uint8_t fCalibrateGyro = 1;
static uint16_t counter = 0;
static float accum[3] = {0.0f};

/* PID controller parameters. */
static PIDStruct PID[3];

/**
 * @brief  Implements basic PID stabilization.
 * @param  cmd_id - command id to apply PID action to.
 * @param  err - difference between measured value and set-point value.
 * @return weighted sum of P, I and D actions.
 */
static float pidControllerApply(uint8_t cmd_id, float err) {
  float ierr = constrain(err, PID_INTEGRAL_ERROR_MIN, PID_INTEGRAL_ERROR_MAX);
  float poles2 = (float)(g_pwmOutput[cmd_id].num_poles >> 1);
  /* Convert mechanical error to electrical error: */
  err *= poles2;
  ierr *= poles2;
  /* Update electrical offset of the motor: */
  g_motorOffset[cmd_id] += ierr * PID[cmd_id].I;
  /* Wind-up guard limits motor offset range to one mechanical rotation: */
  if (g_motorOffset[cmd_id] < 0.0f) {
    g_motorOffset[cmd_id] = fmodf(g_motorOffset[cmd_id] - M_PI*poles2, -M_TWOPI*poles2) + M_PI*poles2;
  } else {
    g_motorOffset[cmd_id] = fmodf(g_motorOffset[cmd_id] + M_PI*poles2,  M_TWOPI*poles2) - M_PI*poles2;
  }
  float diff = err - PID[cmd_id].prevErr;
  PID[cmd_id].prevErr = err;
  /* Calculate the real command value: */
  float cmd = err*PID[cmd_id].P + g_motorOffset[cmd_id] + diff*PID[cmd_id].D;
  /* Convert command value to [-pi..pi] range by removing motor offset: */
  if (cmd < 0.0f) {
    cmd = fmodf(cmd - M_PI, -M_TWOPI) + M_PI;
  } else {
    cmd = fmodf(cmd + M_PI,  M_TWOPI) - M_PI;
  }
  /* Over-speeding guard:
     - limit motor speed to STEP_LIMIT electrical degrees per iteration.
   */
  diff = circadjust(PID[cmd_id].prevCmd - cmd, M_PI);
  if (diff > STEP_LIMIT) {
    cmd = circadjust(PID[cmd_id].prevCmd - STEP_LIMIT, M_PI);
  } else if (diff < -STEP_LIMIT) {
    cmd = circadjust(PID[cmd_id].prevCmd + STEP_LIMIT, M_PI);
  }
  PID[cmd_id].prevCmd = cmd;
  return cmd;
}

/**
 * @brief
 */
static void pidUpdateStruct(void) {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    PID[i].P = (float)g_pidSettings[i].P*0.1f;
    PID[i].I = (float)g_pidSettings[i].I*0.01f;
    PID[i].D = (float)g_pidSettings[i].D*1.0f;
    if (!g_pidSettings[i].I) {
      g_motorOffset[i] = 0.0f;
    }
  }
}

/**
 * @brief
 */
static void cameraRotationUpdate(void) {
  uint8_t i;
  float coef;
  float speedLimit;

  for (i = 0; i < 3; i++) {
    speedLimit = ((float)g_modeSettings[i].speed) * DEG2RAD;

    if (g_modeSettings[i].mode_id & INPUT_MODE_FOLLOW) {
      /* Calculate offset of the gimbal: */
      coef = g_modeSettings[i].offset * DEG2RAD - g_motorOffset[i] / (g_pwmOutput[i].num_poles >> 1);
      if (coef > MODE_FOLLOW_DEAD_BAND) {
        coef -= MODE_FOLLOW_DEAD_BAND;
        /* Convert to speed: */
        coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;
      } else if (coef < -MODE_FOLLOW_DEAD_BAND) {
        coef += MODE_FOLLOW_DEAD_BAND;
        /* Convert to speed: */
        coef /= INPUT_SIGNAL_ALPHA * FIXED_DT_STEP;
      } else {
        coef = 0.0f;
      }
    } else if (g_mixedInput[i].channel_id == INPUT_CHANNEL_DISABLED) {
      camRot[i] = 0.0f;
      continue;
    } else {
      /* Calculate input scaling coefficient: */
      coef = ((float)(g_inputValues[g_mixedInput[i].channel_id] - g_mixedInput[i].mid_val)) /
             ((float)(g_mixedInput[i].max_val - g_mixedInput[i].min_val));

      if (g_modeSettings[i].mode_id & INPUT_MODE_SPEED) {
        /* Calculate speed from RC input data: */
        coef *= speedLimit * 2.0f;
        camRotSpeedPrev[i] += (coef - camRotSpeedPrev[i]) / INPUT_SIGNAL_ALPHA;
        coef = camRotSpeedPrev[i];
        // TODO: calculate speed limitations based on attitude constrains;
      } else { /* INPUT_MODE_ANGLE */
        /* Calculate angle from input data: */
        coef *= (g_modeSettings[i].max_angle - g_modeSettings[i].min_angle);
        coef += (g_modeSettings[i].max_angle + g_modeSettings[i].min_angle) / 2;
        coef = constrain(coef, (float)g_modeSettings[i].min_angle, (float)g_modeSettings[i].max_angle);
        coef *= DEG2RAD;

        /* Convert angle difference to speed: */
        coef = (coef - camRot[i]) / INPUT_SIGNAL_ALPHA / FIXED_DT_STEP;
      }
    }
    coef = constrain(coef, -speedLimit, speedLimit);
    camRot[i] += coef * FIXED_DT_STEP;
    camRot[i] = circadjust(camRot[i], M_PI);
  }
}

/**
 * @brief
 */
void attitudeInit(void) {
  memset((void *)PID, 0, sizeof(PID));
  pidUpdateStruct();
}

/**
 * @brief
 */
void attitudeUpdate(void) {
  float accelErr[3] = {0.0f};
  float mag;
  float dq[4];

  if (fCalibrateGyro) {
    if (counter++ < CALIBRATION_COUNTER_MAX) {
      accum[0] += g_gyroData[0];
      accum[1] += g_gyroData[1];
      accum[2] += g_gyroData[2];
      return;
    } else {
      gyroBias[0] = accum[0] / CALIBRATION_COUNTER_MAX;
      gyroBias[1] = accum[1] / CALIBRATION_COUNTER_MAX;
      gyroBias[2] = accum[2] / CALIBRATION_COUNTER_MAX;

      counter = 0;
      accum[0] = 0.0f;
      accum[1] = 0.0f;
      accum[2] = 0.0f;

      fCalibrateGyro = 0;
    }
  }

  if (fCalibrateAccel) {
    if (counter++ < CALIBRATION_COUNTER_MAX) {
      accum[0] += g_accelData[0];
      accum[1] += g_accelData[1];
      accum[2] += g_accelData[2] + GRAV;
      return;
    } else {
      accelBias[0] = accum[0] / CALIBRATION_COUNTER_MAX;
      accelBias[1] = accum[1] / CALIBRATION_COUNTER_MAX;
      accelBias[2] = accum[2] / CALIBRATION_COUNTER_MAX;

      counter = 0;
      accum[0] = 0.0f;
      accum[1] = 0.0f;
      accum[2] = 0.0f;

      fCalibrateAccel = 0;
    }
  }

  g_accelData[0] -= accelBias[0];
  g_accelData[1] -= accelBias[1];
  g_accelData[2] -= accelBias[2];

  g_gyroData[0] -= gyroBias[0];
  g_gyroData[1] -= gyroBias[1];
  g_gyroData[2] -= gyroBias[2];

  // Account for accel's magnitude.
  mag = QInvSqrtf(g_accelData[0]*g_accelData[0] + g_accelData[1]*g_accelData[1] + g_accelData[2]*g_accelData[2]);

  if ((mag > 0.0724f) && (mag < 0.1724f)) {
    float grot[3];

    // Rotate gravity to body frame and cross with accels.
    grot[0] = -(2.0f*(g_qIMU[1]*g_qIMU[3] - g_qIMU[0]*g_qIMU[2]));
    grot[1] = -(2.0f*(g_qIMU[2]*g_qIMU[3] + g_qIMU[0]*g_qIMU[1]));
    grot[2] =  (2.0f*(g_qIMU[1]*g_qIMU[1] + g_qIMU[2]*g_qIMU[2])) - 1.0f;

    // Compute the error between the predicted direction of gravity and smoothed acceleration.
    CrossProduct(g_accelData, grot, accelErr);

    // Normalize accel_error.
    accelErr[0] *= mag;
    accelErr[1] *= mag;
    accelErr[2] *= mag;
  }

  // Correct rates based on error.
  g_gyroData[0] += accelErr[0]*(accelKp / FIXED_DT_STEP);
  g_gyroData[1] += accelErr[1]*(accelKp / FIXED_DT_STEP);
  g_gyroData[2] += accelErr[2]*(accelKp / FIXED_DT_STEP);

  // Correct rates based on error.
  gyroBias[0] -= accelErr[0]*accelKi;
  gyroBias[1] -= accelErr[1]*accelKi;
  gyroBias[2] -= accelErr[2]*accelKi;

  dq[0] = (-g_qIMU[1]*g_gyroData[0] - g_qIMU[2]*g_gyroData[1] - g_qIMU[3]*g_gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[1] = ( g_qIMU[0]*g_gyroData[0] - g_qIMU[3]*g_gyroData[1] + g_qIMU[2]*g_gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[2] = ( g_qIMU[3]*g_gyroData[0] + g_qIMU[0]*g_gyroData[1] - g_qIMU[1]*g_gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[3] = (-g_qIMU[2]*g_gyroData[0] + g_qIMU[1]*g_gyroData[1] + g_qIMU[0]*g_gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);

  g_qIMU[0] += dq[0];
  g_qIMU[1] += dq[1];
  g_qIMU[2] += dq[2];
  g_qIMU[3] += dq[3];

  // Normalize attitude quaternion g_qIMU.
  mag = QInvSqrtf (g_qIMU[0]*g_qIMU[0] + g_qIMU[1]*g_qIMU[1] + g_qIMU[2]*g_qIMU[2] + g_qIMU[3]*g_qIMU[3]);
  g_qIMU[0] *= mag;
  g_qIMU[1] *= mag;
  g_qIMU[2] *= mag;
  g_qIMU[3] *= mag;

  cameraRotationUpdate();

  // Convert attitude into Euler degrees;
  Quaternion2RPY (g_qIMU, camAtti);

  // Calculate attitude error for PID controller to minimize;
  camErr[0] = circadjust(camRot[0] - camAtti[0], M_PI);
  camErr[2] = circadjust(camRot[2] - camAtti[2], M_PI);
  camErr[1] = camRot[1] - camAtti[1];
}

/**
 * @brief
 */
void actuatorsUpdate(void) {
  float cmd = 0.0f;
  /* Pitch: */
  uint8_t cmd_id = g_pwmOutput[PWM_OUT_PITCH].cmd_id;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    cmd = pidControllerApply(cmd_id, camErr[cmd_id]);
  }
  pwmOutputUpdate(PWM_OUT_PITCH, cmd);
  /* Roll: */
  cmd_id = g_pwmOutput[PWM_OUT_ROLL].cmd_id;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    cmd = pidControllerApply(cmd_id, camErr[cmd_id]);
  }
  pwmOutputUpdate(PWM_OUT_ROLL, cmd);
  /* Yaw: */
  cmd_id = g_pwmOutput[PWM_OUT_YAW].cmd_id;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    cmd = pidControllerApply(cmd_id, camErr[cmd_id]);
  }
  pwmOutputUpdate(PWM_OUT_YAW, cmd);
}

/**
 * @brief
 */
void pidSettingsUpdate(const PPIDSettings pNewSettings) {
  memcpy((void *)&g_pidSettings, (void *)pNewSettings, sizeof(g_pidSettings));
  pidUpdateStruct();
}

/**
 * @brief
 */
void inputModeSettingsUpdate(const PInputModeStruct pNewSettings) {
  memcpy((void *)&g_modeSettings, (void *)pNewSettings, sizeof(g_modeSettings));
}
