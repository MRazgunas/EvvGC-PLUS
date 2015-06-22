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

#include "ch.h"
#include "hal.h"

#include "mpu6050.h"
#include "pwmio.h"
#include "misc.h"
#include "attitude.h"
#include <parameters.h>
#include "AP_Param.h"

/* C libraries: */
#include <string.h>

#define FIXED_DT_STEP             0.0015f

#define MOTOR_STEP_LIMIT_MAX      M_PI / 45.0f
#define MOTOR_STEP_LIMIT_MIN      -MOTOR_STEP_LIMIT_MAX

#define ACCEL_TAU                 0.1f
#define INPUT_SIGNAL_ALPHA        300.0f
#define MODE_FOLLOW_DEAD_BAND     M_PI / 36.0f

/* Input modes: */
#define INPUT_MODE_ANGLE          0x00
#define INPUT_MODE_SPEED          0x01
#define INPUT_MODE_FOLLOW         0x02

/* PID controller structure. */
typedef struct tagPIDStruct {
  float P;
  float I;
  float D;
  float prevDist;
  float prevSpeed;
  float prevCmd;
} __attribute__((packed)) PIDStruct, *PPIDStruct;

const AP_Param::GroupInfo PIDSettings::var_info[] PROGMEM = {

   // @Param: P
   // @DisplayName: P value for controler
   // @Description: P value for controler
   // @Values: 0 128
   // @User: Advanced
   AP_GROUPINFO("P", 0, PIDSettings, P, 0),

   // @Param: I
   // @DisplayName: I value for controler
   // @Description: I value for controler
   // @Values: 0 128
   // @User: Advanced
   AP_GROUPINFO("I", 1, PIDSettings, I, 0),

   // @Param: D
   // @DisplayName: D value for controler
   // @Description: D value for controler
   // @Values: 0 128
   // @User: Advanced
   AP_GROUPINFO("D", 2, PIDSettings, D, 0),

   AP_GROUPEND
};

const AP_Param::GroupInfo InputMode::var_info[] PROGMEM = {

   // @Param: MIN_ANGLE
   // @DisplayName: Minimum angle
   // @Description: Minimal angle which gimbal can reach in ANGLE mode
   // @Units: degrees
   // @Range: -16512 16512
   // @Increment: 1
   // @User: Advanced
   AP_GROUPINFO("MIN_ANGLE", 0, InputMode, min_angle, -60),

   // @Param: MAX_ANGLE
   // @DisplayName: Maximum angle
   // @Description: Maximum angle which gimbal can reach in ANGLE mode
   // @Units: degrees
   // @Range: -16512 16512
   // @Increment: 1
   // @User: Advanced
   AP_GROUPINFO("MAX_ANGLE", 1, InputMode, max_angle, 60),

   // @Param: OFFSET
   // @DisplayName: Angle offset
   // @Description: Offset from 0 degrees in angle mode
   // @Units: degrees
   // @Range: -16512 16512
   // @Increment: 1
   // @User: Advanced
   AP_GROUPINFO("OFFSET", 2, InputMode, offset, 0),

   // @Param: OFFSET
   // @DisplayName: Angle offset
   // @Description: Offset from 0 degrees in angle mode
   // @Units: degrees/s
   // @Range: -16512 16512
   // @Increment: 1
   // @User: Advanced
   AP_GROUPINFO("SPEED", 3, InputMode, speed, 0),

   // @Param: INPUT_MODE
   // @DisplayName: Input mode
   // @Description: Mode of input
   // @Values: 0:Angle mode 1:Speed mode 2:Follow mode
   // @Increment: 1
   // @User: Advanced
   AP_GROUPINFO("INPUT_MODE", 4, InputMode, mode_id, INPUT_MODE_ANGLE),

   AP_GROUPEND
};


extern Parameters g;
/**
 * Global variables.
 */
/* Mechanical offset of the motors. */
float g_motorOffset[3] = {0.0f, 0.0f, 0.0f};

/**
 * PID settings.
 */
PIDSettings g_pidSettings[3];

/**
 * Input mode settings.
 */
InputMode g_modeSettings[3];

/**
 * Local variables
 */
static float camAtti[3] = {0.0f};
static float camRot[3] = {0.0f};
static float camRotSpeedPrev[3] = {0.0f};

static float accel2Kp = 30.0f;
static float accel2Ki = 0.001f;

/* Accelerometer filter variables. */
static uint8_t fAccelFilterEnabled = TRUE;
static float accel_alpha = 0.0f;

/* PID controller parameters. */
static PIDStruct PID[3] = {
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

/**
 * @brief  Implements basic PID stabilization of the motor speed.
 * @param  cmd_id - command id to apply PID action to.
 * @param  err - process error value.
 * @param  rot - rotation command value.
 * @return weighted sum of P, I and D actions.
 */
static float pidControllerApply(uint8_t cmd_id, float err, float rot) {
  float poles2 = (float)(g_pwmOutput[cmd_id].num_poles >> 1);
  /* Distance for the motor to travel: */
  float dist = circadjust(err, M_PI);
  /* Convert mechanical distance to electrical distance: */
  dist *= poles2;
  /* Convert mechanical rotation to electrical rotation: */
  rot *= poles2;
  /* If there is a distance to travel then rotate the motor in small steps: */
  float step = constrain(dist*PID[cmd_id].I, MOTOR_STEP_LIMIT_MIN, MOTOR_STEP_LIMIT_MAX);
  /* Calculate proportional speed of the motor: */
  float speed = dist - PID[cmd_id].prevDist;
  step += speed*PID[cmd_id].P;
  /* Account for the acceleration of the motor: */
  step += (speed - PID[cmd_id].prevSpeed)*PID[cmd_id].D;
  /* Add rotation command. */
  step += rot;
  /* Update offset of the motor: */
  g_motorOffset[cmd_id] += step / poles2;
  /* Wind-up guard limits motor offset range to one mechanical rotation: */
  g_motorOffset[cmd_id] = circadjust(g_motorOffset[cmd_id], M_PI);
  /* Update motor position: */
  float cmd = PID[cmd_id].prevCmd + step;
  /* Normalize command to -M_PI..M_PI range: */
  if (cmd < 0.0f) {
    cmd = fmodf(cmd - M_PI, -M_TWOPI) + M_PI;
  } else {
    cmd = fmodf(cmd + M_PI,  M_TWOPI) - M_PI;
  }
  /* Save values for the next iteration: */
  PID[cmd_id].prevDist = dist;
  PID[cmd_id].prevSpeed = speed;
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
static void cfUpdateSettings(void) {
  accel2Kp = g.cf_2kp * 0.1f;
  accel2Ki = g.cf_2ki * 0.00001f;
}

/**
 * @brief  First order low-pass filter.
 * @param  raw - pointer to raw data array;
 * @param  filtered - pointer to filtered data array;
 */
static void accelFilterApply(const float raw[], float filtered[]) {
  if (fAccelFilterEnabled) {
    filtered[0] = (filtered[0] - raw[0])*accel_alpha + raw[0];
    filtered[1] = (filtered[1] - raw[1])*accel_alpha + raw[1];
    filtered[2] = (filtered[2] - raw[2])*accel_alpha + raw[2];
  } else {
    memcpy((void *)filtered, (void *)raw, sizeof(float)*3);
  }
}

/**
 * @brief
 */
void attitudeInit(void) {
  memset((void *)PID, 0, sizeof(PID));
  pidUpdateStruct();
  cfUpdateSettings();
  accel_alpha = expf(-FIXED_DT_STEP / ACCEL_TAU);
}

/**
 * @brief
 */
__attribute__((optimize("O3"))) void attitudeUpdate(PIMUStruct pIMU) {
  float accelErr[3] = {0.0f};
  float mag;
  float dq[4];

  pIMU->accelData[0] -= pIMU->accelBias[0];
  pIMU->accelData[1] -= pIMU->accelBias[1];
  pIMU->accelData[2] -= pIMU->accelBias[2];

  // Account for accel's magnitude.
  mag = QInvSqrtf(pIMU->accelData[0]*pIMU->accelData[0] + pIMU->accelData[1]*pIMU->accelData[1] + pIMU->accelData[2]*pIMU->accelData[2]);

  if ((mag > 0.0724f) && (mag < 0.1724f)) {
    float v2[3];

    /* Compute estimated direction of gravity halved.
     *
     * Rotated gravity vector v2 is calculated by multiplying gravity
     * vector v={0,0,1} by conjugate (q`) of attitude quaternion (q) (v2=q`vq),
     * because MPU6050 senses gravity in opposite direction.
     */
    v2[0] = (pIMU->qIMU[1]*pIMU->qIMU[3] - pIMU->qIMU[0]*pIMU->qIMU[2]);
    v2[1] = (pIMU->qIMU[2]*pIMU->qIMU[3] + pIMU->qIMU[0]*pIMU->qIMU[1]);
    v2[2] = (pIMU->qIMU[0]*pIMU->qIMU[0] + pIMU->qIMU[3]*pIMU->qIMU[3]) - 0.5f;

    // Apply smoothing to accel values, to reduce vibration noise before main calculations.
    accelFilterApply(pIMU->accelData, pIMU->accelFiltered);
    // Apply the same filtering to the estimated direction of gravity to match phase shift.
    accelFilterApply(v2, pIMU->v2Filtered);
    // Compute the error between the predicted direction of gravity and smoothed acceleration.
    CrossProduct(pIMU->accelFiltered, pIMU->v2Filtered, accelErr);

    // Normalize accel_error.
    accelErr[0] *= mag;
    accelErr[1] *= mag;
    accelErr[2] *= mag;
  }

  // Correct rates based on error.
  pIMU->gyroBias[0] -= accelErr[0]*accel2Ki;
  pIMU->gyroBias[1] -= accelErr[1]*accel2Ki;
  pIMU->gyroBias[2] -= accelErr[2]*accel2Ki;

  pIMU->gyroData[0] -= pIMU->gyroBias[0];
  pIMU->gyroData[1] -= pIMU->gyroBias[1];
  pIMU->gyroData[2] -= pIMU->gyroBias[2];

  // Correct rates based on error.
  pIMU->gyroData[0] += accelErr[0]*accel2Kp;
  pIMU->gyroData[1] += accelErr[1]*accel2Kp;
  pIMU->gyroData[2] += accelErr[2]*accel2Kp;

  dq[0] = (-pIMU->qIMU[1]*pIMU->gyroData[0] - pIMU->qIMU[2]*pIMU->gyroData[1] - pIMU->qIMU[3]*pIMU->gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[1] = ( pIMU->qIMU[0]*pIMU->gyroData[0] - pIMU->qIMU[3]*pIMU->gyroData[1] + pIMU->qIMU[2]*pIMU->gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[2] = ( pIMU->qIMU[3]*pIMU->gyroData[0] + pIMU->qIMU[0]*pIMU->gyroData[1] - pIMU->qIMU[1]*pIMU->gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);
  dq[3] = (-pIMU->qIMU[2]*pIMU->gyroData[0] + pIMU->qIMU[1]*pIMU->gyroData[1] + pIMU->qIMU[0]*pIMU->gyroData[2])*(FIXED_DT_STEP*DEG2RAD*0.5f);

  pIMU->qIMU[0] += dq[0];
  pIMU->qIMU[1] += dq[1];
  pIMU->qIMU[2] += dq[2];
  pIMU->qIMU[3] += dq[3];

  // Normalize attitude quaternion.
  mag = QInvSqrtf(pIMU->qIMU[0]*pIMU->qIMU[0] + pIMU->qIMU[1]*pIMU->qIMU[1] + pIMU->qIMU[2]*pIMU->qIMU[2] + pIMU->qIMU[3]*pIMU->qIMU[3]);
  pIMU->qIMU[0] *= mag;
  pIMU->qIMU[1] *= mag;
  pIMU->qIMU[2] *= mag;
  pIMU->qIMU[3] *= mag;

  // Convert attitude into Euler angles;
  Quaternion2RPY(pIMU->qIMU, pIMU->rpy);
}

/**
 * @brief
 */
void cameraRotationUpdate(void) {
  uint8_t i;
  float coef;
  float speedLimit;

  for (i = 0; i < 3; i++) {
    speedLimit = ((float)g_modeSettings[i].speed)*DEG2RAD;

    if (g_modeSettings[i].mode_id & INPUT_MODE_FOLLOW) {
      /* Calculate offset of the gimbal: */
      coef = g_modeSettings[i].offset*DEG2RAD - g_motorOffset[i];
      if (coef > MODE_FOLLOW_DEAD_BAND) {
        coef -= MODE_FOLLOW_DEAD_BAND;
        /* Convert to speed: */
        coef /= INPUT_SIGNAL_ALPHA*FIXED_DT_STEP;
      } else if (coef < -MODE_FOLLOW_DEAD_BAND) {
        coef += MODE_FOLLOW_DEAD_BAND;
        /* Convert to speed: */
        coef /= INPUT_SIGNAL_ALPHA*FIXED_DT_STEP;
      } else {
        coef = 0.0f;
      }
    } else if (g_mixedInput[i].channel_id == INPUT_CHANNEL_DISABLED) {
      camRot[i] = 0.0f;
      continue;
    } else {
      /* Calculate input scaling coefficient: */
      if (g_mixedInput[i].max_val == g_mixedInput[i].min_val) {
        /* Avoid divisions by zero. */
        coef = 0.0f;
      } else {
        coef = ((float)(g_inputValues[g_mixedInput[i].channel_id] - g_mixedInput[i].mid_val)) /
               ((float)(g_mixedInput[i].max_val - g_mixedInput[i].min_val));
      }

      if (g_modeSettings[i].mode_id & INPUT_MODE_SPEED) {
        /* Calculate speed from RC input data: */
        coef *= 2.0f*speedLimit;
        camRotSpeedPrev[i] += (coef - camRotSpeedPrev[i]) / INPUT_SIGNAL_ALPHA;
        coef = camRotSpeedPrev[i];
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
    camRot[i] = coef*FIXED_DT_STEP;
  }
}

/**
 * @brief
 */
void actuatorsUpdate(void) {
  float cmd = 0.0f;
  float err;
  /* Pitch: */
  uint8_t cmd_id = g_pwmOutput[PWM_OUT_PITCH].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    err = camAtti[cmd_id] - g_IMU1.rpy[cmd_id];
    camAtti[cmd_id] += camRot[cmd_id];
    camAtti[cmd_id] = circadjust(camAtti[cmd_id], M_PI);
    cmd = pidControllerApply(cmd_id, err, camRot[cmd_id]);
  }
  pwmOutputUpdate(PWM_OUT_PITCH, cmd);
  cmd = 0.0f;
  /* Roll: */
  cmd_id = g_pwmOutput[PWM_OUT_ROLL].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    err = camAtti[cmd_id] - g_IMU1.rpy[cmd_id];
    camAtti[cmd_id] += camRot[cmd_id];
    camAtti[cmd_id] = circadjust(camAtti[cmd_id], M_PI);
    cmd = pidControllerApply(cmd_id, err, camRot[cmd_id]);
  }
  pwmOutputUpdate(PWM_OUT_ROLL, cmd);
  cmd = 0.0f;
  /* Yaw: */
  cmd_id = g_pwmOutput[PWM_OUT_YAW].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (cmd_id != PWM_OUT_CMD_DISABLED) {
    err = camAtti[cmd_id] - g_IMU1.rpy[cmd_id];
    camAtti[cmd_id] += camRot[cmd_id];
    camAtti[cmd_id] = circadjust(camAtti[cmd_id], M_PI);
    cmd = pidControllerApply(cmd_id, err, camRot[cmd_id]);
  }
  pwmOutputUpdate(PWM_OUT_YAW, cmd);
}

/**
 * @brief
 */
void pidSettingsUpdate(const PPIDSettings pNewSettings) {
   enum ap_var_type var_type;
   AP_Param *vp;
   vp = AP_Param::set_param_by_name("PITCH_P", pNewSettings[0].P, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("PITCH_I", pNewSettings[0].I, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("PITCH_D", pNewSettings[0].D, &var_type);
   vp->save();

   vp = AP_Param::set_param_by_name("ROLL_P", pNewSettings[1].P, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("ROLL_I", pNewSettings[1].I, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("ROLL_D", pNewSettings[1].D, &var_type);
   vp->save();

   vp = AP_Param::set_param_by_name("YAW_P", pNewSettings[2].P, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("YAW_I", pNewSettings[2].I, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("YAW_D", pNewSettings[2].D, &var_type);
   vp->save();

  pidUpdateStruct();
}

/**
 * @brief
 */
void inputModeSettingsUpdate(const PInputModeStruct pNewSettings) {
   enum ap_var_type var_type;
   AP_Param *vp;
   vp = AP_Param::set_param_by_name("PITCH_MIN_ANGLE", pNewSettings[0].min_angle, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("PITCH_MAX_ANGLE", pNewSettings[0].max_angle, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("PITCH_OFFSET", pNewSettings[0].offset, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("PITCH_SPEED", pNewSettings[0].speed, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("PITCH_INPUT_MODE", pNewSettings[0].mode_id, &var_type);
   vp->save();

   vp = AP_Param::set_param_by_name("ROLL_MIN_ANGLE", pNewSettings[1].min_angle, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("ROLL_MAX_ANGLE", pNewSettings[1].max_angle, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("ROLL_OFFSET", pNewSettings[1].offset, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("ROLL_SPEED", pNewSettings[1].speed, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("ROLL_INPUT_MODE", pNewSettings[1].mode_id, &var_type);
   vp->save();

   vp = AP_Param::set_param_by_name("YAW_MIN_ANGLE", pNewSettings[2].min_angle, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("YAW_MAX_ANGLE", pNewSettings[2].max_angle, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("YAW_OFFSET", pNewSettings[2].offset, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("YAW_SPEED", pNewSettings[2].speed, &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("YAW_INPUT_MODE", pNewSettings[2].mode_id, &var_type);
   vp->save();
}

/**
 * @brief
 */
void cfSettingsUpdate(const uint16_t *pNewSettings) {
   uint16_t g_cfSettings[2];
   memcpy((void *)&g_cfSettings, (void *)pNewSettings, sizeof(g_cfSettings));
   enum ap_var_type var_type;
   AP_Param *vp;
   vp = AP_Param::set_param_by_name("COMP_F_2KP", g_cfSettings[0], &var_type);
   vp->save();
   vp = AP_Param::set_param_by_name("COMP_F_2KI", g_cfSettings[1], &var_type);
   vp->save();
  cfUpdateSettings();
}
