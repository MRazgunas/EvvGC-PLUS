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

#ifndef _MISC_H_
#define _MISC_H_

#include <math.h>

#define RAD2DEG     ( 180.0f / M_PI )
#define DEG2RAD     ( M_PI / 180.0f )

#define constrain(val,min,max)  ((val)<(min)?(min):((val)>(max)?(max):(val)))
#define circadjust(val,lim)     ((val)<-(lim)?(val)+2*(lim):((val)>(lim)?(val)-2*(lim):(val)))

typedef struct tagI2CErrorStruct {
  i2cflags_t last_i2c_error;
  uint32_t i2c_error_counter;
} __attribute__((packed)) I2CErrorStruct, *PI2CErrorStruct;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fast Inverse Square Root approximation from www.gamedev.net
 * with magic number proposed by Chris Lomont:
 * - Lomont, Chris (February 2003).
 */
static inline float QInvSqrtf(float x) {
  float xhalf = 0.5f * x;
  union {
    uint32_t i;
    float    f;
  } y;
  y.f = x;
  y.i = 0x5f375a86 - (y.i >> 1);      // gives initial guess y0
  y.f *= 1.5f - xhalf * y.f * y.f;    // First Newton step, repeating increases accuracy
  //y.f *= 1.5f - xhalf * y.f * y.f;    // Second Newton step
  // and so on...
  return y.f;
}

/**
 * res = v1 x v2;
 */
static inline void CrossProduct(const float v1[3], const float v2[3], float res[3]) {
  res[0] = v1[1]*v2[2] - v2[1]*v1[2];
  res[1] = v2[0]*v1[2] - v1[0]*v2[2];
  res[2] = v1[0]*v2[1] - v2[0]*v1[1];
}

/**
 * @brief Find quaternion from roll, pitch and yaw.
 * @note  The order of rotations is:
 *        1. pitch (X);
 *        2. roll (Y);
 *        3. yaw (Z).
 */
static inline void RPY2Quaternion (const float rpy[3], float q[4]) {
  float phi, theta, psi;
  float cphi, sphi, ctheta, stheta, cpsi, spsi;

  phi    = rpy[0]*0.5f;
  theta  = rpy[1]*0.5f;
  psi    = rpy[2]*0.5f;

  cphi   = cosf(phi);
  sphi   = sinf(phi);
  ctheta = cosf(theta);
  stheta = sinf(theta);
  cpsi   = cosf(psi);
  spsi   = sinf(psi);

  q[0] = cphi*ctheta*cpsi + sphi*stheta*spsi;
  q[1] = sphi*ctheta*cpsi - cphi*stheta*spsi;
  q[2] = cphi*stheta*cpsi + sphi*ctheta*spsi;
  q[3] = cphi*ctheta*spsi - sphi*stheta*cpsi;
}

/**
 * @brief Find roll, pitch and yaw from quaternion.
 * @note  The order of rotations is:
 *        1. pitch (X);
 *        2. roll (Y);
 *        3. yaw (Z).
 */
static inline void Quaternion2RPY(const float q[4], float rpy[3]) {
  float R13, R11, R12, R23, R33;
  float q2s = q[2]*q[2];

  R11 = 1.0f - 2.0f * (q2s + q[3]*q[3]);
  R12 = 2.0f * (q[0]*q[3] + q[1]*q[2]);
  R13 = 2.0f * (q[0]*q[2] - q[1]*q[3]);
  R23 = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  R33 = 1.0f - 2.0f * (q2s + q[1]*q[1]);

  rpy[1] = asinf (R13);   // roll always between -pi/2 to pi/2
  rpy[2] = atan2f(R12, R11);
  rpy[0] = atan2f(R23, R33);

  //TODO: consider the cases where |R13| ~= 1, |roll| ~= pi/2
}

/**
 * @brief  Resets the CRC Data register (DR).
 * @param  None
 * @retval None
 */
static inline void crcResetDR(void) {
  /* Resets CRC generator. */
  CRC->CR = CRC_CR_RESET;
}

/**
 * @brief  Computes the 32-bit CRC of a given buffer of data word(32-bit).
 * @param  pBuf: pointer to the buffer containing the data to be computed
 * @param  length: length of the buffer to be computed
 * @retval 32-bit CRC
 */
static inline uint32_t crcCRC32(const uint32_t pBuf[], uint32_t length) {
  uint32_t i;
  /* Resets CRC generator. */
  CRC->CR = CRC_CR_RESET;
  /* Calculates CRC32 checksum. */
  for(i = 0; i < length; i++) {
    CRC->DR = pBuf[i];
  }
  return CRC->DR;
}

#ifdef __cplusplus
}
#endif

#endif /* _MISC_H_ */
