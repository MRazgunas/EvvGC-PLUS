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

#ifndef ATTITUDE_H_
#define ATTITUDE_H_

#include "mpu6050.h"

#define INPUT_MODE_ANGLE      0x00
#define INPUT_MODE_SPEED      0x01
#define INPUT_MODE_FOLLOW     0x02

typedef struct tagPIDSettings {
  uint8_t P;
  uint8_t I;
  uint8_t D;
} __attribute__((packed)) PIDSettings, *PPIDSettings;

typedef struct tagInputModeStruct {
  int16_t min_angle;
  int16_t max_angle;
  int16_t offset;
  uint8_t speed;
  uint8_t mode_id;
} __attribute__((packed)) InputModeStruct, *PInputModeStruct;

extern float g_motorOffset[3];
extern PIDSettings g_pidSettings[3];
extern InputModeStruct g_modeSettings[3];

#ifdef __cplusplus
extern "C" {
#endif
  void attitudeInit(void);
  void attitudeUpdate(PIMUStruct pIMU);
  void cameraRotationUpdate(void);
  void actuatorsUpdate(void);
  void pidSettingsUpdate(const PPIDSettings pNewSettings);
  void inputModeSettingsUpdate(const PInputModeStruct pNewSettings);
#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H_ */
