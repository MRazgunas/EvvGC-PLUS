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

#ifndef PWMIO_H_
#define PWMIO_H_

#define PWM_OUT_PITCH           0x00
#define PWM_OUT_ROLL            0x01
#define PWM_OUT_YAW             0x02

#define PWM_OUT_CMD_PITCH       0x00
#define PWM_OUT_CMD_ROLL        0x01
#define PWM_OUT_CMD_YAW         0x02
#define PWM_OUT_CMD_DISABLED    0x03

#define PWM_OUT_DT1000NS        0x00
#define PWM_OUT_DT2000NS        0x01
#define PWM_OUT_DT3000NS        0x02
#define PWM_OUT_DT4000NS        0x03
#define PWM_OUT_DT5000NS        0x04

#define INPUT_CHANNEL_AUX1      0x00
#define INPUT_CHANNEL_AUX2      0x01
#define INPUT_CHANNEL_AUX3      0x02
#define INPUT_CHANNEL_AUX4      0x03
#define INPUT_CHANNEL_AUX5      0x04
#define INPUT_CHANNEL_DISABLED  0x05

typedef struct tagPWMOutputStruct {
  uint8_t power;
  uint8_t num_poles;
  uint8_t reverse;
  uint8_t cmd_id;
  uint8_t dt_id;
} __attribute__((packed)) PWMOutputStruct, *PPWMOutputStruct;

typedef struct tagMixedInputStruct {
  int16_t min_val;
  int16_t mid_val;
  int16_t max_val;
  uint8_t channel_id;
} __attribute__((packed)) MixedInputStruct, *PMixedInputStruct;

extern PWMOutputStruct g_pwmOutput[3];
extern MixedInputStruct g_mixedInput[3];
extern int16_t g_inputValues[5];

#ifdef __cplusplus
extern "C" {
#endif
  void pwmOutputStart(void);
  void pwmOutputUpdate(const uint8_t channel_id, float cmd);
  void pwmOutputSettingsUpdate(const PPWMOutputStruct pNewSettings);
  void mixedInputStart(void);
  void mixedInputSettingsUpdate(const PMixedInputStruct pNewSettings);
#ifdef __cplusplus
}
#endif

#endif /* PWMIO_H_ */
