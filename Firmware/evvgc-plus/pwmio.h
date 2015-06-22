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

#ifndef _PWMIO_H_
#define _PWMIO_H_

/**
 * Output channels.
 */
#define PWM_OUT_PITCH           0x00
#define PWM_OUT_ROLL            0x01
#define PWM_OUT_YAW             0x02

/**
 * Output command IDs.
 */
#define PWM_OUT_CMD_PITCH       0x00
#define PWM_OUT_CMD_ROLL        0x01
#define PWM_OUT_CMD_YAW         0x02
#define PWM_OUT_CMD_DISABLED    0x03

/**
 * Dead-time IDs.
 */
#define PWM_OUT_DT750NS         0x00
#define PWM_OUT_DT1000NS        0x10
#define PWM_OUT_DT2000NS        0x20
#define PWM_OUT_DT3000NS        0x30
#define PWM_OUT_DT4000NS        0x40
#define PWM_OUT_DT5000NS        0x50

/**
 * Flags.
 */
#define PWM_OUT_REV_FLAG        0x01
#define PWM_OUT_THI_FLAG        0x02

/**
 * Masks for high and low nibbles of data.
 */
#define PWM_OUT_CMD_ID_MASK     0x0F
#define PWM_OUT_DT_ID_MASK      0xF0

/**
 * Input channel IDs.
 */
#define INPUT_CHANNEL_AUX1      0x00
#define INPUT_CHANNEL_AUX2      0x01
#define INPUT_CHANNEL_AUX3      0x02
#define INPUT_CHANNEL_AUX4      0x03
#define INPUT_CHANNEL_AUX5      0x04
#define INPUT_CHANNEL_DISABLED  0x05

#include <AP_Param.h>

class PWMOutput {
public:
   PWMOutput(){
      AP_Param::setup_object_defaults(this, var_info);
   }

   static const AP_Param::GroupInfo var_info[];

   AP_Int8 power;
   AP_Int8 num_poles;
   AP_Int8 flags;
   AP_Int8 dt_cmd_id;
};

class MixedInput {
public:
   MixedInput() {
      AP_Param::setup_object_defaults(this, var_info);
   }

   static const AP_Param::GroupInfo var_info[];

   AP_Int16 min_val;
   AP_Int16 mid_val;
   AP_Int16 max_val;
   AP_Int8 channel_id;
};

typedef struct tagMixedInputStruct {
  int16_t min_val;
  int16_t mid_val;
  int16_t max_val;
  uint8_t channel_id;
} __attribute__((packed)) MixedInputStruct, *PMixedInputStruct;

typedef struct tagPWMOutputStruct {
  uint8_t power;
  uint8_t num_poles;
  uint8_t flags;
  uint8_t dt_cmd_id; /* High nibble contains dead-time ID, low nibble contains command ID. */
} __attribute__((packed)) PWMOutputStruct, *PPWMOutputStruct;

extern PWMOutput g_pwmOutput[3];
extern MixedInput g_mixedInput[3];
extern int16_t g_inputValues[5];

#ifdef __cplusplus
extern "C" {
#endif
  void pwmOutputStart(void);
  void pwmOutputStop(void);
  void pwmOutputUpdate(const uint8_t channel_id, float cmd);
  void pwmOutputDisableAll(void);
  void pwmOutputSettingsUpdate(const PPWMOutputStruct pNewSettings);
  void mixedInputStart(void);
  void mixedInputStop(void);
  void mixedInputSettingsUpdate(const PMixedInputStruct  pNewSettings);
#ifdef __cplusplus
}
#endif

#endif /* _PWMIO_H_ */
