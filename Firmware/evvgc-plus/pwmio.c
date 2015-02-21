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
#include "pwmio.h"

#include <math.h>
#include <string.h>

/**
 * Compiler directives
 */
/* Uncomment this line to activate third harmonic injection PWM.
 * This technique increases motor driving efficiency by almost 14%.
 * Disabled by default.
 */
//#define USE_THI_PWM

/**
 * DeadTime range (us) = (0..127) * 1 / 72;
 */
#define BDTR_DTG_MUL1   0x00
#define BDTR_DTG_MSK1   0x7F
/**
 * DeadTime range (us) = (64 + 0..63) * 2 / 72;
 */
#define BDTR_DTG_MUL2   0x80
#define BDTR_DTG_MSK2   0x3F
/**
 * DeadTime range (us) = (32 + 0..15) * 8 / 72;
 */
#define BDTR_DTG_MUL8   0xC0
#define BDTR_DTG_MSK8   0x1F
/**
 * DeadTime range (us) = (32 + 0..15) * 16 / 72;
 */
#define BDTR_DTG_MUL16  0xE0
#define BDTR_DTG_MSK16  0x1F

/* DT = 36 * 2 / 72 = 1us */
#define PWM_OUT_TIM4_5_DT_1US   0x24
/* DT = 72 * 2 / 72 = 2us */
#define PWM_OUT_TIM4_5_DT_2US   0x48
/* DT = 108 * 2 / 72 = 3us */
#define PWM_OUT_TIM4_5_DT_3US   0x6C
/* DT = 144 * 2 / 72 = 4us */
#define PWM_OUT_TIM4_5_DT_4US   0x90
/* DT = 180 * 2 / 72 = 5us */
#define PWM_OUT_TIM4_5_DT_5US   0xB4
/* DT = 216 * 2 / 72 = 6us */
#define PWM_OUT_TIM4_5_DT_6US   0xD8
/* DT = 252 * 2 / 72 = 7us */
#define PWM_OUT_TIM4_5_DT_7US   0xFC

/**
 * PWM value for the half percent of the total power, given:
 * - PWM clock frequency = 72 MHz;
 * - PWM period = 1/18000 s;
 */
#define PWM_OUT_POWER_1PCT2     0x0A

/**
 * ADC related constants.
 */
#define ADC_GRP_NUM_CHANNELS    0x02
#define ADC_GRP_BUF_DEPTH       0x20

/**
 * Separation angle between phases.
 */
#ifndef M_2PI_3
#define M_2PI_3         (2.0f * M_PI / 3.0f)
#endif

/**
 * Amplitude scaling factor for third harmonic injection PWM.
 */
#ifdef USE_THI_PWM
#define THI_PWM_K       (2.0f / sqrtf(3.0f))
#endif /* USE_THI_PWM */

/**
 * Local macros for dead time calculation.
 */
#define constrainLeft(val,left)     ((val)<(left)?(left):(val))
#define constrainRight(val,right)   ((val)>(right)?(right):(val))

/**
 * Forward declarations:
 */
/* Callback function for ADC conversions. */
static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
/* Callback function for ICU width calculation. */
static void icuwidthcb(ICUDriver *icup, icuchannel_t channel);
/* Callback function for ICU period calculation. */
static void icuperiodcb(ICUDriver *icup, icuchannel_t channel);

/**
 * Default settings for PWM outputs.
 */
PWMOutputStruct g_pwmOutput[3] = {
  {0,                      /* Motor power;       */
   14,                     /* Number of poles;   */
   0,                      /* Reverse flag;      */
   PWM_OUT_CMD_DISABLED,   /* Output command ID; */
   PWM_OUT_DT5000NS},      /* Dead-time ID;      */
  {0,                      /* Motor power;       */
   14,                     /* Number of poles;   */
   0,                      /* Reverse flag;      */
   PWM_OUT_CMD_DISABLED,   /* Output command ID; */
   PWM_OUT_DT5000NS},      /* Dead-time ID;      */
  {0,                      /* Motor power;       */
   14,                     /* Number of poles;   */
   0,                      /* Reverse flag;      */
   PWM_OUT_CMD_DISABLED,   /* Output command ID; */
   PWM_OUT_DT5000NS}       /* Dead-time ID;      */
};

/**
 * Default settings for generic inputs.
 */
MixedInputStruct g_mixedInput[3] = {
  {0,                      /* Min value;      */
   0,                      /* Mid value;      */
   0,                      /* Max value;      */
   INPUT_CHANNEL_DISABLED},/* Input channel#; */
  {0,                      /* Min value;      */
   0,                      /* Mid value;      */
   0,                      /* Max value;      */
   INPUT_CHANNEL_DISABLED},/* Input channel#; */
  {0,                      /* Min value;      */
   0,                      /* Mid value;      */
   0,                      /* Max value;      */
   INPUT_CHANNEL_DISABLED} /* Input channel#; */
};

/**
 * Values of the input channels.
 */
int16_t g_inputValues[5] = {0};

/**
 * PWM configuration structure for TIM1 and TIM8 output.
 */
static PWMConfig pwmcfg_d1_d8 = {
  72000000, /* PWM clock frequency (72 MHz). */
  2000,     /* PWM period (1/18000 s) in ticks
               for center-aligned mode.      */
  NULL,     /* Callback disabled.            */
  {         /* PWM channel configuration:    */
    {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL}, /* CH1 */
    {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL}, /* CH2 */
    {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL}, /* CH3 */
    {PWM_OUTPUT_DISABLED,                                           NULL}  /* CH4 */
  },
  0, /* CR2 register value. */
#if STM32_PWM_USE_ADVANCED
  /* BDTR register value: */
  STM32_TIM_BDTR_LOCK(1) | /* LOCK Level 1: DTG bits in TIMx_BDTR
                              register, OISx and OISxN bits in TIMx_CR2
                              register and BKE/BKP/AOE bits in TIMx_BDTR
                              register can no longer be written.        */
  STM32_TIM_BDTR_OSSI |    /* Off-state selection for Idle mode enable. */
  STM32_TIM_BDTR_OSSR |    /* Off-state selection for Run mode enable.  */
  STM32_TIM_BDTR_AOE,      /* Automatic output enable.                  */
#endif
  0  /* DIER register value. */
};

/**
 * PWM configuration structure for TIM4 output.
 */
static const PWMConfig pwmcfg_d4 = {
  72000000, /* PWM clock frequency (72 MHz). */
  2000,     /* PWM period (1/18000 s) in ticks
               for center-aligned mode.      */
  NULL,     /* Callback disabled.            */
  {         /* PWM channel configuration:    */
    {PWM_OUTPUT_ACTIVE_LOW, NULL},  /* CH1 */
    {PWM_OUTPUT_ACTIVE_LOW, NULL},  /* CH2 */
    {PWM_OUTPUT_ACTIVE_LOW, NULL},  /* CH3 */
    {PWM_OUTPUT_DISABLED,   NULL}   /* CH4 */
  },
  0, /* CR2 register value. */
#if STM32_PWM_USE_ADVANCED
  0, /* BDTR register value. Not used for TIM4. */
#endif
  0  /* DIER register value. */
};

/**
 * PWM configuration structure for TIM5 output.
 */
static const PWMConfig pwmcfg_d5 = {
  72000000, /* PWM clock frequency (72 MHz). */
  2000,     /* PWM period (1/18000 s) in ticks
               for center-aligned mode.      */
  NULL,     /* Callback disabled.            */
  {         /* PWM channel configuration:    */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* CH1 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* CH2 */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* CH3 */
    {PWM_OUTPUT_DISABLED,    NULL}  /* CH4 */
  },
  0, /* CR2 register value. */
#if STM32_PWM_USE_ADVANCED
  0, /* BDTR register value. Not used for TIM5. */
#endif
  0  /* DIER register value. */
};

/**
 * ADC conversion group.
 * Mode:     Continuous, circular, 32 samples of 2 channels.
 * Channels: IN12, IN13.
 */
static const ADCConversionGroup adcgrpcfg = {
  TRUE,                 /* Circular buffer mode enabled.    */
  ADC_GRP_NUM_CHANNELS, /* Number of channels in the group. */
  adccb,                /* Callback function of the group.  */
  NULL,                 /* Error callback function.         */
  /* STM32F1xx dependent part: */
  0,                                      /* CR1.   */
  0,                                      /* CR2.   */                        
  ADC_SMPR1_SMP_AN12(ADC_SAMPLE_239P5) |
  ADC_SMPR1_SMP_AN13(ADC_SAMPLE_239P5),   /* SMPR1. */
  0,                                      /* SMPR2. */
  ADC_SQR1_NUM_CH(ADC_GRP_NUM_CHANNELS),  /* SQR1.  */
  0,                                      /* SQR2.  */
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN13) |
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN12)        /* SQR3.  */
};

/**
 * Input capture configuration for ICU2 driver.
 *
 * @note ICU drivers used in the firmware are modified ChibiOS
 *       drivers for extended input capture functionality.
 */
static const ICUConfig icucfg2 = {
  ICU_INPUT_TYPE_PWM,       /* Driver input type (EDGE, PULSE, PWM).  */
  1000000,                  /* 1MHz ICU clock frequency.              */
  {                         /* ICU channel configuration.             */
    {ICU_INPUT_DISABLED, NULL},           /* CH1 */
    {ICU_INPUT_ACTIVE_HIGH, icuwidthcb},  /* CH2 */
    {ICU_INPUT_DISABLED, NULL},           /* CH3 */
    {ICU_INPUT_DISABLED, NULL}            /* CH4 */
  },
  icuperiodcb,              /* Callback for cycle period measurement. */
  NULL,                     /* Callback for timer overflow.           */
  0                         /* DIER.                                  */
};

/**
 * Input capture configuration for ICU3 driver.
 *
 * @note ICU drivers used in the firmware are modified ChibiOS
 *       drivers for extended input capture functionality.
 */
static const ICUConfig icucfg3 = {
  ICU_INPUT_TYPE_PULSE,     /* Driver input type (EDGE, PULSE, PWM).  */
  1000000,                  /* 1MHz ICU clock frequency.              */
  {                         /* ICU channel configuration.             */
    {ICU_INPUT_ACTIVE_HIGH, icuwidthcb},  /* CH1 */
    {ICU_INPUT_ACTIVE_HIGH, icuwidthcb},  /* CH2 */
    {ICU_INPUT_DISABLED, NULL},           /* CH3 */
    {ICU_INPUT_DISABLED, NULL}            /* CH4 */
  },
  NULL,                     /* Callback for cycle period measurement. */
  NULL,                     /* Callback for timer overflow.           */
  0                         /* DIER.                                  */
};

/**
 * Local dead-time value for TIM4-TIM5 complementary output.
 */
static uint32_t pwmOutTIM4_5_DT = PWM_OUT_TIM4_5_DT_5US;

/**
 * Local PWM output values for three phase BLDC motor driver.
 */
static uint32_t pwm3PhaseDrv[3];

/**
 * Local buffer for ADC conversions.
 */
static adcsample_t adcBuf[ADC_GRP_NUM_CHANNELS * ADC_GRP_BUF_DEPTH];

/**
 * @brief  ADC streaming callback.
 * @param  adcp - pointer to the ADCDriver object triggering the callback.
 * @param  buffer - pointer to the most recent samples data.
 * @param  n - number of buffer rows available starting from buffer.
 * @return none.
 */
static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;
  uint32_t sum1 = 0;
  uint32_t sum2 = 0;
  size_t i;
  for(i = 0; i < n; i++) {
    sum1 += *buffer++;
    sum2 += *buffer++;
  }
  g_inputValues[INPUT_CHANNEL_AUX1] = (adcsample_t)(sum1 / n);
  g_inputValues[INPUT_CHANNEL_AUX2] = (adcsample_t)(sum2 / n);
}

/**
 * @brief  Callback function for input capture unit.
 * @param  icup - pointer to the input capture driver.
 * @param  channel - input capture channel triggering the callback.
 * @return none.
 */
static void icuwidthcb(ICUDriver *icup, icuchannel_t channel) {
  if (&ICUD2 == icup) {
    g_inputValues[INPUT_CHANNEL_AUX3] = icuGetWidth(icup, channel);
  } else {
    g_inputValues[INPUT_CHANNEL_AUX4 + channel] = icuGetWidth(icup, channel);
  }
}

/**
 * @brief  Callback function for input capture unit.
 * @param  icup - pointer to the input capture driver.
 * @param  channel - input capture channel triggering the callback.
 * @return none.
 */
static void icuperiodcb(ICUDriver *icup, icuchannel_t channel) {
  (void)icup;
  (void)channel;
}

/**
 * @brief  Calculates value for DTG (dead-time generator) bits of BDTR register.
 * @param  id - dead-time value ID.
 * @return value for DTG bits.
 */
#if STM32_PWM_USE_ADVANCED
static uint32_t pwmOutputGetBDTRDeadTime(const uint8_t id) {
  uint32_t bdtr_dtg;
  switch (id) {
  case PWM_OUT_DT1000NS: /* DT = 72 / 72 = 1us */
    bdtr_dtg = STM32_TIM_BDTR_DTG(BDTR_DTG_MUL1 | (BDTR_DTG_MSK1 & 0x48));
    break;
  case PWM_OUT_DT2000NS: /* DT = (64 + 8) * 2 / 72 = 2us */
    bdtr_dtg = STM32_TIM_BDTR_DTG(BDTR_DTG_MUL2 | (BDTR_DTG_MSK2 & 0x08));
    break;
  case PWM_OUT_DT3000NS: /* DT = (64 + 44) * 2 / 72 = 3us */
    bdtr_dtg = STM32_TIM_BDTR_DTG(BDTR_DTG_MUL2 | (BDTR_DTG_MSK2 & 0x2C));
    break;
  case PWM_OUT_DT4000NS: /* DT = (32 + 4) * 8 / 72 = 4us */
    bdtr_dtg = STM32_TIM_BDTR_DTG(BDTR_DTG_MUL8 | (BDTR_DTG_MSK8 & 0x04));
    break;
  case PWM_OUT_DT5000NS: /* DT = (32 + 13) * 8 / 72 = 5us */
    bdtr_dtg = STM32_TIM_BDTR_DTG(BDTR_DTG_MUL8 | (BDTR_DTG_MSK8 & 0x0D));
    break;
  default: /* Apply the longest dead time. DT = (32 + 13) * 8 / 72 = 5us */
    bdtr_dtg = STM32_TIM_BDTR_DTG(BDTR_DTG_MUL8 | (BDTR_DTG_MSK8 & 0x0D));
  }
  return bdtr_dtg;
}
#endif

/**
 * @brief  Disables PWM on roll driver.
 * @return none.
 */
static void pwmOutputDisableRoll(void) {
  memset((void *)PWMD8.tim->CCR, 0, sizeof(uint32_t) * 3);
}

/**
 * @brief  Disables PWM on pitch driver.
 * @return none.
 */
static void pwmOutputDisablePitch(void) {
  memset((void *)PWMD1.tim->CCR, 0, sizeof(uint32_t) * 3);
}

/**
 * @brief  Disables PWM on yaw driver.
 * @return none.
 */
static void pwmOutputDisableYaw(void) {
  /* Make atomic writing; */
  chSysLock();
  memset((void *)PWMD5.tim->CCR, 0, sizeof(uint32_t) * 3);
  memset((void *)PWMD4.tim->CCR, 0, sizeof(uint32_t) * 3);
  chSysUnlock();
}

/**
 * @brief
 * @return none.
 */
static void pwmOutputCmdTo3PhasePWM(float cmd, uint8_t power) {
#if defined(USE_THI_PWM)
  float thirdHarmonic = sinf(cmd * 3.0f) / 6.0f;
  float halfPower = THI_PWM_K * power * PWM_OUT_POWER_1PCT2;
  pwm3PhaseDrv[0] = (1.0 + sinf(cmd) + thirdHarmonic) * halfPower;
  pwm3PhaseDrv[1] = (1.0 + sinf(cmd + M_2PI_3) + thirdHarmonic) * halfPower;
  pwm3PhaseDrv[2] = (1.0 + sinf(cmd - M_2PI_3) + thirdHarmonic) * halfPower;
#else
  float halfPower = power * PWM_OUT_POWER_1PCT2;
  pwm3PhaseDrv[0] = (1.0 + sinf(cmd)) * halfPower;
  pwm3PhaseDrv[1] = (1.0 + sinf(cmd + M_2PI_3)) * halfPower;
  pwm3PhaseDrv[2] = (1.0 + sinf(cmd - M_2PI_3)) * halfPower;
#endif /* USE_THI_PWM */
}

/**
 *
 */
static void pwmOutputUpdateRoll(void) {
  if (g_pwmOutput[PWM_OUT_ROLL].reverse) {
    PWMD8.tim->CCR[0] = pwm3PhaseDrv[1];
    PWMD8.tim->CCR[1] = pwm3PhaseDrv[0];
  } else {
    PWMD8.tim->CCR[0] = pwm3PhaseDrv[0];
    PWMD8.tim->CCR[1] = pwm3PhaseDrv[1];
  }
  PWMD8.tim->CCR[2] = pwm3PhaseDrv[2];
}

/**
 *
 */
static void pwmOutputUpdatePitch(void) {
  if (g_pwmOutput[PWM_OUT_PITCH].reverse) {
    PWMD1.tim->CCR[0] = pwm3PhaseDrv[1];
    PWMD1.tim->CCR[1] = pwm3PhaseDrv[0];
  } else {
    PWMD1.tim->CCR[0] = pwm3PhaseDrv[0];
    PWMD1.tim->CCR[1] = pwm3PhaseDrv[1];
  }
  PWMD1.tim->CCR[2] = pwm3PhaseDrv[2];
}

/**
 *
 */
static void pwmOutputUpdateYaw(void) {
  uint32_t pwmDrvYaw[6];
  /* Apply dead-time to Yaw PWM: */
  if (g_pwmOutput[PWM_OUT_YAW].reverse) {
    pwmDrvYaw[0] = constrainLeft (pwm3PhaseDrv[1], pwmOutTIM4_5_DT) - pwmOutTIM4_5_DT; /* Yaw_B  */
    pwmDrvYaw[1] = constrainLeft (pwm3PhaseDrv[0], pwmOutTIM4_5_DT) - pwmOutTIM4_5_DT; /* YAW_A  */
    pwmDrvYaw[3] = constrainRight(pwm3PhaseDrv[1], PWM_OUT_POWER_1PCT2 * 200 -
      pwmOutTIM4_5_DT - 1) + pwmOutTIM4_5_DT;                                          /* Yaw_BN */
    pwmDrvYaw[4] = constrainRight(pwm3PhaseDrv[0], PWM_OUT_POWER_1PCT2 * 200 -
      pwmOutTIM4_5_DT - 1) + pwmOutTIM4_5_DT;                                          /* YAW_AN */
  } else {
    pwmDrvYaw[0] = constrainLeft (pwm3PhaseDrv[0], pwmOutTIM4_5_DT) - pwmOutTIM4_5_DT; /* Yaw_A  */
    pwmDrvYaw[1] = constrainLeft (pwm3PhaseDrv[1], pwmOutTIM4_5_DT) - pwmOutTIM4_5_DT; /* YAW_B  */
    pwmDrvYaw[3] = constrainRight(pwm3PhaseDrv[0], PWM_OUT_POWER_1PCT2 * 200 -
      pwmOutTIM4_5_DT - 1) + pwmOutTIM4_5_DT;                                          /* Yaw_AN */
    pwmDrvYaw[4] = constrainRight(pwm3PhaseDrv[1], PWM_OUT_POWER_1PCT2 * 200 -
      pwmOutTIM4_5_DT - 1) + pwmOutTIM4_5_DT;                                          /* YAW_BN */
  }
  pwmDrvYaw[2] = constrainLeft (pwm3PhaseDrv[2], pwmOutTIM4_5_DT) - pwmOutTIM4_5_DT;   /* YAW_C  */
  pwmDrvYaw[5] = constrainRight(pwm3PhaseDrv[2], PWM_OUT_POWER_1PCT2 * 200 -
    pwmOutTIM4_5_DT - 1) + pwmOutTIM4_5_DT;                                            /* YAW_CN */
  /* Make atomic writing; */
  chSysLock();
  memcpy((void *)PWMD5.tim->CCR, (void *)&pwmDrvYaw[0], sizeof(pwmDrvYaw) / 2);
  memcpy((void *)PWMD4.tim->CCR, (void *)&pwmDrvYaw[3], sizeof(pwmDrvYaw) / 2);
  chSysUnlock();
}

/**
 * @brief  Starts the PWM output.
 * @note   The pwmStart() function used in this code is not
 *         the original ChibiOS HAL function, but modified
 *         one with STM32_TIM_CR1_CEN flag removed.
 * @return none.
 */
void pwmOutputStart(void) {
#if STM32_PWM_USE_ADVANCED
  /* Get dead-time generator value for TIM1. */
  uint32_t bdtr_dt = pwmOutputGetBDTRDeadTime(g_pwmOutput[PWM_OUT_ROLL].dt_id);
  pwmcfg_d1_d8.bdtr |= bdtr_dt;
#endif
  pwmStart(&PWMD1, &pwmcfg_d1_d8);

#if STM32_PWM_USE_ADVANCED
  /* Clear bdtr_dt value from previous calculation. */
  pwmcfg_d1_d8.bdtr &= ~bdtr_dt;
  /* Get dead-time generator value for TIM8. */
  bdtr_dt = pwmOutputGetBDTRDeadTime(g_pwmOutput[PWM_OUT_PITCH].dt_id);
  pwmcfg_d1_d8.bdtr |= bdtr_dt;
#endif
  pwmStart(&PWMD8, &pwmcfg_d1_d8);

  switch (g_pwmOutput[PWM_OUT_YAW].dt_id) {
  case PWM_OUT_DT1000NS:
    pwmOutTIM4_5_DT = PWM_OUT_TIM4_5_DT_1US;
    break;
  case PWM_OUT_DT2000NS:
    pwmOutTIM4_5_DT = PWM_OUT_TIM4_5_DT_2US;
    break;
  case PWM_OUT_DT3000NS:
    pwmOutTIM4_5_DT = PWM_OUT_TIM4_5_DT_3US;
    break;
  case PWM_OUT_DT4000NS:
    pwmOutTIM4_5_DT = PWM_OUT_TIM4_5_DT_4US;
    break;
  case PWM_OUT_DT5000NS:
    pwmOutTIM4_5_DT = PWM_OUT_TIM4_5_DT_5US;
    break;
  default:
    pwmOutTIM4_5_DT = PWM_OUT_TIM4_5_DT_5US;
  }
  pwmStart(&PWMD4, &pwmcfg_d4);
  pwmStart(&PWMD5, &pwmcfg_d5);

  /* Switch to center-aligned mode 1 and start timers. */
  PWMD1.tim->CR1 |= (STM32_TIM_CR1_CMS(1) | STM32_TIM_CR1_CEN);
  PWMD8.tim->CR1 |= (STM32_TIM_CR1_CMS(1) | STM32_TIM_CR1_CEN);
  PWMD4.tim->CR1 |= (STM32_TIM_CR1_CMS(1) | STM32_TIM_CR1_CEN);
  PWMD5.tim->CR1 |= (STM32_TIM_CR1_CMS(1) | STM32_TIM_CR1_CEN);
}

/**
 * @brief  Updates specified PWM output channel driver state
 *         according to the given command.
 * @param  channel_id - PWM output channel ID.
 * @param  cmd - new command to the motor driver.
 * @return none.
 */
void pwmOutputUpdate(const uint8_t channel_id, float cmd) {
  switch (channel_id) {
  case PWM_OUT_PITCH:
    if (g_pwmOutput[PWM_OUT_PITCH].cmd_id == PWM_OUT_CMD_DISABLED) {
      pwmOutputDisablePitch();
    } else {
      pwmOutputCmdTo3PhasePWM(cmd, g_pwmOutput[PWM_OUT_PITCH].power);
      pwmOutputUpdatePitch();
    }
    break;
  case PWM_OUT_ROLL:
    if (g_pwmOutput[PWM_OUT_ROLL].cmd_id == PWM_OUT_CMD_DISABLED) {
      pwmOutputDisableRoll();
    } else {
      pwmOutputCmdTo3PhasePWM(cmd, g_pwmOutput[PWM_OUT_ROLL].power);
      pwmOutputUpdateRoll();
    }
    break;
  case PWM_OUT_YAW:
    if (g_pwmOutput[PWM_OUT_YAW].cmd_id == PWM_OUT_CMD_DISABLED) {
      pwmOutputDisableYaw();
    } else {
      pwmOutputCmdTo3PhasePWM(cmd, g_pwmOutput[PWM_OUT_YAW].power);
      pwmOutputUpdateYaw();
    }
    break;
  default:;
  }
}

/**
 *
 */
void pwmOutputSettingsUpdate(const PPWMOutputStruct pNewSettings) {
  memcpy((void *)&g_pwmOutput, (void *)pNewSettings, sizeof(g_pwmOutput));
}

/**
 * @brief  Starts the ADC and ICU input drivers.
 * @note   ICU drivers used in the firmware are modified ChibiOS
 *         drivers for extended input capture functionality.
 * @return none.
 */
void mixedInputStart(void) {
  /* Activates the ICU2 and ICU3 drivers. */
  icuStart(&ICUD2, &icucfg2);
  icuStart(&ICUD3, &icucfg3);
  /* Starts continuous pulse width measurements. */
  icuEnable(&ICUD2);
  icuEnable(&ICUD3);

  /* Activates the ADC1 driver. */
  adcStart(&ADCD1, NULL);
  /* Starts an ADC continuous conversion. */
  adcStartConversion(&ADCD1, &adcgrpcfg, adcBuf, ADC_GRP_BUF_DEPTH);
}

/**
 *
 */
void mixedInputSettingsUpdate(const PMixedInputStruct pNewSettings) {
  memcpy((void *)&g_mixedInput, (void *)pNewSettings, sizeof(g_mixedInput));
}
