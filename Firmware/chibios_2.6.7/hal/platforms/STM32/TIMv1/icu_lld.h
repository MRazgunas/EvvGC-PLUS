/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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
   Modified by Emil Fresk and Sarunas Vaitekonis for extended
   input capture functionality.
 */

/**
 * @file    STM32/icu_lld.h
 * @brief   STM32 ICU subsystem low level driver header.
 *
 * @addtogroup ICU
 * @{
 */

#ifndef _ICU_LLD_H_
#define _ICU_LLD_H_

#include "stm32_tim.h"

#if HAL_USE_ICU || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Number of ICU channels per ICU driver.
 */
#define ICU_CHANNELS                            4

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   ICUD1 driver enable switch.
 * @details If set to @p TRUE the support for ICUD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ICU_USE_TIM1) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM1                  FALSE
#endif

/**
 * @brief   ICUD2 driver enable switch.
 * @details If set to @p TRUE the support for ICUD2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ICU_USE_TIM2) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM2                  FALSE
#endif

/**
 * @brief   ICUD3 driver enable switch.
 * @details If set to @p TRUE the support for ICUD3 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ICU_USE_TIM3) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM3                  FALSE
#endif

/**
 * @brief   ICUD4 driver enable switch.
 * @details If set to @p TRUE the support for ICUD4 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ICU_USE_TIM4) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM4                  FALSE
#endif

/**
 * @brief   ICUD5 driver enable switch.
 * @details If set to @p TRUE the support for ICUD5 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ICU_USE_TIM5) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM5                  FALSE
#endif

/**
 * @brief   ICUD8 driver enable switch.
 * @details If set to @p TRUE the support for ICUD8 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ICU_USE_TIM8) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM8                  FALSE
#endif

/**
 * @brief   ICUD9 driver enable switch.
 * @details If set to @p TRUE the support for ICUD9 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ICU_USE_TIM9) || defined(__DOXYGEN__)
#define STM32_ICU_USE_TIM9                  FALSE
#endif

/**
 * @brief   ICUD1 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM1_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD2 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM2_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD3 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM3_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD4 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM4_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD5 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM5_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD8 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM8_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM8_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD9 interrupt priority level setting.
 */
#if !defined(STM32_ICU_TIM9_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ICU_TIM9_IRQ_PRIORITY         7
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_ICU_USE_TIM1 && !STM32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM2 && !STM32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM3 && !STM32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM4 && !STM32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM5 && !STM32_HAS_TIM5
#error "TIM5 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM8 && !STM32_HAS_TIM8
#error "TIM8 not present in the selected device"
#endif

#if STM32_ICU_USE_TIM9 && !STM32_HAS_TIM9
#error "TIM9 not present in the selected device"
#endif

#if !STM32_ICU_USE_TIM1 && !STM32_ICU_USE_TIM2 &&                           \
    !STM32_ICU_USE_TIM3 && !STM32_ICU_USE_TIM4 &&                           \
    !STM32_ICU_USE_TIM5 && !STM32_ICU_USE_TIM8 &&                           \
    !STM32_ICU_USE_TIM9
#error "ICU driver activated but no TIM peripheral assigned"
#endif

#if STM32_ICU_USE_TIM1 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_ICU_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if STM32_ICU_USE_TIM2 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_ICU_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if STM32_ICU_USE_TIM3 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_ICU_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if STM32_ICU_USE_TIM4 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_ICU_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
#endif

#if STM32_ICU_USE_TIM5 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_ICU_TIM5_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM5"
#endif

#if STM32_ICU_USE_TIM8 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_ICU_TIM8_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM8"
#endif

#if STM32_ICU_USE_TIM9 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_ICU_TIM9_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM9"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ICU driver mode.
 */
typedef enum {
  ICU_INPUT_ACTIVE_HIGH = 0,        /**< Trigger on rising edge.            */
  ICU_INPUT_ACTIVE_LOW = 1,         /**< Trigger on falling edge.           */
  ICU_INPUT_DISABLED = 2,           /**< Input is disabled.                 */
} icumode_t;

/**
 * @brief   ICU frequency type.
 */
typedef uint32_t icufreq_t;

/**
 * @brief   ICU Input type.
 */
typedef enum {
  ICU_INPUT_TYPE_EDGE = 0,          /**< Triggers on the edge of the input.         */
  ICU_INPUT_TYPE_PULSE = 1,         /**< Triggers on detected pulse.                */
  ICU_INPUT_TYPE_PWM = 2,           /**< Triggers on detected PWM period and width. */
} icuinput_t;

/**
 * @brief   ICU counter type.
 */
typedef uint16_t icucnt_t;

/**
 * @brief   ICU diver channel configuration structure.
 */
typedef struct {
  /**
   * @brief   Specifies the active edge of the input signal.
   */
  icumode_t                 mode;
  /**
   * @brief   Capture event callback. Used for PWM width, pulse width and
   *          normal capture event.
   */
  icucallback_t             width_cb;
} ICUChannelConfig;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Select which input type the driver will be configured for.
   */
  icuinput_t                input;
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  icufreq_t                 frequency;
  /**
   * @brief   Pointer to each Input Capture channel configuration.
   * @note    A NULL parameter indicates the channel as unused.
   * @note    In PWM mode, only Channel 1 OR Channel 2 may be used.
   */
  ICUChannelConfig          channels[ICU_CHANNELS];
  /**
   * @brief   Callback for cycle period measurement.
   */
  icucallback_t             period_cb;
  /**
   * @brief   Callback for timer overflow.
   */
  icucallback_t             overflow_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief TIM DIER register initialization data.
   * @note  The value of this field should normally be equal to zero.
   * @note  Only the DMA-related bits can be specified in this field.
   */
  uint32_t                  dier;
} ICUConfig;

/**
 * @brief   Structure representing an ICU driver.
 */
struct ICUDriver {
  /**
   * @brief Driver state.
   */
  icustate_t                state;
  /**
   * @brief Current configuration data.
   */
  const ICUConfig           *config;
#if defined(ICU_DRIVER_EXT_FIELDS)
  ICU_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Timer base clock.
   */
  uint32_t                  clock;
  /**
   * @brief Temporary width holder during pulse measurement.
   */
  icucnt_t                  last_count[4];
  /**
   * @brief Pointer to the TIMx registers block.
   */
  stm32_tim_t               *tim;
  /**
   * @brief CCR registers used for width capture.
   */
  volatile uint32_t         *wccrp[4];
  /**
   * @brief CCR register used for period capture.
   * @note  Only one is needed since only one PWM input per timer is allowed.
   */
  volatile uint32_t         *pccrp;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the compare value of the latest pulse.
 * @details The compare value is defined as number of ticks between the start
 *          edge and the stop edge.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 * @return              The number of ticks.
 *
 * @notapi
 */
#define icu_lld_get_compare(icup, channel) (*((icup)->wccrp[(channel)]) + 1)

/**
 * @brief   Returns the width of the latest cycle.
 * @details The cycle width is defined as number of ticks between a start
 *          edge and the next start edge.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The number of ticks.
 *
 * @notapi
 */
#define icu_lld_get_period(icup) (*((icup)->pccrp) + 1)

/**
 * @brief   Inverts the polarity for the given channel.
 *
 * @param[in] icup      pointer to the @p ICUDriver object.
 * @param[in] channel   The timer channel to invert.
 *
 * @notapi
 */
#define icu_lld_invert_polarity(icup, channel) \
  (icup)->tim->CCER ^= STM32_TIM_CCER_CC1P << ((channel) << 2)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_ICU_USE_TIM1 && !defined(__DOXYGEN__)
extern ICUDriver ICUD1;
#endif

#if STM32_ICU_USE_TIM2 && !defined(__DOXYGEN__)
extern ICUDriver ICUD2;
#endif

#if STM32_ICU_USE_TIM3 && !defined(__DOXYGEN__)
extern ICUDriver ICUD3;
#endif

#if STM32_ICU_USE_TIM4 && !defined(__DOXYGEN__)
extern ICUDriver ICUD4;
#endif

#if STM32_ICU_USE_TIM5 && !defined(__DOXYGEN__)
extern ICUDriver ICUD5;
#endif

#if STM32_ICU_USE_TIM8 && !defined(__DOXYGEN__)
extern ICUDriver ICUD8;
#endif

#if STM32_ICU_USE_TIM9 && !defined(__DOXYGEN__)
extern ICUDriver ICUD9;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void icu_lld_init(void);
  void icu_lld_start(ICUDriver *icup);
  void icu_lld_stop(ICUDriver *icup);
  void icu_lld_enable(ICUDriver *icup);
  void icu_lld_disable(ICUDriver *icup);
  icucnt_t icu_lld_get_width(ICUDriver *icup, icuchannel_t channel);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ICU */

#endif /* _ICU_LLD_H_ */

/** @} */
