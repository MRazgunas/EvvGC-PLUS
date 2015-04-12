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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the EvvGC v1.x board.
 */

/*
 * Board identifier.
 */
#define BOARD_EVVGC_V1_X
#define BOARD_NAME              "EvvGC v1.x"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./chibios_2.6.7/hal/platforms/hal_lld.h.
 */
#define STM32F10X_HD

/*
 * IO pins assignments.
 */
#define GPIOA_USB_DP            12
#define GPIOB_LED_A             12 // Red LED on v1.2, blue LED on v1.3.
#define GPIOA_LED_B             3  // Red LED on v1.3 only.


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA0  - Alternate Push Pull output (TIM5 CH1).
 * PA1  - Alternate Push Pull output (TIM5 CH2).
 * PA2  - Alternate Push Pull output (TIM5 CH3).
 * PA3  - Push Pull output (LED_B, v1.3 only).
 * PA7  - Alternate Push Pull output (TIM8 CH1N).
 * PA8  - Alternate Push Pull output (TIM1 CH1).
 * PA9  - Alternate Push Pull output (TIM1 CH2).
 * PA10 - Alternate Push Pull output (TIM1 CH3).
 * PA11 - Normal input (USB DM).
 * PA12 - Normal input (USB DP).
 */
#define VAL_GPIOACRL            0xA8882AAA      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x88844AAA      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB0  - Alternate Push Pull output (TIM8 CH2N).
 * PB1  - Alternate Push Pull output (TIM8 CH3N).
 * PB3  - Normal input (TIM2 CH2 - AUX3).
 * PB4  - Normal input (TIM3 CH1 - AUX4).
 * PB5  - Normal input (TIM3 CH2 - AUX5).
 * PB6  - Alternate Push Pull output (TIM4 CH1).
 * PB7  - Alternate Push Pull output (TIM4 CH2).
 * PB8  - Alternate Push Pull output (TIM4 CH3).
 * PB9  - Alternate Push Pull output (TIM4 CH4).
 * PB10 - Alternate Open Drain output (SCL).
 * PB11 - Alternate Open Drain output (SDA).
 * PB12 - Push Pull output (LED_A).
 * PB13 - Alternate Push Pull output (TIM1 CH1N).
 * PB14 - Alternate Push Pull output (TIM1 CH2N).
 * PB15 - Alternate Push Pull output (TIM1 CH3N).
 */
#define VAL_GPIOBCRL            0xAA4448AA      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0xAAA2FFAA      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC2  - Analog input (ADC Channel12 - AUX1).
 * PC3  - Analog input (ADC Channel13 - AUX2).
 * PC6  - Alternate Push Pull output (TIM8 CH1).
 * PC7  - Alternate Push Pull output (TIM8 CH2).
 * PC8  - Alternate Push Pull output (TIM8 CH3).
 * PC10 - Alternate Push Pull output (UART4 TX).
 * PC11 - Normal Input (UART4 RX).
 */
#define VAL_GPIOCCRL            0xAA880088      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88884A8A      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
 */
#define VAL_GPIODCRL            0x88888844      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup (not populated).
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

/*
 * Port F setup (not populated).
 * Everything input with pull-up except:
 */
#define VAL_GPIOFCRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOFCRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOFODR            0xFFFFFFFF

/*
 * Port G setup (not populated).
 * Everything input with pull-up except:
 */
#define VAL_GPIOGCRL            0x88888888      /*  PG7...PG0 */
#define VAL_GPIOGCRH            0x88888888      /* PG15...PG8 */
#define VAL_GPIOGODR            0xFFFFFFFF

/*
 * USB bus activation macro, required by the USB driver.
 */
#define usb_lld_connect_bus(usbp) { \
  palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_INPUT); \
}

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) { \
  palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_OUTPUT_PUSHPULL); \
  palClearPad(GPIOA, GPIOA_USB_DP); \
}

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
