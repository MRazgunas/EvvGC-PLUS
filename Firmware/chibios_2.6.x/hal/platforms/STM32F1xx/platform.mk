# List of all the STM32F1xx platform files.
PLATFORMSRC = ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32F1xx/stm32_dma.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32F1xx/hal_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32F1xx/adc_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32F1xx/ext_lld_isr.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/can_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/ext_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/mac_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/sdc_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/GPIOv1/pal_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/I2Cv1/i2c_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/RTCv1/rtc_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/SPIv1/spi_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/TIMv1/gpt_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/TIMv1/icu_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/TIMv1/pwm_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/USARTv1/serial_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/USARTv1/uart_lld.c \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/USBv1/usb_lld.c

# Required include directories
PLATFORMINC = ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32F1xx \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32 \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/GPIOv1 \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/I2Cv1 \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/RTCv1 \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/SPIv1 \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/TIMv1 \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/USARTv1 \
              ${SRCDIR}/chibios_2.6.x/hal/platforms/STM32/USBv1

