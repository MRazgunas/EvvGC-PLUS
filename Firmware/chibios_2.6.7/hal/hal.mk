# List of all the ChibiOS/RT HAL files, there is no need to remove the files
# from this list, you can disable parts of the HAL by editing halconf.h.
HALSRC = ${CHIBIOS}/chibios_2.6.7/hal/src/hal.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/adc.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/can.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/ext.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/gpt.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/i2c.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/icu.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/mac.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/mmc_spi.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/mmcsd.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/pal.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/pwm.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/rtc.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/sdc.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/serial.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/serial_usb.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/spi.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/tm.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/uart.c \
         ${CHIBIOS}/chibios_2.6.7/hal/src/usb.c

# Required include directories
HALINC = ${CHIBIOS}/chibios_2.6.7/hal/include
