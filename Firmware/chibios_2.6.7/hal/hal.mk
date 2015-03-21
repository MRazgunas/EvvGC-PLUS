# List of all the ChibiOS/RT HAL files, there is no need to remove the files
# from this list, you can disable parts of the HAL by editing halconf.h.
HALSRC = ${SRCDIR}/chibios_2.6.7/hal/src/hal.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/adc.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/can.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/ext.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/gpt.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/i2c.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/icu.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/mac.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/mmc_spi.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/mmcsd.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/pal.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/pwm.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/rtc.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/sdc.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/serial.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/serial_usb.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/spi.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/tm.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/uart.c \
         ${SRCDIR}/chibios_2.6.7/hal/src/usb.c

# Required include directories
HALINC = ${SRCDIR}/chibios_2.6.7/hal/include
