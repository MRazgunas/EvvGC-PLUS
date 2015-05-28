# List of all the ChibiOS/RT HAL files, there is no need to remove the files
# from this list, you can disable parts of the HAL by editing halconf.h.
HALSRC = ${SRCDIR}/chibios_2.6.x/hal/src/hal.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/adc.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/can.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/ext.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/gpt.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/i2c.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/icu.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/mac.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/mmc_spi.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/mmcsd.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/pal.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/pwm.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/rtc.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/sdc.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/serial.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/serial_usb.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/spi.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/tm.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/uart.c \
         ${SRCDIR}/chibios_2.6.x/hal/src/usb.c

# Required include directories
HALINC = ${SRCDIR}/chibios_2.6.x/hal/include
