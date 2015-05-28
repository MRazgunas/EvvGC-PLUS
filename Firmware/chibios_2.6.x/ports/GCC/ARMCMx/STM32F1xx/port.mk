# List of the ChibiOS/RT Cortex-M3 STM32 port files.
PORTSRC = ${SRCDIR}/chibios_2.6.x/ports/GCC/ARMCMx/crt0.c \
          ${SRCDIR}/chibios_2.6.x/ports/GCC/ARMCMx/STM32F1xx/vectors.c \
          ${SRCDIR}/chibios_2.6.x/ports/GCC/ARMCMx/chcore.c \
          ${SRCDIR}/chibios_2.6.x/ports/GCC/ARMCMx/chcore_v7m.c \
          ${SRCDIR}/chibios_2.6.x/ports/common/ARMCMx/nvic.c

PORTASM =

PORTINC = ${SRCDIR}/chibios_2.6.x/ports/common/ARMCMx/CMSIS/include \
          ${SRCDIR}/chibios_2.6.x/ports/common/ARMCMx \
          ${SRCDIR}/chibios_2.6.x/ports/GCC/ARMCMx \
          ${SRCDIR}/chibios_2.6.x/ports/GCC/ARMCMx/STM32F1xx

PORTLD  = ${SRCDIR}/chibios_2.6.x/ports/GCC/ARMCMx/STM32F1xx/ld
