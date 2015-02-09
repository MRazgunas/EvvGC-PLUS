# List of the ChibiOS/RT Cortex-M3 STM32 port files.
PORTSRC = $(CHIBIOS)/chibios_2.6.7/ports/GCC/ARMCMx/crt0.c \
          $(CHIBIOS)/chibios_2.6.7/ports/GCC/ARMCMx/STM32F1xx/vectors.c \
          ${CHIBIOS}/chibios_2.6.7/ports/GCC/ARMCMx/chcore.c \
          ${CHIBIOS}/chibios_2.6.7/ports/GCC/ARMCMx/chcore_v7m.c \
          ${CHIBIOS}/chibios_2.6.7/ports/common/ARMCMx/nvic.c

PORTASM =

PORTINC = ${CHIBIOS}/chibios_2.6.7/ports/common/ARMCMx/CMSIS/include \
          ${CHIBIOS}/chibios_2.6.7/ports/common/ARMCMx \
          ${CHIBIOS}/chibios_2.6.7/ports/GCC/ARMCMx \
          ${CHIBIOS}/chibios_2.6.7/ports/GCC/ARMCMx/STM32F1xx

PORTLD  = ${CHIBIOS}/chibios_2.6.7/ports/GCC/ARMCMx/STM32F1xx/ld
