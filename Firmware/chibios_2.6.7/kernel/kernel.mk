# List of all the ChibiOS/RT kernel files, there is no need to remove the files
# from this list, you can disable parts of the kernel by editing chconf.h.
KERNSRC = ${CHIBIOS}/chibios_2.6.7/kernel/src/chsys.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chdebug.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chlists.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chvt.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chschd.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chthreads.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chdynamic.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chregistry.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chsem.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chmtx.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chcond.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chevents.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chmsg.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chmboxes.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chqueues.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chmemcore.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chheap.c \
          ${CHIBIOS}/chibios_2.6.7/kernel/src/chmempools.c

# Required include directories
KERNINC = ${CHIBIOS}/chibios_2.6.7/kernel/include
