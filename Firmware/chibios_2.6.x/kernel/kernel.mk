# List of all the ChibiOS/RT kernel files, there is no need to remove the files
# from this list, you can disable parts of the kernel by editing chconf.h.
KERNSRC = ${SRCDIR}/chibios_2.6.x/kernel/src/chsys.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chdebug.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chlists.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chvt.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chschd.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chthreads.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chdynamic.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chregistry.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chsem.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chmtx.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chcond.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chevents.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chmsg.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chmboxes.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chqueues.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chmemcore.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chheap.c \
          ${SRCDIR}/chibios_2.6.x/kernel/src/chmempools.c

# Required include directories
KERNINC = ${SRCDIR}/chibios_2.6.x/kernel/include
