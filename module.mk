MODULE_NAME ?= uDC_module

##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O0 -ggdb -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -fno-exceptions -fno-threadsafe-statics
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

ifeq ($(R2P_USE_DEBUGTRANSPORT),)
  R2P_USE_DEBUGTRANSPORT = no
endif

ifeq ($(R2P_USE_RTCANTRANSPORT),)
  R2P_USE_RTCANTRANSPORT = yes
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

R2P_ROOT ?= /opt/r2p
CHIBIOS ?= $(R2P_ROOT)/core/ChibiOS
RTCAN ?= $(R2P_ROOT)/core/RTCAN
MW ?= $(R2P_ROOT)/core/Middleware

# Define project name here
PROJECT ?= fw

# Imported source files and paths
include $(CHIBIOS)/os/hal/platforms/STM32F30x/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F3xx/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include $(RTCAN)/platforms/STM32/platform.mk
include $(RTCAN)/RTCAN.mk
include $(MW)/middleware.mk
include $(MW)/port/chibios/port.mk

# Define linker script file here
LDSCRIPT ?= $(PORTLD)/STM32F303xC.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/syscalls.c \
       $(RTCANSRC) \
       $(RTCANPLATFORMSRC) \
       $(MODULE_PATH)/board.c \
       $(MODULE_PATH)/stubs.c \
       $(MODULE_PATH)/qei.c \
       $(MODULE_PATH)/qei_lld.c \
       $(PACKAGES_CSRC) \
       $(PRJ_CSRC)

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(MW_CPPSRC) \
         chnew.cpp \
         $(PACKAGES_CPPSRC) \
         $(PRJ_CPPSRC)

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHIBIOS)/os/various \
         $(RTCANINC) $(RTCANPLATFORMINC) \
         $(MW_INC) \
         $(PACKAGES_INC) \
         $(MODULE_PATH)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
DDEFS = -DMODULE_NAME="\"$(MODULE_NAME)\""

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS += -DR2P_ITERATE_PUBSUB=1 -DR2P_USE_BRIDGE_MODE=0 -DR2P_USE_BOOTLOADER=0 #-DR2P_ASSERT=""

# Define ASM defines here
UADEFS +=

# List all user directories here
UINCDIR +=

# List the user directory to look for the libraries here
ULIBDIR +=

# List all user libraries here
ULIBS +=

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/ports/GCC/ARMCMx
include $(RULESPATH)/rules.mk
