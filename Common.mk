### Machine flags
#
CC_CMACH	= -mmcu=cc430f6137
CC_DMACH	= -D__MSP430_6137__ -DMRFI_CC430 -D__CC430F6137__
FREQ_FLAGS 	?= $(shell cat config.h | grep CONFIG_FREQUENCY | sed 's/.define CONFIG_FREQUENCY //' | sed 's/902/-DISM_US/' | sed 's/433/-DISM_LF/' | sed 's/868/-DISM_EU/')

### Build flags
#
# -fdata-sections, -ffunction-sections and -Wl,--gc-sections -Wl,-s
# are used for dead code elimination, see:
# http://gcc.gnu.org/ml/gcc-help/2003-08/msg00128.html
#
CFLAGS		+= $(CC_CMACH) $(CC_DMACH) $(FREQ_FLAGS) -Wall
CFLAGS		+= -fno-force-addr -finline-limit=1 -fno-schedule-insns
CFLAGS		+= -fshort-enums -Wl,-Map=output.map
LDFLAGS		=

CFLAGS_REL	+= -Os -fdata-sections -ffunction-sections -fomit-frame-pointer
LDFLAGS_REL	+= -Wl,--gc-sections -Wl,-s

CFLAGS_DBG	+= -O1 -ggdb
LDFLAGS_DBG	+= -Wl,--gc-sections

# linker flags and include directories
INCLUDES	+= -I/usr/msp430/include/ -I/usr/lib/gcc/msp430/4.6.3/include/ -I./ -I./drivers/
### Build tools
# 
CC		= msp430-gcc
LD		= msp430-ld
AS		= msp430-as
AR		= msp430-ar
