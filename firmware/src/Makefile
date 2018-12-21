# Name of the binaries.
PROJ_NAME=ToneLock

######################################################################
#                         SETUP SOURCES                              #
######################################################################


# This is the directory containing the firmware package,
# the unzipped folder downloaded from here:
# http://www.st.com/web/en/catalog/tools/PF257904
STM_DIR=../

# This is where the source files are located,
# which are not in the current directory
# (the sources of the standard peripheral library, which we use)
# see also "info:/make/Selective Search" in Konqueror
STM_SRC = $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src
STM_SRC += $(STM_DIR)/Libraries/CMSIS/DSP_Lib/Source/TransformFunctions
STM_SRC += $(STM_DIR)/Libraries/CMSIS/DSP_Lib/Source/CommonTables

# Tell make to look in that folder if it cannot find a source
# in the current directory
vpath %.c $(STM_SRC)

# My source file
SRCS   = main.c

# Contains initialisation code and must be compiled into
# our project. This file is in the current directory and
# was writen by ST.
SRCS  += system_stm32f4xx.c

# These source files implement the functions we use.
# make finds them by searching the vpath defined above.
SRCS  += stm32f4xx_rcc.c 
SRCS  += stm32f4xx_gpio.c
SRCS  += stm32f4xx_usart.c
SRCS  += stm32f4xx_it.c
SRCS  += misc.c
SRCS  += ../Libraries/pdm_filter.a
SRCS  += stm32f4xx_spi.c
SRCS  += arm_rfft_f32.c
SRCS  += arm_rfft_init_f32.c
SRCS  += arm_cfft_radix4_f32.c
SRCS  += arm_cfft_radix4_init_f32.c
SRCS  += arm_common_tables.c

# Startup file written by ST
# The assembly code in this file is the first one to be
# executed. Normally you do not change this file.
SRCS += $(STM_DIR)/Libraries/CMSIS/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f4xx.s

# The header files we use are located here
INC_DIRS  = $(STM_DIR)/Utilities/STM32F4-Discovery
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/Include
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/DSP_Lib/Include
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/ST/STM32F4xx/Include
INC_DIRS += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc
INC_DIRS += ../inc
INC_DIRS += .

# in case we have to many sources and don't want 
# to compile all sources every time
# OBJS = $(SRCS:.c=.o)

######################################################################
#                         SETUP TOOLS                                #
######################################################################

# This is the path to ST-LINK
STLINK_DIR = ~/stlink/build/Release

# This is the path to the toolchain
# (we don't put our toolchain on $PATH to keep the system clean)
TOOLS_DIR = /usr/bin

# The tool we use
CC      = $(TOOLS_DIR)/arm-none-eabi-gcc
OBJCOPY = $(TOOLS_DIR)/arm-none-eabi-objcopy
GDB     = $(TOOLS_DIR)/arm-none-eabi-gdb

## Preprocessor options

# directories to be searched for header files
INCLUDE = $(addprefix -I,$(INC_DIRS))

# #defines needed when working with the STM library
DEFS    = -DUSE_STDPERIPH_DRIVER
# if you use the following option, you must implement the function 
#    assert_failed(uint8_t* file, uint32_t line)
# because it is conditionally used in the library
# DEFS   += -DUSE_FULL_ASSERT

## Compiler options
CFLAGS  = -ggdb
# please do not optimize anything because we are debugging
CFLAGS += -O0 
CFLAGS += -Wall -Wextra -Warray-bounds
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

## Linker options
# tell ld which linker file to use
# (this file is in the current directory)
LFLAGS  = -Tstm32_flash.ld


######################################################################
#                         SETUP TARGETS                              #
######################################################################

TARGET=../bin/$(PROJ_NAME)

.PHONY: $(TARGET)
$(TARGET): $(TARGET).elf
$(TARGET).elf: $(SRCS)
	@echo $(TARGET)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $(LFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(TARGET).elf   $(TARGET).hex
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin
	

clean:
	rm -f *.o $(TARGET).elf $(TARGET).hex $(TARGET).bin

# Flash the STM32F4
flash: 
	$(STLINK_DIR)/st-flash write $(TARGET).bin 0x8000000

.PHONY: debug
debug:
# before you start gdb, you must start st-util
	$(GDB) $(TARGET).elf