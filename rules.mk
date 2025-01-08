# 'make V=1' will show all compiler calls.
V		?= 0
ifeq ($(V),0)
Q		:= @
NULL	:= 2>/dev/null
endif

PREFIX		?= $(ARM_TOOLCHAIN)/arm-none-eabi-
CC		= $(PREFIX)gcc
XX		= $(PREFIX)g++
AS		= $(PREFIX)as
LD		= $(PREFIX)ld
OBJCOPY		= $(PREFIX)objcopy
OBJDUMP		= $(PREFIX)objdump
BDIR := $(BUILD_DIR)

# TOP is py32f0-template, SRC is user's base - identical in the simple case but need not be
TOP := $(dir $(lastword $(MAKEFILE_LIST)))
SRC := $(TOP:%py32f0-template/=%)
ifneq (,$(findstring ../,$(TOP)))
	# py32f0-template is a neighbor to cwd, ensure proper build dir structure
	SRC_PREFIX := ../$(notdir $(shell pwd))/
	ADIRS := $(ADIRS:%=$(SRC_PREFIX)%)
	AFILES := $(AFILES:%=$(SRC_PREFIX)%)
	CDIRS := $(CDIRS:%=$(SRC_PREFIX)%)
	CFILES := $(CFILES:%=$(SRC_PREFIX)%)
	CPPFILES := $(CPPFILES:%=$(SRC_PREFIX)%)
endif

##### Library Paths ############

LIB := $(TOP)Libraries/

# Library flags
LIB_FLAGS += $(MCU_TYPE)
# JLink device (Uppercases)
JLINK_DEVICE ?= $(shell echo $(MCU_TYPE) | tr '[:lower:]' '[:upper:]')
# PyOCD device (Lowercases)
PYOCD_DEVICE ?= $(shell echo $(MCU_TYPE) | tr '[:upper:]' '[:lower:]')
# Link descript file: 
LDSCRIPT ?= $(LIB)LDScripts/$(PYOCD_DEVICE).ld


ifneq (,$(findstring PY32F002B,$(MCU_TYPE)))

# PY32F002B >>>
CFILES += $(LIB)CMSIS/Device/PY32F0xx/Source/system_py32f002b.c

ifeq ($(USE_LL_LIB),y)
CDIRS += \
	$(LIB)PY32F002B_LL_Driver/Src \
	$(LIB)PY32F002B_LL_BSP/Src
INCLUDES += \
	$(LIB)PY32F002B_LL_Driver/Inc \
	$(LIB)PY32F002B_LL_BSP/Inc
LIB_FLAGS += USE_FULL_LL_DRIVER
else
CDIRS += \
	$(LIB)PY32F002B_HAL_Driver/Src  \
	$(LIB)PY32F002B_HAL_BSP/Src
INCLUDES +=
	$(LIB)PY32F002B_HAL_Driver/Inc \
	$(LIB)PY32F002B_HAL_BSP/Inc
endif
# Startup file
AFILES += $(LIB)CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f002b.s
# PY32F002B <<<

else ifneq (,$(findstring PY32F07,$(MCU_TYPE)))

# PY32F07x >>>
CFILES += $(LIB)CMSIS/Device/PY32F0xx/Source/system_py32f07x.c

CDIRS += \
	$(LIB)PY32F07x_HAL_Driver/Src \
	$(LIB)PY32F07x_HAL_BSP/Src
INCLUDES += \
	$(LIB)PY32F07x_HAL_Driver/Inc \
	$(LIB)PY32F07x_HAL_BSP/Inc
LIB_FLAGS += USE_HAL_DRIVER
# Startup file
AFILES += $(LIB)CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f072.s
# PY32F07 <<<

else

# PY32F002A,003,030 >>>
CFILES += $(LIB)CMSIS/Device/PY32F0xx/Source/system_py32f0xx.c

ifeq ($(USE_LL_LIB),y)
CDIRS += \
	$(LIB)PY32F0xx_LL_Driver/Src \
	$(LIB)PY32F0xx_LL_BSP/Src
INCLUDES += \
	$(LIB)PY32F0xx_LL_Driver/Inc \
	$(LIB)PY32F0xx_LL_BSP/Inc
LIB_FLAGS += USE_FULL_LL_DRIVER
else
CDIRS += \
	$(LIB)PY32F0xx_HAL_Driver/Src \
	$(LIB)PY32F0xx_HAL_BSP/Src
INCLUDES += \
	$(LIB)PY32F0xx_HAL_Driver/Inc \
	$(LIB)PY32F0xx_HAL_BSP/Inc
endif
# Startup file
ifneq (,$(findstring PY32F002A,$(LIB_FLAGS)))
AFILES += $(LIB)CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f002a.s
endif
ifneq (,$(findstring PY32F003,$(LIB_FLAGS)))
AFILES += $(LIB)CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f003.s
endif
ifneq (,$(findstring PY32F030,$(LIB_FLAGS)))
AFILES += $(LIB)CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f030.s
endif
# PY32F002A,003,030 <<<

endif

######## Additional Libs ########

ifeq ($(USE_FREERTOS),y)
CDIRS += \
	$(LIB)FreeRTOS \
	$(LIB)FreeRTOS/portable/GCC/ARM_CM0

CFILES += $(LIB)FreeRTOS/portable/MemMang/heap_4.c

INCLUDES += \
	$(LIB)FreeRTOS/include \
	$(LIB)FreeRTOS/portable/GCC/ARM_CM0
endif

ifeq ($(USE_DSP),y)
CFILES += \
	$(LIB)CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c \
	$(LIB)CMSIS/DSP/Source/BayesFunctions/BayesFunctions.c \
	$(LIB)CMSIS/DSP/Source/CommonTables/CommonTables.c \
	$(LIB)CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c \
	$(LIB)CMSIS/DSP/Source/ControllerFunctions/ControllerFunctions.c \
	$(LIB)CMSIS/DSP/Source/DistanceFunctions/DistanceFunctions.c \
	$(LIB)CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c \
	$(LIB)CMSIS/DSP/Source/FilteringFunctions/FilteringFunctions.c \
	$(LIB)CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.c \
	$(LIB)CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c \
	$(LIB)CMSIS/DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c \
	$(LIB)CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c \
	$(LIB)CMSIS/DSP/Source/SupportFunctions/SupportFunctions.c \
	$(LIB)CMSIS/DSP/Source/SVMFunctions/SVMFunctions.c \
	$(LIB)CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c
INCLUDES += \
	$(LIB)CMSIS/DSP/Include \
	$(LIB)CMSIS/DSP/PrivateInclude
endif

ifeq ($(USE_EPAPER),y)
CDIRS += \
	$(LIB)EPaper/Lib \
	$(LIB)EPaper/Examples \
	$(LIB)EPaper/Fonts \
	$(LIB)EPaper/GUI

INCLUDES += \
	$(LIB)EPaper/Lib \
	$(LIB)EPaper/Examples \
	$(LIB)EPaper/Fonts \
	$(LIB)EPaper/GUI
endif

# For each direcotry, add it to csources
CSOURCES := $(foreach dir, $(CDIRS), $(shell find $(dir) -maxdepth 1 -name '*.c'))
# Add single c source files to csources
CSOURCES += $(CFILES)
# C++ files
CPPSOURCES := $(foreach dir, $(CDIRS), $(shell find $(dir) -maxdepth 1 -name '*.cpp'))
CPPSOURCES += $(CPPFILES)

# Then assembly source folders and files
ASOURCES := $(foreach dir, $(ADIRS), $(shell find $(dir) -maxdepth 1 -iname '*.s'))
ASOURCES += $(AFILES)

# Fill object files with c and asm files (keep source directory structure)
OBJS := $(CSOURCES:%.c=%.o)
OBJS += $(CPPSOURCES:%.cpp=%.o)
OBJS += $(patsubst %.s,%.o, \
        $(patsubst %.S,%.o,$(ASOURCES)))
OBJS := $(OBJS:../%=%)
OBJS := $(OBJS:%=$(BDIR)/%)

# d files for detecting h file changes
DEPS := $(OBJS:%.o=%.d)

# Arch and target specified flags
ARCH_FLAGS	:= -mcpu=cortex-m0plus
# Debug options, -gdwarf-2 for debug, -g0 for release 
# https://gcc.gnu.org/onlinedocs/gcc-12.2.0/gcc/Debugging-Options.html
#  -g: system’s native format, -g0:off, -g/g1,-g2,-g3 -> more verbosely
#  -ggdb: for gdb, -ggdb0:off, -ggdb/ggdb1,-ggdb2,-ggdb3 -> more verbosely
#  -gdwarf: in DWARF format, -gdwarf-2,-gdwarf-3,-gdwarf-4,-gdwarf-5
DEBUG_FLAGS ?= -gdwarf-3

OPT		?= -Os
# C flags
TGT_CFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -std=c17 $(addprefix -D, $(LIB_FLAGS)) -Wall -ffunction-sections -fdata-sections
# C++ flags
TGT_CPPFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -std=c++11 $(addprefix -D, $(LIB_FLAGS)) -Wall -ffunction-sections -fdata-sections
# ASM flags
TGT_ASFLAGS	?= $(ARCH_FLAGS) $(DEBUG_FLAGS) $(OPT) -Wa,--warn
# LD flags
TGT_LDFLAGS	?= $(ARCH_FLAGS) -specs=nano.specs -specs=nosys.specs -lc -lm \
				-Wl,-Map=$(BDIR)/$(PROJECT).map \
				-Wl,--gc-sections \
				-Wl,--print-memory-usage

GCC_VERSION := $(shell $(CC) -dumpversion)
IS_GCC_ABOVE_12 := $(shell expr "$(GCC_VERSION)" ">=" "12")
ifeq "$(IS_GCC_ABOVE_12)" "1"
    TGT_LDFLAGS += -Wl,--no-warn-rwx-segments
endif

ifeq ($(ENABLE_PRINTF_FLOAT),y)
TGT_LDFLAGS	+= -u _printf_float
endif

# include paths
INCLUDES += \
	$(LIB)CMSIS/Core/Include \
	$(LIB)CMSIS/Device/PY32F0xx/Include
TGT_INCFLAGS := $(addprefix -I , $(INCLUDES))


.PHONY: all clean flash echo

all: fullcheck $(BDIR)/$(PROJECT).elf $(BDIR)/$(PROJECT).bin $(BDIR)/$(PROJECT).hex $(BDIR)/$(PROJECT).lst

fullcheck:
	@if [ '$(findstring PY32F07,$(MCU_TYPE))' = 'PY32F07' ] && [ '$(USE_LL_LIB)' = 'y' ]; then \
		echo "LL for PY32F07x is not supported yet"; \
		return 1; \
	fi

# for debug
echo:
	$(info 1. $(AFILES))
	$(info 2. $(ASOURCES))
	$(info 3. $(CSOURCES))
	$(info 4. $(OBJS))
	$(info 5. $(TGT_INCFLAGS))

# include d files without non-exist warning
-include $(DEPS)

# Compile c to obj
$(BDIR)/%.o: $(SRC)%.c
	@printf "  CC\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_CFLAGS) $(TGT_INCFLAGS) -MT $@ -o $@ -c $< -MD -MF $(BDIR)/$*.d -MP

$(BDIR)/%.o: $(SRC)%.cpp
	@printf "  XX\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(XX) $(TGT_CPPFLAGS) $(TGT_INCFLAGS) -MT $@ -o $@ -c $< -MD -MF $(BDIR)/$*.d -MP

# Compile asm to obj
$(BDIR)/%.o: $(SRC)%.s
	@printf "  AS\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_ASFLAGS) -o $@ -c $<

# Compile asm to obj with preprocessing
$(BDIR)/%.o: $(SRC)%.S
	@printf "  AS\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_ASFLAGS) $(TGT_INCFLAGS) -o $@ -c $< -MD -MF $(BDIR)/$*.d -MP

# Link object files to elf
$(BDIR)/$(PROJECT).elf: $(OBJS) $(LDSCRIPT)
	@printf "  LD\t$(LDSCRIPT) -> $@\n"
	$(Q)$(CC) $(TGT_LDFLAGS) -T$(LDSCRIPT) $(OBJS) -o $@

# Convert elf to bin
%.bin: %.elf
	@printf "  OBJCP BIN\t$@\n"
	$(Q)$(OBJCOPY) -I elf32-littlearm -O binary  $< $@

# Convert elf to hex
%.hex: %.elf
	@printf "  OBJCP HEX\t$@\n"
	$(Q)$(OBJCOPY) -I elf32-littlearm -O ihex  $< $@

# Create assembly listing
%.lst: %.elf
	@printf "  OBJDP LST\t$@\n"
	$(Q)$(OBJDUMP) --source $< > $@

clean:
	rm -rf $(BDIR)/*

flash: $(BDIR)/$(PROJECT).elf
ifeq ($(FLASH_PROGRM),jlink)
	$(JLINKEXE) -device $(JLINK_DEVICE) -if swd -speed 4000 -JLinkScriptFile $(TOP)Misc/jlink-script -CommanderScript $(TOP)Misc/jlink-command
else ifeq ($(FLASH_PROGRM),pyocd)
	$(PYOCD_EXE) erase -t $(PYOCD_DEVICE) --chip --config $(TOP)Misc/pyocd.yaml
	$(PYOCD_EXE) load $< -t $(PYOCD_DEVICE) --config $(TOP)Misc/pyocd.yaml
else
	@echo "FLASH_PROGRM is invalid\n"
endif
