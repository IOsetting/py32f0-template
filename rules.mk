# 'make V=1' will show all compiler calls.
V		?= 0
ifeq ($(V),0)
Q		:= @
NULL	:= 2>/dev/null
endif

PREFIX		?= $(ARM_TOOCHAIN)/arm-none-eabi-
CC			= $(PREFIX)gcc
AS			= $(PREFIX)as
LD			= $(PREFIX)ld
OBJCOPY		= $(PREFIX)objcopy
# `$(shell pwd)` or `.`, both works
TOP			= .
BDIR		= $(TOP)/$(BUILD_DIR)

# For each direcotry, add it to csources
CSOURCES := $(foreach dir, $(CDIRS), $(shell find $(TOP)/$(dir) -name '*.c'))
# Add single c source files to csources
CSOURCES += $(addprefix $(TOP)/, $(CFILES))
# Then assembly source folders and files
ASOURCES := $(foreach dir, $(ADIRS), $(shell find $(TOP)/$(dir) -name '*.s'))
ASOURCES += $(addprefix $(TOP)/, $(AFILES))

# Fill object files with c and asm files (keep source directory structure)
OBJS = $(CSOURCES:$(TOP)/%.c=$(BDIR)/%.o)
OBJS += $(ASOURCES:$(TOP)/%.s=$(BDIR)/%.o)
# d files for detecting h file changes
DEPS=$(CSOURCES:$(TOP)/%.c=$(BDIR)/%.d)

# Global compile flags
CFLAGS		= -Wall -ggdb -ffunction-sections -fdata-sections
ASFLAGS		= -g -Wa,--warn

# Arch and target specified flags
OPT			?= -Os
CSTD		?= -std=c99
ARCH_FLAGS	:= -fno-common -mcpu=cortex-m0plus -mthumb

### c flags ###
TGT_CFLAGS 	+= $(ARCH_FLAGS) $(addprefix -D, $(LIB_FLAGS))

### asm flags ###
TGT_ASFLAGS += $(ARCH_FLAGS)

### ld flags ###
TGT_LDFLAGS += --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--gc-sections -Wl,-Map=$(BDIR)/$(PROJECT).map -Wl,--print-memory-usage
 # Use newlib-nano instead of newlib for smaller flash size
TGT_LDFLAGS += --specs=nano.specs
 # Exclude standard initialization actions, when __libc_init_array exists, this should be omit, \
   otherwise it will generate "undefined reference to `_init'" error. \
   **Remove** `bl __libc_init_array` from startup.s if you want to enable this.
# TGT_LDFLAGS += -nostartfiles


### included paths ###
TGT_INCFLAGS := $(addprefix -I $(TOP)/, $(INCLUDES))


.PHONY: all clean flash echo

all: $(BDIR)/$(PROJECT).elf $(BDIR)/$(PROJECT).bin $(BDIR)/$(PROJECT).hex

# for debug
echo:
	$(info 1. $(AFILES))
	$(info 2. $(ASOURCES))
	$(info 3. $(CSOURCES))
	$(info 4. $(OBJS))
	$(info 5. $(TGT_INCFLAGS))

# include d files without non-exist warning
-include $(DEPS)

# Compile c to obj -- should be `$(BDIR)/%.o: $(TOP)/%.c`, but since $(TOP) is base folder so non-path also works
$(BDIR)/%.o: %.c
	@printf "  CC\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_CFLAGS) $(OPT) $(CFLAGS) $(TGT_INCFLAGS) -MT $@ -MMD -MP -MF $(BDIR)/$*.d -o $@ -c $< 

# Compile asm to obj
$(BDIR)/%.o: %.s
	@printf "  AS\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_ASFLAGS) $(ASFLAGS) -o $@ -c $<

# Link object files to elf
$(BDIR)/$(PROJECT).elf: $(OBJS) $(TOP)/$(LDSCRIPT)
	@printf "  LD\t$@\n"
	$(Q)$(CC) $(OBJS) $(TGT_LDFLAGS) -T$(TOP)/$(LDSCRIPT) -o $@

# Convert elf to bin
%.bin: %.elf
	@printf "  OBJCP BIN\t$@\n"
	$(Q)$(OBJCOPY) -I elf32-littlearm -O binary  $< $@

# Convert elf to hex
%.hex: %.elf
	@printf "  OBJCP HEX\t$@\n"
	$(Q)$(OBJCOPY) -I elf32-littlearm -O ihex  $< $@

clean:
	rm -rf $(BDIR)/*

flash:
ifeq ($(FLASH_PROGRM),jlink)
	$(JLINKEXE) -device $(JLINK_DEVICE) -if swd -speed 4000 -CommanderScript $(TOP)/Misc/flash.jlink
else ifeq ($(FLASH_PROGRM),pyocd)
	$(PYOCD_EXE) erase -t $(PYOCD_DEVICE) --chip --config $(TOP)/Misc/pyocd.yaml
	$(PYOCD_EXE) load $(BDIR)/$(PROJECT).hex -t $(PYOCD_DEVICE) --config $(TOP)/Misc/pyocd.yaml
else
	@echo "FLASH_PROGRM is invalid\n"
endif
