CYGWIN=nodosfilewarning

# setup executables
CC_PREFIX ?= arm-none-eabi-
CC = $(CC_PREFIX)gcc
OBJCOPY		 = $(CC_PREFIX)objcopy
OBJDUMP		 = $(CC_PREFIX)objdump
SIZE		 = $(CC_PREFIX)size
CO_FLASH     = /cygdrive/c/CooCox/CoIDE/bin/coflash.exe

# indicate which platform we're building for
TARGET ?= STM32F3_Discovery
TARGETS = NAZE32 STM32F3_Discovery

# check for valid target
ifeq ($(TARGET),$(filter $(TARGET),$(TARGETS)),)
$(error Invalid Target '$(TARGET)'. Valid targets are $(TARGETS))
endif

USER_OPTIONS ?=

# directories
ROOT         := .
SRC_DIR		 = $(ROOT)/src
OBJ_DIR	     = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/bin
COOS_DIR     = $(SRC_DIR)/CoOS
TARGET_HEX   = $(BIN_DIR)/$(TARGET).hex
TARGET_ELF   = $(BIN_DIR)/$(TARGET).elf
TARGET_DIS   = $(BIN_DIR)/$(TARGET).dis
TARGET_MAP   = $(BIN_DIR)/$(TARGET).map

COMMON_CFLAGS = -ffunction-sections \
                -fdata-sections \
				-g \
				-Wall \
				-Wextra

OPTIMISE_FLAGS = -O3


INCLUDE_DIRS = 	$(SRC_DIR)/drivers \
				$(SRC_DIR)/flightcontroller \
				$(COOS_DIR) \
				$(COOS_DIR)/portable \
				$(COOS_DIR)/kernel \
				$(SRC_DIR)/STM32_USB-FS-Device_Driver/inc \
				$(SRC_DIR)/vcp \
				$(SRC_DIR)/stm32f30x

ifeq ($(TARGET),STM32F3_Discovery)
INCLUDE_DIRS := $(INCLUDE_DIRS) \
				$(SRC_DIR)/cmsis_boot/startup \
				$(SRC_DIR)/cmsis_boot \
				$(SRC_DIR)/cmsis_lib/include \
				$(SRC_DIR)/cmsis


CPU_FLAGS = -mcpu=cortex-m4 -mthumb
CPU_DEFINES = -DSTM32F303VC -DSTM32F30X

# use hardware floating point
FPU_FLAGS = -mfpu=fpv4-sp-d16 -mfloat-abi=hard
FPU_DEFINES = -D__FPU_USED
PLATFORM_FLAGS = STM32F303xC STM32F303

LINK_SCRIPT = $(ROOT)/arm-gcc-link-stm32f3-nizfc.ld
CO_FLASH_PROCESSOR_TYPE = STM32F303VC

CMSIS_BOOT_SRC = $(SRC_DIR)/cmsis_boot/startup/startup_stm32f30x.S \
                 $(SRC_DIR)/cmsis_boot/*.c

CMSIS_LIB_SRC = $(SRC_DIR)/cmsis_lib/source/*.c

TARGET_SRC = $(CMSIS_BOOT_SRC) \
             $(SRC_DIR)/drivers/stm32f3discovery/*.c \
             $(CMSIS_LIB_SRC)

STM32_SRC = $(SRC_DIR)/stm32f30x/*.c

USB_DIR = $(SRC_DIR)/STM32_USB-FS-Device_Driver/src
USB_SRC = $(USB_DIR)/*.c

VCP_DIR = $(SRC_DIR)/vcp
VCP_SRC = $(VCP_DIR)/*.c


else ifeq ($(TARGET),NAZE32)

INCLUDE_DIRS := $(INCLUDE_DIRS) \
				$(SRC_DIR)/cmsis/CM3/CoreSupport \
				$(SRC_DIR)/cmsis/CM3/DeviceSupport/ST/STM32F10x \
				$(SRC_DIR)/STM32F10x_StdPeriph_Driver/inc

CPU_FLAGS	 = -mthumb -mcpu=cortex-m3
CPU_DEFINES = -DSTM32F10X_MD -DSTM32F10X

LINK_SCRIPT = $(ROOT)/arm-gcc-link-stm32f103_128k.ld

STD_PERIPH_SRC = $(SRC_DIR)/STM32F10x_StdPeriph_Driver/src/*.c

CMSIS_BOOT_SRC = $(SRC_DIR)/cmsis_boot/startup/startup_stm32f10x_md_gcc.S

CMSIS_SRC	 = $(SRC_DIR)/cmsis/CM3/CoreSupport/*.c \
			   $(SRC_DIR)/cmsis/CM3/DeviceSupport/ST/STM32F10x/*.c

TARGET_SRC = $(STD_PERIPH_SRC) \
			 $(CMSIS_BOOT_SRC) \
			 $(CMSIS_SRC)

STM32_SRC = $(SRC_DIR)/stm32f10x/*.c

endif

STM32_SRC := $(STM32_SRC) \
			 $(SRC_DIR)/stm32common/*.c

LTO_FLAGS	 = -flto -fuse-linker-plugin

CFLAGS = $(COMMON_CFLAGS) \
			$(OPTIMISE_FLAGS) \
			-DUSE_STDPERIPH_DRIVER \
			-D'__TARGET__="$(TARGET)"' \
			$(addprefix -I,$(INCLUDE_DIRS)) \
			$(CPU_FLAGS) \
			$(CPU_DEFINES) \
			$(FPU_FLAGS) \
			$(FPU_DEFINES) \
			$(addprefix -D,$(PLATFORM_FLAGS)) \
			$(addprefix -D,$(USER_OPTIONS)) \
			$(LTO_FLAGS) \
			-MMD

COMMON_LDFLAGS = -g \
                 -Wall \
                 $(OPTIMISE_FLAGS) \
                 -Wl,--gc-sections \
                 -nostartfiles \
                 -lm
		         --specs=nano.specs \
		         -lc \
		         -lnosys

LDFLAGS = $(CPU_FLAGS) \
          $(FPU_FLAGS) \
          $(COMMON_LDFLAGS) \
          -Wl,-Map=$(TARGET_MAP) \
          $(LTO_FLAGS) \
          -T$(LINK_SCRIPT)

# now specify source files
COMMON_SRC = $(SRC_DIR)/flightcontroller/*.c \
             $(SRC_DIR)/drivers/*.c \
             $(SRC_DIR)/stdio/*.c
COOS_SRC = $(COOS_DIR)/kernel/*.c \
           $(COOS_DIR)/portable/*.c

COOS_SRC_NO_LTO = $(wildcard $(COOS_DIR)/portable/gcc/*.c)


SRC_FILES = $(COMMON_SRC) \
            $(TARGET_SRC) \
            $(STM32_SRC) \
            $(COOS_SRC) \
            $(USB_SRC) \
            $(VCP_SRC)

SRC_FILES_NO_LTO = $(COOS_SRC_NO_LTO) \
				$(SRC_DIR)/syscalls/*.c


# add .c files to object list
OBJS = $(patsubst %.c,%.o,$(wildcard $(SRC_FILES)))
# add .S files to object list
OBJS := $(patsubst %.S,%.o,$(OBJS))
# prepend obj directory to object list
OBJS := $(OBJS:%=$(OBJ_DIR)/%)

OBJS_NO_LTO = $(patsubst %.c,%.o,$(wildcard $(SRC_FILES_NO_LTO)))
# prepend obj directory to object list
OBJS_NO_LTO := $(OBJS_NO_LTO:%=$(OBJ_DIR)/no_lto/%)


TARGET_DEPENDENCIES = $(patsubst %.o,%.d,$(OBJS)) $(patsubst %.o,%.d,$(OBJS_NO_LTO))


all: $(TARGET_HEX)

flash: all
	$(CO_FLASH) program $(CO_FLASH_PROCESSOR_TYPE) $(TARGET_ELF) --adapter-name=ST-Link

dump: all
	$(OBJDUMP) -D $(TARGET_ELF) > $(TARGET_DIS)

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(OBJS) $(OBJS_NO_LTO)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

$(OBJ_DIR)/no_lto/%.o: CFLAGS_NO_LTO=$(filter-out -flto, $(CFLAGS))
$(OBJ_DIR)/no_lto/%.o : %.c
	@mkdir -p $(dir $@)
	@$(CC) -c -o $@ $< $(CFLAGS_NO_LTO)

$(OBJ_DIR)/%.o : %.c
	@mkdir -p $(dir $@)
	$(CC) -c -o $@ $< $(CFLAGS)

$(OBJ_DIR)/%.o: %.s
	@echo $<
	@mkdir -p $(dir $@)
	@$(CC) -c -o $@ $< $(CFLAGS)

$(OBJ_DIR)/%.o: %.S
	@echo $<
	@mkdir -p $(dir $@)
	@$(CC) -c -o $@ $< $(CFLAGS)


.PHONY:	clean


clean:
	rm -rf $(OBJS)
	rm -rf $(TARGET_ELF)
	rm -rf $(TARGET_HEX)

-include $(TARGET_DEPENDENCIES)

