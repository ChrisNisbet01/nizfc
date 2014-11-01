CYGWIN=nodosfilewarning

# setup executables
CC_PREFIX ?= arm-none-eabi-
CC = $(CC_PREFIX)gcc
OBJCOPY		 = $(CC_PREFIX)objcopy
SIZE		 = $(CC_PREFIX)size
CO_FLASH     = /cygdrive/c/CooCox/CoIDE/bin/coflash.exe

# indicate which platform we're building for
TARGET ?= STM32F3_Discovery
TARGETS = STM32F3_Discovery

# check for valid target
ifeq ($(filter $(TARGET),$(TARGETS)),)
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
TARGET_MAP   = $(BIN_DIR)/$(TARGET).map

COMMON_CFLAGS = -ffunction-sections \
                -fdata-sections \
				-g \
				-Wall \
				-Wextra

OPTIMISE_FLAGS = -Os

INCLUDE_DIRS = $(SRC_DIR)/cmsis_boot/startup \
				$(SRC_DIR)/cmsis_boot \
				$(SRC_DIR)/cmsis_lib/include \
				$(SRC_DIR)/cmsis \
				$(SRC_DIR)/drivers \
				$(SRC_DIR)/flightcontroller \
				$(COOS_DIR) \
				$(COOS_DIR)/portable \
				$(COOS_DIR)/kernel \
				$(SRC_DIR)/STM32F3_Discovery \
				$(SRC_DIR)/stm32f30x

ifeq ($(TARGET),STM32F3_Discovery)
CPU_FLAGS = -mcpu=cortex-m4 -mthumb
CPU_DEFINES = -DSTM32F303VC -DSTM32F30X

# use hardware floating point
FPU_FLAGS = -mfpu=fpv4-sp-d16 -mfloat-abi=hard
FPU_DEFINES = -D__FPU_USED
PLATFORM_FLAGS =

PLATFORM_DIR = $(SRC_DIR)/STM32F3_Discovery
PLATFORM_SRC = $(PLATFORM_DIR)/*.c \
               $(SRC_DIR)/stm32f30x/*.c
LINK_SCRIPT = $(ROOT)/arm-gcc-link-stm32f3-nizfc.ld
CO_FLASH_PROCESSOR_TYPE = STM32F303VC

endif

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
			-MMD

COMMON_LDFLAGS = -g \
                 -Wall \
                 -Os \
                 -Wl,--gc-sections \
                 -nostartfiles \
                 -lm
		         --specs=nano.specs \
		         -lc \
		         -lnosys \

LDFLAGS = $(CPU_FLAGS) \
          $(FPU_FLAGS) \
          $(COMMON_LDFLAGS) \
          -Wl,-Map=$(TARGET_MAP) \
          -T$(LINK_SCRIPT)

# now specify source files
COMMON_SRC = $(SRC_DIR)/flightcontroller/*.c \
             $(SRC_DIR)/drivers/*.c \
             $(SRC_DIR)/stdio/*.c
COOS_SRC = $(COOS_DIR)/kernel/*.c \
           $(COOS_DIR)/portable/*.c \
           $(COOS_DIR)/portable/gcc/*.c \

CMSIS_BOOT_SRC = $(SRC_DIR)/cmsis_boot/startup/*.S \
                 $(SRC_DIR)/cmsis_boot/*.c

CMSIS_LIB_SRC = $(SRC_DIR)/cmsis_lib/source/*.c

SRC_FILES = $(COMMON_SRC) \
            $(CMSIS_BOOT_SRC) \
            $(PLATFORM_SRC) \
            $(COOS_SRC) \
            $(CMSIS_LIB_SRC)

# add .c files to object list
OBJS = $(patsubst %.c,%.o,$(wildcard $(SRC_FILES)))
# add .S files to object list
OBJS := $(patsubst %.S,%.o,$(OBJS))
# prepend obj directory to object list
OBJS := $(OBJS:%=$(OBJ_DIR)/%)

TARGET_DEPENDENCIES = $(patsubst %.o,%.d,$(OBJS))


all: $(TARGET_HEX)

flash: all
	$(CO_FLASH) program $(CO_FLASH_PROCESSOR_TYPE) $(TARGET_ELF) --adapter-name=ST-Link

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

$(OBJ_DIR)/%.o : %.c
	@mkdir -p $(dir $@)
	@$(CC) -c -o $@ $< $(CFLAGS)

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

