RM := rm -rf

BUILD_ARTIFACT_NAME := k91_main_source.0x080A0000
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
$(BUILD_ARTIFACT_NAME).elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
$(BUILD_ARTIFACT_NAME).list \

OBJCOPY_HEX += \
$(BUILD_ARTIFACT_NAME).hex \

OBJCOPY_BIN += \
$(BUILD_ARTIFACT_NAME).bin \

OBJCOPY_SREC += \
$(BUILD_ARTIFACT_NAME).srec \

PROJDIR := $(realpath $(CURDIR)/..)

TOP := $(shell pwd)

#setting s source folders
S_SOURCEDIR := Core/Startup \

#setting c source folders
C_SOURCEDIR := \
	Core/Src \
	Drivers/STM32F4xx_HAL_Driver/Src \
	Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
	Middlewares/Third_Party/FreeRTOS/Source \
	Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
	Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang \
	chirp/src \

#setting include folders
INC_FOLDERS := -I../Core/Inc \
	-I../chirp/inc \
	-I../bare_driver/platform/inc \
	-I../bare_driver/core/api/inc \
	-I../bare_driver/core/dci/inc \
	-I../bare_driver/core/common/inc \
	-I../bare_driver/config \
	-I../Drivers/STM32F4xx_HAL_Driver/Inc \
	-I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
	-I../Drivers/CMSIS/Device/ST/STM32F4xx/Include \
	-I../Drivers/CMSIS/Include \
	-I../Middlewares/Third_Party/FreeRTOS/Source/include \
	-I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
	-I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \

S_SOURCEDIRS := $(foreach dir,$(S_SOURCEDIR),$(addprefix $(PROJDIR)/,$(dir)))
#$(info $$S_SOURCEDIRS is [${S_SOURCEDIRS}])

S_OBJDIRS := $(foreach dir,$(S_SOURCEDIR),$(addprefix $(TOP)/,$(dir)))

S_SOURCE := $(foreach dir,$(S_SOURCEDIRS),$(wildcard $(dir)/*.s))

S_OBJS := $(subst $(PROJDIR),$(TOP),$(S_SOURCE:%.s=%.o))

C_SOURCEDIRS := $(foreach dir,$(C_SOURCEDIR),$(addprefix $(PROJDIR)/,$(dir)))

C_OBJDIRS := $(foreach dir,$(C_SOURCEDIR),$(addprefix $(TOP)/,$(dir)))

C_SOURCE := $(foreach dir,$(C_SOURCEDIRS),$(wildcard $(dir)/*.c))

C_OBJS := $(subst $(PROJDIR),$(TOP),$(C_SOURCE:.c=.o))

MAKEDIRS = $(foreach dir,$(S_OBJDIRS),$(shell mkdir -p $(dir))) $(foreach dir,$(C_OBJDIRS),$(shell mkdir -p $(dir)))

CFLAGS := $(INC_FOLDERS) -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb

# All Target
all: main-build

# Main-build Targets
main-build: $(MAKEDIRS) $(BUILD_ARTIFACT_NAME).elf secondary-outputs

$(C_OBJS):
	arm-none-eabi-gcc $(subst .o,.c,$(subst $(TOP),$(PROJDIR),$@)) -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DDEBUG -c $(CFLAGS) -o $@

$(S_OBJS):$(S_SOURCE)
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o $@ $<

$(BUILD_ARTIFACT_NAME).elf: $(S_OBJS) $(C_OBJS) $(PROJDIR)/STM32F405RGTX_FLASH.ld
	arm-none-eabi-gcc -o "$(BUILD_ARTIFACT_NAME).elf" $(S_OBJS) $(C_OBJS) -lvl53l5 -mcpu=cortex-m4 -T"$(PROJDIR)/STM32F405RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="$(BUILD_ARTIFACT_NAME).map" -Wl,--gc-sections -static -L"$(PROJDIR)/bare_driver" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

$(BUILD_ARTIFACT_NAME).list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "$(BUILD_ARTIFACT_NAME).list"
	@echo 'Finished building: $@'
	@echo ' '

$(BUILD_ARTIFACT_NAME).hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "$(BUILD_ARTIFACT_NAME).hex"
	@echo 'Finished building: $@'
	@echo ' '

$(BUILD_ARTIFACT_NAME).bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "$(BUILD_ARTIFACT_NAME).bin"
	@echo 'Finished building: $@'
	@echo ' '

$(BUILD_ARTIFACT_NAME).srec: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O srec $(EXECUTABLES) "$(BUILD_ARTIFACT_NAME).srec"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Core
	-@echo ' '
	-$(RM) Drivers
	-@echo ' '
	-$(RM) Middlewares
	-@echo ' '
	-$(RM) bare_driver
	-@echo ' '
	-$(RM) chirp
	-@echo ' '
	-$(RM) $(BUILD_ARTIFACT_NAME).bin
	-@echo ' '
	-$(RM) $(BUILD_ARTIFACT_NAME).elf
	-@echo ' '
	-$(RM) $(BUILD_ARTIFACT_NAME).hex
	-@echo ' '
	-$(RM) $(BUILD_ARTIFACT_NAME).list
	-@echo ' '
	-$(RM) $(BUILD_ARTIFACT_NAME).srec
	-@echo ' '
	-$(RM) $(BUILD_ARTIFACT_NAME).map
	-@echo ' '
secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN) $(OBJCOPY_SREC)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
