################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
niipnnp.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
niipnnp.list \

OBJCOPY_BIN += \
niipnnp.bin \


# All Target
all: niipnnp.elf secondary-outputs

# Tool invocations
niipnnp.elf: $(OBJS) $(USER_OBJS) C:\Users\Felix\STM32CubeIDE\workspace_1.2.0\niipnnp\STM32G031J6MX_FLASH.ld
	arm-none-eabi-gcc -o "niipnnp.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\Felix\STM32CubeIDE\workspace_1.2.0\niipnnp\STM32G031J6MX_FLASH.ld" --specs=nosys.specs -Wl,-Map="niipnnp.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

niipnnp.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "niipnnp.list"
	@echo 'Finished building: $@'
	@echo ' '

niipnnp.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "niipnnp.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
