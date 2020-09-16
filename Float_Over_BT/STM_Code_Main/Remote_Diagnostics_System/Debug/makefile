################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
Remote_Diagnostics_System.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Remote_Diagnostics_System.list \

OBJCOPY_HEX += \
Remote_Diagnostics_System.hex \

OBJCOPY_BIN += \
Remote_Diagnostics_System.bin \


# All Target
all: Remote_Diagnostics_System.elf secondary-outputs

# Tool invocations
Remote_Diagnostics_System.elf: $(OBJS) $(USER_OBJS) C:\ARM_Toolchain\Diagnostix_EEVV\Remote_Diagnostics_System\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Remote_Diagnostics_System.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\ARM_Toolchain\Diagnostix_EEVV\Remote_Diagnostics_System\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Remote_Diagnostics_System.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Remote_Diagnostics_System.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Remote_Diagnostics_System.list"
	@echo 'Finished building: $@'
	@echo ' '

Remote_Diagnostics_System.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Remote_Diagnostics_System.hex"
	@echo 'Finished building: $@'
	@echo ' '

Remote_Diagnostics_System.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Remote_Diagnostics_System.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
