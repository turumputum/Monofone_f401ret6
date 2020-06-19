################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
usb-i2s-3WORK_USB.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
usb-i2s-3WORK_USB.list \

OBJCOPY_HEX += \
usb-i2s-3WORK_USB.hex \

OBJCOPY_BIN += \
usb-i2s-3WORK_USB.bin \


# All Target
all: usb-i2s-3WORK_USB.elf secondary-outputs

# Tool invocations
usb-i2s-3WORK_USB.elf: $(OBJS) $(USER_OBJS) C:\Users\Yac_not\Desktop\usb-i2s-4\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "usb-i2s-3WORK_USB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Yac_not\Desktop\usb-i2s-4\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="usb-i2s-3WORK_USB.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

usb-i2s-3WORK_USB.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "usb-i2s-3WORK_USB.list"
	@echo 'Finished building: $@'
	@echo ' '

usb-i2s-3WORK_USB.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "usb-i2s-3WORK_USB.hex"
	@echo 'Finished building: $@'
	@echo ' '

usb-i2s-3WORK_USB.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "usb-i2s-3WORK_USB.bin"
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
