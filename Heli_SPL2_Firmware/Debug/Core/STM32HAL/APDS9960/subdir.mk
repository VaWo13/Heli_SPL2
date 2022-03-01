################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/STM32HAL/APDS9960/APDS9960.c 

C_DEPS += \
./Core/STM32HAL/APDS9960/APDS9960.d 

OBJS += \
./Core/STM32HAL/APDS9960/APDS9960.o 


# Each subdirectory must supply rules for building sources it contributes
Core/STM32HAL/APDS9960/%.o: ../Core/STM32HAL/APDS9960/%.c Core/STM32HAL/APDS9960/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F205xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-STM32HAL-2f-APDS9960

clean-Core-2f-STM32HAL-2f-APDS9960:
	-$(RM) ./Core/STM32HAL/APDS9960/APDS9960.d ./Core/STM32HAL/APDS9960/APDS9960.o

.PHONY: clean-Core-2f-STM32HAL-2f-APDS9960

