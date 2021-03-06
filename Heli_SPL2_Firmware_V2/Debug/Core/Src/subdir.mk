################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/stm32f2xx_hal_msp.c \
../Core/Src/stm32f2xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f2xx.c 

CPP_SRCS += \
../Core/Src/I2Cdev.cpp \
../Core/Src/MPU6050.cpp \
../Core/Src/MPU6050_6Axis_MotionApps20.cpp \
../Core/Src/PID.cpp \
../Core/Src/SBUS.cpp \
../Core/Src/human_interface.cpp \
../Core/Src/main.cpp \
../Core/Src/motorControl.cpp \
../Core/Src/realMain.cpp 

C_DEPS += \
./Core/Src/stm32f2xx_hal_msp.d \
./Core/Src/stm32f2xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f2xx.d 

OBJS += \
./Core/Src/I2Cdev.o \
./Core/Src/MPU6050.o \
./Core/Src/MPU6050_6Axis_MotionApps20.o \
./Core/Src/PID.o \
./Core/Src/SBUS.o \
./Core/Src/human_interface.o \
./Core/Src/main.o \
./Core/Src/motorControl.o \
./Core/Src/realMain.o \
./Core/Src/stm32f2xx_hal_msp.o \
./Core/Src/stm32f2xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f2xx.o 

CPP_DEPS += \
./Core/Src/I2Cdev.d \
./Core/Src/MPU6050.d \
./Core/Src/MPU6050_6Axis_MotionApps20.d \
./Core/Src/PID.d \
./Core/Src/SBUS.d \
./Core/Src/human_interface.d \
./Core/Src/main.d \
./Core/Src/motorControl.d \
./Core/Src/realMain.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F205xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F205xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/I2Cdev.d ./Core/Src/I2Cdev.o ./Core/Src/MPU6050.d ./Core/Src/MPU6050.o ./Core/Src/MPU6050_6Axis_MotionApps20.d ./Core/Src/MPU6050_6Axis_MotionApps20.o ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/SBUS.d ./Core/Src/SBUS.o ./Core/Src/human_interface.d ./Core/Src/human_interface.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/motorControl.d ./Core/Src/motorControl.o ./Core/Src/realMain.d ./Core/Src/realMain.o ./Core/Src/stm32f2xx_hal_msp.d ./Core/Src/stm32f2xx_hal_msp.o ./Core/Src/stm32f2xx_it.d ./Core/Src/stm32f2xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f2xx.d ./Core/Src/system_stm32f2xx.o

.PHONY: clean-Core-2f-Src

