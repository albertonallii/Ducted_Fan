################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bno055.c \
../Core/Src/bno055_stm32_hal.c \
../Core/Src/config.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/pwm.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/tof.c \
../Core/Src/vl53l1_api.c \
../Core/Src/vl53l1_api_calibration.c \
../Core/Src/vl53l1_api_core.c \
../Core/Src/vl53l1_api_debug.c \
../Core/Src/vl53l1_api_preset_modes.c \
../Core/Src/vl53l1_api_strings.c \
../Core/Src/vl53l1_core.c \
../Core/Src/vl53l1_core_support.c \
../Core/Src/vl53l1_error_strings.c \
../Core/Src/vl53l1_platform.c \
../Core/Src/vl53l1_platform_init.c \
../Core/Src/vl53l1_platform_log.c \
../Core/Src/vl53l1_register_funcs.c \
../Core/Src/vl53l1_silicon_core.c \
../Core/Src/vl53l1_wait.c 

OBJS += \
./Core/Src/bno055.o \
./Core/Src/bno055_stm32_hal.o \
./Core/Src/config.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/pwm.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/tof.o \
./Core/Src/vl53l1_api.o \
./Core/Src/vl53l1_api_calibration.o \
./Core/Src/vl53l1_api_core.o \
./Core/Src/vl53l1_api_debug.o \
./Core/Src/vl53l1_api_preset_modes.o \
./Core/Src/vl53l1_api_strings.o \
./Core/Src/vl53l1_core.o \
./Core/Src/vl53l1_core_support.o \
./Core/Src/vl53l1_error_strings.o \
./Core/Src/vl53l1_platform.o \
./Core/Src/vl53l1_platform_init.o \
./Core/Src/vl53l1_platform_log.o \
./Core/Src/vl53l1_register_funcs.o \
./Core/Src/vl53l1_silicon_core.o \
./Core/Src/vl53l1_wait.o 

C_DEPS += \
./Core/Src/bno055.d \
./Core/Src/bno055_stm32_hal.d \
./Core/Src/config.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/pwm.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/tof.d \
./Core/Src/vl53l1_api.d \
./Core/Src/vl53l1_api_calibration.d \
./Core/Src/vl53l1_api_core.d \
./Core/Src/vl53l1_api_debug.d \
./Core/Src/vl53l1_api_preset_modes.d \
./Core/Src/vl53l1_api_strings.d \
./Core/Src/vl53l1_core.d \
./Core/Src/vl53l1_core_support.d \
./Core/Src/vl53l1_error_strings.d \
./Core/Src/vl53l1_platform.d \
./Core/Src/vl53l1_platform_init.d \
./Core/Src/vl53l1_platform_log.d \
./Core/Src/vl53l1_register_funcs.d \
./Core/Src/vl53l1_silicon_core.d \
./Core/Src/vl53l1_wait.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bno055.cyclo ./Core/Src/bno055.d ./Core/Src/bno055.o ./Core/Src/bno055.su ./Core/Src/bno055_stm32_hal.cyclo ./Core/Src/bno055_stm32_hal.d ./Core/Src/bno055_stm32_hal.o ./Core/Src/bno055_stm32_hal.su ./Core/Src/config.cyclo ./Core/Src/config.d ./Core/Src/config.o ./Core/Src/config.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pwm.cyclo ./Core/Src/pwm.d ./Core/Src/pwm.o ./Core/Src/pwm.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/tof.cyclo ./Core/Src/tof.d ./Core/Src/tof.o ./Core/Src/tof.su ./Core/Src/vl53l1_api.cyclo ./Core/Src/vl53l1_api.d ./Core/Src/vl53l1_api.o ./Core/Src/vl53l1_api.su ./Core/Src/vl53l1_api_calibration.cyclo ./Core/Src/vl53l1_api_calibration.d ./Core/Src/vl53l1_api_calibration.o ./Core/Src/vl53l1_api_calibration.su ./Core/Src/vl53l1_api_core.cyclo ./Core/Src/vl53l1_api_core.d ./Core/Src/vl53l1_api_core.o ./Core/Src/vl53l1_api_core.su ./Core/Src/vl53l1_api_debug.cyclo ./Core/Src/vl53l1_api_debug.d ./Core/Src/vl53l1_api_debug.o ./Core/Src/vl53l1_api_debug.su ./Core/Src/vl53l1_api_preset_modes.cyclo ./Core/Src/vl53l1_api_preset_modes.d ./Core/Src/vl53l1_api_preset_modes.o ./Core/Src/vl53l1_api_preset_modes.su ./Core/Src/vl53l1_api_strings.cyclo ./Core/Src/vl53l1_api_strings.d ./Core/Src/vl53l1_api_strings.o ./Core/Src/vl53l1_api_strings.su ./Core/Src/vl53l1_core.cyclo ./Core/Src/vl53l1_core.d ./Core/Src/vl53l1_core.o ./Core/Src/vl53l1_core.su ./Core/Src/vl53l1_core_support.cyclo ./Core/Src/vl53l1_core_support.d ./Core/Src/vl53l1_core_support.o ./Core/Src/vl53l1_core_support.su ./Core/Src/vl53l1_error_strings.cyclo ./Core/Src/vl53l1_error_strings.d ./Core/Src/vl53l1_error_strings.o ./Core/Src/vl53l1_error_strings.su ./Core/Src/vl53l1_platform.cyclo ./Core/Src/vl53l1_platform.d ./Core/Src/vl53l1_platform.o ./Core/Src/vl53l1_platform.su ./Core/Src/vl53l1_platform_init.cyclo ./Core/Src/vl53l1_platform_init.d ./Core/Src/vl53l1_platform_init.o ./Core/Src/vl53l1_platform_init.su ./Core/Src/vl53l1_platform_log.cyclo ./Core/Src/vl53l1_platform_log.d ./Core/Src/vl53l1_platform_log.o ./Core/Src/vl53l1_platform_log.su ./Core/Src/vl53l1_register_funcs.cyclo ./Core/Src/vl53l1_register_funcs.d ./Core/Src/vl53l1_register_funcs.o ./Core/Src/vl53l1_register_funcs.su ./Core/Src/vl53l1_silicon_core.cyclo ./Core/Src/vl53l1_silicon_core.d ./Core/Src/vl53l1_silicon_core.o ./Core/Src/vl53l1_silicon_core.su ./Core/Src/vl53l1_wait.cyclo ./Core/Src/vl53l1_wait.d ./Core/Src/vl53l1_wait.o ./Core/Src/vl53l1_wait.su

.PHONY: clean-Core-2f-Src

