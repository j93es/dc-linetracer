################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../External\ Library/Src/custom_exception.c \
../External\ Library/Src/custom_filesystem.c \
../External\ Library/Src/custom_flash.c \
../External\ Library/Src/custom_oled.c \
../External\ Library/Src/custom_switch.c 

OBJS += \
./External\ Library/Src/custom_exception.o \
./External\ Library/Src/custom_filesystem.o \
./External\ Library/Src/custom_flash.o \
./External\ Library/Src/custom_oled.o \
./External\ Library/Src/custom_switch.o 

C_DEPS += \
./External\ Library/Src/custom_exception.d \
./External\ Library/Src/custom_filesystem.d \
./External\ Library/Src/custom_flash.d \
./External\ Library/Src/custom_oled.d \
./External\ Library/Src/custom_switch.d 


# Each subdirectory must supply rules for building sources it contributes
External\ Library/Src/custom_exception.o: ../External\ Library/Src/custom_exception.c External\ Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_FULL_LL_DRIVER -DSTM32F411xE -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/External Library/Inc" -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/Main/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"External Library/Src/custom_exception.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
External\ Library/Src/custom_filesystem.o: ../External\ Library/Src/custom_filesystem.c External\ Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_FULL_LL_DRIVER -DSTM32F411xE -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/External Library/Inc" -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/Main/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"External Library/Src/custom_filesystem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
External\ Library/Src/custom_flash.o: ../External\ Library/Src/custom_flash.c External\ Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_FULL_LL_DRIVER -DSTM32F411xE -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/External Library/Inc" -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/Main/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"External Library/Src/custom_flash.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
External\ Library/Src/custom_oled.o: ../External\ Library/Src/custom_oled.c External\ Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_FULL_LL_DRIVER -DSTM32F411xE -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/External Library/Inc" -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/Main/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"External Library/Src/custom_oled.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
External\ Library/Src/custom_switch.o: ../External\ Library/Src/custom_switch.c External\ Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_FULL_LL_DRIVER -DSTM32F411xE -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/External Library/Inc" -I"/Users/j93es/workspace/c/step_tracer_masterCode/Nucleo-Linetracer/Main/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"External Library/Src/custom_switch.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-External-20-Library-2f-Src

clean-External-20-Library-2f-Src:
	-$(RM) ./External\ Library/Src/custom_exception.cyclo ./External\ Library/Src/custom_exception.d ./External\ Library/Src/custom_exception.o ./External\ Library/Src/custom_exception.su ./External\ Library/Src/custom_filesystem.cyclo ./External\ Library/Src/custom_filesystem.d ./External\ Library/Src/custom_filesystem.o ./External\ Library/Src/custom_filesystem.su ./External\ Library/Src/custom_flash.cyclo ./External\ Library/Src/custom_flash.d ./External\ Library/Src/custom_flash.o ./External\ Library/Src/custom_flash.su ./External\ Library/Src/custom_oled.cyclo ./External\ Library/Src/custom_oled.d ./External\ Library/Src/custom_oled.o ./External\ Library/Src/custom_oled.su ./External\ Library/Src/custom_switch.cyclo ./External\ Library/Src/custom_switch.d ./External\ Library/Src/custom_switch.o ./External\ Library/Src/custom_switch.su

.PHONY: clean-External-20-Library-2f-Src

