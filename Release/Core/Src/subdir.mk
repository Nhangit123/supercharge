################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/app.c \
../Core/Src/at24c08c.c \
../Core/Src/bypass.c \
../Core/Src/digital_PI_rl_V2.c \
../Core/Src/main.c \
../Core/Src/modbus.c \
../Core/Src/modbus_crc.c \
../Core/Src/starting.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/timmer.c \
../Core/Src/winform.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/app.o \
./Core/Src/at24c08c.o \
./Core/Src/bypass.o \
./Core/Src/digital_PI_rl_V2.o \
./Core/Src/main.o \
./Core/Src/modbus.o \
./Core/Src/modbus_crc.o \
./Core/Src/starting.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/timmer.o \
./Core/Src/winform.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/app.d \
./Core/Src/at24c08c.d \
./Core/Src/bypass.d \
./Core/Src/digital_PI_rl_V2.d \
./Core/Src/main.d \
./Core/Src/modbus.d \
./Core/Src/modbus_crc.d \
./Core/Src/starting.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/timmer.d \
./Core/Src/winform.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/app.cyclo ./Core/Src/app.d ./Core/Src/app.o ./Core/Src/app.su ./Core/Src/at24c08c.cyclo ./Core/Src/at24c08c.d ./Core/Src/at24c08c.o ./Core/Src/at24c08c.su ./Core/Src/bypass.cyclo ./Core/Src/bypass.d ./Core/Src/bypass.o ./Core/Src/bypass.su ./Core/Src/digital_PI_rl_V2.cyclo ./Core/Src/digital_PI_rl_V2.d ./Core/Src/digital_PI_rl_V2.o ./Core/Src/digital_PI_rl_V2.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/modbus.cyclo ./Core/Src/modbus.d ./Core/Src/modbus.o ./Core/Src/modbus.su ./Core/Src/modbus_crc.cyclo ./Core/Src/modbus_crc.d ./Core/Src/modbus_crc.o ./Core/Src/modbus_crc.su ./Core/Src/starting.cyclo ./Core/Src/starting.d ./Core/Src/starting.o ./Core/Src/starting.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/timmer.cyclo ./Core/Src/timmer.d ./Core/Src/timmer.o ./Core/Src/timmer.su ./Core/Src/winform.cyclo ./Core/Src/winform.d ./Core/Src/winform.o ./Core/Src/winform.su

.PHONY: clean-Core-2f-Src

