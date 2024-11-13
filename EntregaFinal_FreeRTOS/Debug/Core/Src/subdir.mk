################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/EngTrModel.c \
../Core/Src/EngTrModel_data.c \
../Core/Src/delay.c \
../Core/Src/freertos.c \
../Core/Src/keypad.c \
../Core/Src/lcd.c \
../Core/Src/main.c \
../Core/Src/rtGetInf.c \
../Core/Src/rtGetNaN.c \
../Core/Src/rt_nonfinite.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_hal_timebase_tim.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/uart.c 

OBJS += \
./Core/Src/EngTrModel.o \
./Core/Src/EngTrModel_data.o \
./Core/Src/delay.o \
./Core/Src/freertos.o \
./Core/Src/keypad.o \
./Core/Src/lcd.o \
./Core/Src/main.o \
./Core/Src/rtGetInf.o \
./Core/Src/rtGetNaN.o \
./Core/Src/rt_nonfinite.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_hal_timebase_tim.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/uart.o 

C_DEPS += \
./Core/Src/EngTrModel.d \
./Core/Src/EngTrModel_data.d \
./Core/Src/delay.d \
./Core/Src/freertos.d \
./Core/Src/keypad.d \
./Core/Src/lcd.d \
./Core/Src/main.d \
./Core/Src/rtGetInf.d \
./Core/Src/rtGetNaN.d \
./Core/Src/rt_nonfinite.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_hal_timebase_tim.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/EngTrModel.cyclo ./Core/Src/EngTrModel.d ./Core/Src/EngTrModel.o ./Core/Src/EngTrModel.su ./Core/Src/EngTrModel_data.cyclo ./Core/Src/EngTrModel_data.d ./Core/Src/EngTrModel_data.o ./Core/Src/EngTrModel_data.su ./Core/Src/delay.cyclo ./Core/Src/delay.d ./Core/Src/delay.o ./Core/Src/delay.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/keypad.cyclo ./Core/Src/keypad.d ./Core/Src/keypad.o ./Core/Src/keypad.su ./Core/Src/lcd.cyclo ./Core/Src/lcd.d ./Core/Src/lcd.o ./Core/Src/lcd.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/rtGetInf.cyclo ./Core/Src/rtGetInf.d ./Core/Src/rtGetInf.o ./Core/Src/rtGetInf.su ./Core/Src/rtGetNaN.cyclo ./Core/Src/rtGetNaN.d ./Core/Src/rtGetNaN.o ./Core/Src/rtGetNaN.su ./Core/Src/rt_nonfinite.cyclo ./Core/Src/rt_nonfinite.d ./Core/Src/rt_nonfinite.o ./Core/Src/rt_nonfinite.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_hal_timebase_tim.cyclo ./Core/Src/stm32f1xx_hal_timebase_tim.d ./Core/Src/stm32f1xx_hal_timebase_tim.o ./Core/Src/stm32f1xx_hal_timebase_tim.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/uart.cyclo ./Core/Src/uart.d ./Core/Src/uart.o ./Core/Src/uart.su

.PHONY: clean-Core-2f-Src

