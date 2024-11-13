################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/EngTrModel.c \
../Src/EngTrModel_data.c \
../Src/keypad.c \
../Src/lcd.c \
../Src/main.c \
../Src/rtGetInf.c \
../Src/rtGetNaN.c \
../Src/rt_nonfinite.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/timers.c \
../Src/uart.c 

OBJS += \
./Src/EngTrModel.o \
./Src/EngTrModel_data.o \
./Src/keypad.o \
./Src/lcd.o \
./Src/main.o \
./Src/rtGetInf.o \
./Src/rtGetNaN.o \
./Src/rt_nonfinite.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/timers.o \
./Src/uart.o 

C_DEPS += \
./Src/EngTrModel.d \
./Src/EngTrModel_data.d \
./Src/keypad.d \
./Src/lcd.d \
./Src/main.d \
./Src/rtGetInf.d \
./Src/rtGetNaN.d \
./Src/rt_nonfinite.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/timers.d \
./Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/EngTrModel.cyclo ./Src/EngTrModel.d ./Src/EngTrModel.o ./Src/EngTrModel.su ./Src/EngTrModel_data.cyclo ./Src/EngTrModel_data.d ./Src/EngTrModel_data.o ./Src/EngTrModel_data.su ./Src/keypad.cyclo ./Src/keypad.d ./Src/keypad.o ./Src/keypad.su ./Src/lcd.cyclo ./Src/lcd.d ./Src/lcd.o ./Src/lcd.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/rtGetInf.cyclo ./Src/rtGetInf.d ./Src/rtGetInf.o ./Src/rtGetInf.su ./Src/rtGetNaN.cyclo ./Src/rtGetNaN.d ./Src/rtGetNaN.o ./Src/rtGetNaN.su ./Src/rt_nonfinite.cyclo ./Src/rt_nonfinite.d ./Src/rt_nonfinite.o ./Src/rt_nonfinite.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/timers.cyclo ./Src/timers.d ./Src/timers.o ./Src/timers.su ./Src/uart.cyclo ./Src/uart.d ./Src/uart.o ./Src/uart.su

.PHONY: clean-Src

