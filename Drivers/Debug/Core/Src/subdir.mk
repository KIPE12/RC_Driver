################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Adc.c \
../Core/Src/Can.c \
../Core/Src/CurrentContorl.c \
../Core/Src/Dac.c \
../Core/Src/GlobalVar.c \
../Core/Src/IntDAC.c \
../Core/Src/MainControl.c \
../Core/Src/PosControl.c \
../Core/Src/SpeedControl.c \
../Core/Src/SpeedObserver.c \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/Adc.o \
./Core/Src/Can.o \
./Core/Src/CurrentContorl.o \
./Core/Src/Dac.o \
./Core/Src/GlobalVar.o \
./Core/Src/IntDAC.o \
./Core/Src/MainControl.o \
./Core/Src/PosControl.o \
./Core/Src/SpeedControl.o \
./Core/Src/SpeedObserver.o \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/Adc.d \
./Core/Src/Can.d \
./Core/Src/CurrentContorl.d \
./Core/Src/Dac.d \
./Core/Src/GlobalVar.d \
./Core/Src/IntDAC.d \
./Core/Src/MainControl.d \
./Core/Src/PosControl.d \
./Core/Src/SpeedControl.d \
./Core/Src/SpeedObserver.d \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Adc.cyclo ./Core/Src/Adc.d ./Core/Src/Adc.o ./Core/Src/Adc.su ./Core/Src/Can.cyclo ./Core/Src/Can.d ./Core/Src/Can.o ./Core/Src/Can.su ./Core/Src/CurrentContorl.cyclo ./Core/Src/CurrentContorl.d ./Core/Src/CurrentContorl.o ./Core/Src/CurrentContorl.su ./Core/Src/Dac.cyclo ./Core/Src/Dac.d ./Core/Src/Dac.o ./Core/Src/Dac.su ./Core/Src/GlobalVar.cyclo ./Core/Src/GlobalVar.d ./Core/Src/GlobalVar.o ./Core/Src/GlobalVar.su ./Core/Src/IntDAC.cyclo ./Core/Src/IntDAC.d ./Core/Src/IntDAC.o ./Core/Src/IntDAC.su ./Core/Src/MainControl.cyclo ./Core/Src/MainControl.d ./Core/Src/MainControl.o ./Core/Src/MainControl.su ./Core/Src/PosControl.cyclo ./Core/Src/PosControl.d ./Core/Src/PosControl.o ./Core/Src/PosControl.su ./Core/Src/SpeedControl.cyclo ./Core/Src/SpeedControl.d ./Core/Src/SpeedControl.o ./Core/Src/SpeedControl.su ./Core/Src/SpeedObserver.cyclo ./Core/Src/SpeedObserver.d ./Core/Src/SpeedObserver.o ./Core/Src/SpeedObserver.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

