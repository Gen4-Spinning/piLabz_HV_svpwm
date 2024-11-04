################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Gen4_Drivers/Src/ADC.c \
../Drivers/Gen4_Drivers/Src/AS5x47P.c \
../Drivers/Gen4_Drivers/Src/Calibration.c \
../Drivers/Gen4_Drivers/Src/CoggingFrictionRemoval.c \
../Drivers/Gen4_Drivers/Src/Cordic.c \
../Drivers/Gen4_Drivers/Src/EncoderFns.c \
../Drivers/Gen4_Drivers/Src/FOC.c \
../Drivers/Gen4_Drivers/Src/Logger.c \
../Drivers/Gen4_Drivers/Src/MathOps.c \
../Drivers/Gen4_Drivers/Src/PosControl-JC.c \
../Drivers/Gen4_Drivers/Src/PositionControl.c \
../Drivers/Gen4_Drivers/Src/PositionSensor.c \
../Drivers/Gen4_Drivers/Src/RampDuty.c \
../Drivers/Gen4_Drivers/Src/RampRpm.c \
../Drivers/Gen4_Drivers/Src/SpeedSensor.c 

OBJS += \
./Drivers/Gen4_Drivers/Src/ADC.o \
./Drivers/Gen4_Drivers/Src/AS5x47P.o \
./Drivers/Gen4_Drivers/Src/Calibration.o \
./Drivers/Gen4_Drivers/Src/CoggingFrictionRemoval.o \
./Drivers/Gen4_Drivers/Src/Cordic.o \
./Drivers/Gen4_Drivers/Src/EncoderFns.o \
./Drivers/Gen4_Drivers/Src/FOC.o \
./Drivers/Gen4_Drivers/Src/Logger.o \
./Drivers/Gen4_Drivers/Src/MathOps.o \
./Drivers/Gen4_Drivers/Src/PosControl-JC.o \
./Drivers/Gen4_Drivers/Src/PositionControl.o \
./Drivers/Gen4_Drivers/Src/PositionSensor.o \
./Drivers/Gen4_Drivers/Src/RampDuty.o \
./Drivers/Gen4_Drivers/Src/RampRpm.o \
./Drivers/Gen4_Drivers/Src/SpeedSensor.o 

C_DEPS += \
./Drivers/Gen4_Drivers/Src/ADC.d \
./Drivers/Gen4_Drivers/Src/AS5x47P.d \
./Drivers/Gen4_Drivers/Src/Calibration.d \
./Drivers/Gen4_Drivers/Src/CoggingFrictionRemoval.d \
./Drivers/Gen4_Drivers/Src/Cordic.d \
./Drivers/Gen4_Drivers/Src/EncoderFns.d \
./Drivers/Gen4_Drivers/Src/FOC.d \
./Drivers/Gen4_Drivers/Src/Logger.d \
./Drivers/Gen4_Drivers/Src/MathOps.d \
./Drivers/Gen4_Drivers/Src/PosControl-JC.d \
./Drivers/Gen4_Drivers/Src/PositionControl.d \
./Drivers/Gen4_Drivers/Src/PositionSensor.d \
./Drivers/Gen4_Drivers/Src/RampDuty.d \
./Drivers/Gen4_Drivers/Src/RampRpm.d \
./Drivers/Gen4_Drivers/Src/SpeedSensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Gen4_Drivers/Src/%.o Drivers/Gen4_Drivers/Src/%.su Drivers/Gen4_Drivers/Src/%.cyclo: ../Drivers/Gen4_Drivers/Src/%.c Drivers/Gen4_Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Gen4_Drivers/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Gen4_Drivers-2f-Src

clean-Drivers-2f-Gen4_Drivers-2f-Src:
	-$(RM) ./Drivers/Gen4_Drivers/Src/ADC.cyclo ./Drivers/Gen4_Drivers/Src/ADC.d ./Drivers/Gen4_Drivers/Src/ADC.o ./Drivers/Gen4_Drivers/Src/ADC.su ./Drivers/Gen4_Drivers/Src/AS5x47P.cyclo ./Drivers/Gen4_Drivers/Src/AS5x47P.d ./Drivers/Gen4_Drivers/Src/AS5x47P.o ./Drivers/Gen4_Drivers/Src/AS5x47P.su ./Drivers/Gen4_Drivers/Src/Calibration.cyclo ./Drivers/Gen4_Drivers/Src/Calibration.d ./Drivers/Gen4_Drivers/Src/Calibration.o ./Drivers/Gen4_Drivers/Src/Calibration.su ./Drivers/Gen4_Drivers/Src/CoggingFrictionRemoval.cyclo ./Drivers/Gen4_Drivers/Src/CoggingFrictionRemoval.d ./Drivers/Gen4_Drivers/Src/CoggingFrictionRemoval.o ./Drivers/Gen4_Drivers/Src/CoggingFrictionRemoval.su ./Drivers/Gen4_Drivers/Src/Cordic.cyclo ./Drivers/Gen4_Drivers/Src/Cordic.d ./Drivers/Gen4_Drivers/Src/Cordic.o ./Drivers/Gen4_Drivers/Src/Cordic.su ./Drivers/Gen4_Drivers/Src/EncoderFns.cyclo ./Drivers/Gen4_Drivers/Src/EncoderFns.d ./Drivers/Gen4_Drivers/Src/EncoderFns.o ./Drivers/Gen4_Drivers/Src/EncoderFns.su ./Drivers/Gen4_Drivers/Src/FOC.cyclo ./Drivers/Gen4_Drivers/Src/FOC.d ./Drivers/Gen4_Drivers/Src/FOC.o ./Drivers/Gen4_Drivers/Src/FOC.su ./Drivers/Gen4_Drivers/Src/Logger.cyclo ./Drivers/Gen4_Drivers/Src/Logger.d ./Drivers/Gen4_Drivers/Src/Logger.o ./Drivers/Gen4_Drivers/Src/Logger.su ./Drivers/Gen4_Drivers/Src/MathOps.cyclo ./Drivers/Gen4_Drivers/Src/MathOps.d ./Drivers/Gen4_Drivers/Src/MathOps.o ./Drivers/Gen4_Drivers/Src/MathOps.su ./Drivers/Gen4_Drivers/Src/PosControl-JC.cyclo ./Drivers/Gen4_Drivers/Src/PosControl-JC.d ./Drivers/Gen4_Drivers/Src/PosControl-JC.o ./Drivers/Gen4_Drivers/Src/PosControl-JC.su ./Drivers/Gen4_Drivers/Src/PositionControl.cyclo ./Drivers/Gen4_Drivers/Src/PositionControl.d ./Drivers/Gen4_Drivers/Src/PositionControl.o ./Drivers/Gen4_Drivers/Src/PositionControl.su ./Drivers/Gen4_Drivers/Src/PositionSensor.cyclo ./Drivers/Gen4_Drivers/Src/PositionSensor.d ./Drivers/Gen4_Drivers/Src/PositionSensor.o ./Drivers/Gen4_Drivers/Src/PositionSensor.su ./Drivers/Gen4_Drivers/Src/RampDuty.cyclo ./Drivers/Gen4_Drivers/Src/RampDuty.d ./Drivers/Gen4_Drivers/Src/RampDuty.o ./Drivers/Gen4_Drivers/Src/RampDuty.su ./Drivers/Gen4_Drivers/Src/RampRpm.cyclo ./Drivers/Gen4_Drivers/Src/RampRpm.d ./Drivers/Gen4_Drivers/Src/RampRpm.o ./Drivers/Gen4_Drivers/Src/RampRpm.su ./Drivers/Gen4_Drivers/Src/SpeedSensor.cyclo ./Drivers/Gen4_Drivers/Src/SpeedSensor.d ./Drivers/Gen4_Drivers/Src/SpeedSensor.o ./Drivers/Gen4_Drivers/Src/SpeedSensor.su

.PHONY: clean-Drivers-2f-Gen4_Drivers-2f-Src

