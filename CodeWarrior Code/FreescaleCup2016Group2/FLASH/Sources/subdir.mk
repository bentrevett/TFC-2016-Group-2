################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/Accelerometer.c" \
"../Sources/ActiveDifferential.c" \
"../Sources/AutoExposure.c" \
"../Sources/EdgeDetection.c" \
"../Sources/I2C.c" \
"../Sources/LCD.c" \
"../Sources/LED.c" \
"../Sources/LineDetectionDouble.c" \
"../Sources/Probability.c" \
"../Sources/ServoMapping.c" \
"../Sources/SpeedControl.c" \
"../Sources/SpeedSensor.c" \
"../Sources/SteeringControl.c" \
"../Sources/TargetSpeedControl.c" \
"../Sources/Telemetry.c" \
"../Sources/main.c" \

C_SRCS += \
../Sources/Accelerometer.c \
../Sources/ActiveDifferential.c \
../Sources/AutoExposure.c \
../Sources/EdgeDetection.c \
../Sources/I2C.c \
../Sources/LCD.c \
../Sources/LED.c \
../Sources/LineDetectionDouble.c \
../Sources/Probability.c \
../Sources/ServoMapping.c \
../Sources/SpeedControl.c \
../Sources/SpeedSensor.c \
../Sources/SteeringControl.c \
../Sources/TargetSpeedControl.c \
../Sources/Telemetry.c \
../Sources/main.c \

OBJS += \
./Sources/Accelerometer.o \
./Sources/ActiveDifferential.o \
./Sources/AutoExposure.o \
./Sources/EdgeDetection.o \
./Sources/I2C.o \
./Sources/LCD.o \
./Sources/LED.o \
./Sources/LineDetectionDouble.o \
./Sources/Probability.o \
./Sources/ServoMapping.o \
./Sources/SpeedControl.o \
./Sources/SpeedSensor.o \
./Sources/SteeringControl.o \
./Sources/TargetSpeedControl.o \
./Sources/Telemetry.o \
./Sources/main.o \

OBJS_QUOTED += \
"./Sources/Accelerometer.o" \
"./Sources/ActiveDifferential.o" \
"./Sources/AutoExposure.o" \
"./Sources/EdgeDetection.o" \
"./Sources/I2C.o" \
"./Sources/LCD.o" \
"./Sources/LED.o" \
"./Sources/LineDetectionDouble.o" \
"./Sources/Probability.o" \
"./Sources/ServoMapping.o" \
"./Sources/SpeedControl.o" \
"./Sources/SpeedSensor.o" \
"./Sources/SteeringControl.o" \
"./Sources/TargetSpeedControl.o" \
"./Sources/Telemetry.o" \
"./Sources/main.o" \

C_DEPS += \
./Sources/Accelerometer.d \
./Sources/ActiveDifferential.d \
./Sources/AutoExposure.d \
./Sources/EdgeDetection.d \
./Sources/I2C.d \
./Sources/LCD.d \
./Sources/LED.d \
./Sources/LineDetectionDouble.d \
./Sources/Probability.d \
./Sources/ServoMapping.d \
./Sources/SpeedControl.d \
./Sources/SpeedSensor.d \
./Sources/SteeringControl.d \
./Sources/TargetSpeedControl.d \
./Sources/Telemetry.d \
./Sources/main.d \

OBJS_OS_FORMAT += \
./Sources/Accelerometer.o \
./Sources/ActiveDifferential.o \
./Sources/AutoExposure.o \
./Sources/EdgeDetection.o \
./Sources/I2C.o \
./Sources/LCD.o \
./Sources/LED.o \
./Sources/LineDetectionDouble.o \
./Sources/Probability.o \
./Sources/ServoMapping.o \
./Sources/SpeedControl.o \
./Sources/SpeedSensor.o \
./Sources/SteeringControl.o \
./Sources/TargetSpeedControl.o \
./Sources/Telemetry.o \
./Sources/main.o \

C_DEPS_QUOTED += \
"./Sources/Accelerometer.d" \
"./Sources/ActiveDifferential.d" \
"./Sources/AutoExposure.d" \
"./Sources/EdgeDetection.d" \
"./Sources/I2C.d" \
"./Sources/LCD.d" \
"./Sources/LED.d" \
"./Sources/LineDetectionDouble.d" \
"./Sources/Probability.d" \
"./Sources/ServoMapping.d" \
"./Sources/SpeedControl.d" \
"./Sources/SpeedSensor.d" \
"./Sources/SteeringControl.d" \
"./Sources/TargetSpeedControl.d" \
"./Sources/Telemetry.d" \
"./Sources/main.d" \


# Each subdirectory must supply rules for building sources it contributes
Sources/Accelerometer.o: ../Sources/Accelerometer.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/Accelerometer.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/Accelerometer.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/ActiveDifferential.o: ../Sources/ActiveDifferential.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/ActiveDifferential.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/ActiveDifferential.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/AutoExposure.o: ../Sources/AutoExposure.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/AutoExposure.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/AutoExposure.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/EdgeDetection.o: ../Sources/EdgeDetection.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/EdgeDetection.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/EdgeDetection.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/I2C.o: ../Sources/I2C.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/I2C.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/I2C.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/LCD.o: ../Sources/LCD.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/LCD.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/LCD.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/LED.o: ../Sources/LED.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/LED.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/LED.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/LineDetectionDouble.o: ../Sources/LineDetectionDouble.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/LineDetectionDouble.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/LineDetectionDouble.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/Probability.o: ../Sources/Probability.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/Probability.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/Probability.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/ServoMapping.o: ../Sources/ServoMapping.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/ServoMapping.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/ServoMapping.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/SpeedControl.o: ../Sources/SpeedControl.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/SpeedControl.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/SpeedControl.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/SpeedSensor.o: ../Sources/SpeedSensor.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/SpeedSensor.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/SpeedSensor.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/SteeringControl.o: ../Sources/SteeringControl.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/SteeringControl.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/SteeringControl.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TargetSpeedControl.o: ../Sources/TargetSpeedControl.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TargetSpeedControl.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TargetSpeedControl.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/Telemetry.o: ../Sources/Telemetry.c
	@echo 'Building file: $<'
	@echo 'Executing target #15 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/Telemetry.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/Telemetry.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/main.o: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #16 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/main.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/main.o"
	@echo 'Finished building: $<'
	@echo ' '


