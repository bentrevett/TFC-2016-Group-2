################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/TFC/TFC_ADC.c" \
"../Sources/TFC/TFC_ARM_SysTick.c" \
"../Sources/TFC/TFC_BoardSupport.c" \
"../Sources/TFC/TFC_CrystalClock.c" \
"../Sources/TFC/TFC_LineScanCamera.c" \
"../Sources/TFC/TFC_Motor.c" \
"../Sources/TFC/TFC_Queue.c" \
"../Sources/TFC/TFC_Servo.c" \
"../Sources/TFC/TFC_Terminal.c" \
"../Sources/TFC/TFC_UART.c" \
"../Sources/TFC/TFC_arm_cm0.c" \

C_SRCS += \
../Sources/TFC/TFC_ADC.c \
../Sources/TFC/TFC_ARM_SysTick.c \
../Sources/TFC/TFC_BoardSupport.c \
../Sources/TFC/TFC_CrystalClock.c \
../Sources/TFC/TFC_LineScanCamera.c \
../Sources/TFC/TFC_Motor.c \
../Sources/TFC/TFC_Queue.c \
../Sources/TFC/TFC_Servo.c \
../Sources/TFC/TFC_Terminal.c \
../Sources/TFC/TFC_UART.c \
../Sources/TFC/TFC_arm_cm0.c \

OBJS += \
./Sources/TFC/TFC_ADC.o \
./Sources/TFC/TFC_ARM_SysTick.o \
./Sources/TFC/TFC_BoardSupport.o \
./Sources/TFC/TFC_CrystalClock.o \
./Sources/TFC/TFC_LineScanCamera.o \
./Sources/TFC/TFC_Motor.o \
./Sources/TFC/TFC_Queue.o \
./Sources/TFC/TFC_Servo.o \
./Sources/TFC/TFC_Terminal.o \
./Sources/TFC/TFC_UART.o \
./Sources/TFC/TFC_arm_cm0.o \

OBJS_QUOTED += \
"./Sources/TFC/TFC_ADC.o" \
"./Sources/TFC/TFC_ARM_SysTick.o" \
"./Sources/TFC/TFC_BoardSupport.o" \
"./Sources/TFC/TFC_CrystalClock.o" \
"./Sources/TFC/TFC_LineScanCamera.o" \
"./Sources/TFC/TFC_Motor.o" \
"./Sources/TFC/TFC_Queue.o" \
"./Sources/TFC/TFC_Servo.o" \
"./Sources/TFC/TFC_Terminal.o" \
"./Sources/TFC/TFC_UART.o" \
"./Sources/TFC/TFC_arm_cm0.o" \

C_DEPS += \
./Sources/TFC/TFC_ADC.d \
./Sources/TFC/TFC_ARM_SysTick.d \
./Sources/TFC/TFC_BoardSupport.d \
./Sources/TFC/TFC_CrystalClock.d \
./Sources/TFC/TFC_LineScanCamera.d \
./Sources/TFC/TFC_Motor.d \
./Sources/TFC/TFC_Queue.d \
./Sources/TFC/TFC_Servo.d \
./Sources/TFC/TFC_Terminal.d \
./Sources/TFC/TFC_UART.d \
./Sources/TFC/TFC_arm_cm0.d \

OBJS_OS_FORMAT += \
./Sources/TFC/TFC_ADC.o \
./Sources/TFC/TFC_ARM_SysTick.o \
./Sources/TFC/TFC_BoardSupport.o \
./Sources/TFC/TFC_CrystalClock.o \
./Sources/TFC/TFC_LineScanCamera.o \
./Sources/TFC/TFC_Motor.o \
./Sources/TFC/TFC_Queue.o \
./Sources/TFC/TFC_Servo.o \
./Sources/TFC/TFC_Terminal.o \
./Sources/TFC/TFC_UART.o \
./Sources/TFC/TFC_arm_cm0.o \

C_DEPS_QUOTED += \
"./Sources/TFC/TFC_ADC.d" \
"./Sources/TFC/TFC_ARM_SysTick.d" \
"./Sources/TFC/TFC_BoardSupport.d" \
"./Sources/TFC/TFC_CrystalClock.d" \
"./Sources/TFC/TFC_LineScanCamera.d" \
"./Sources/TFC/TFC_Motor.d" \
"./Sources/TFC/TFC_Queue.d" \
"./Sources/TFC/TFC_Servo.d" \
"./Sources/TFC/TFC_Terminal.d" \
"./Sources/TFC/TFC_UART.d" \
"./Sources/TFC/TFC_arm_cm0.d" \


# Each subdirectory must supply rules for building sources it contributes
Sources/TFC/TFC_ADC.o: ../Sources/TFC/TFC_ADC.c
	@echo 'Building file: $<'
	@echo 'Executing target #17 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_ADC.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_ADC.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_ARM_SysTick.o: ../Sources/TFC/TFC_ARM_SysTick.c
	@echo 'Building file: $<'
	@echo 'Executing target #18 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_ARM_SysTick.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_ARM_SysTick.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_BoardSupport.o: ../Sources/TFC/TFC_BoardSupport.c
	@echo 'Building file: $<'
	@echo 'Executing target #19 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_BoardSupport.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_BoardSupport.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_CrystalClock.o: ../Sources/TFC/TFC_CrystalClock.c
	@echo 'Building file: $<'
	@echo 'Executing target #20 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_CrystalClock.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_CrystalClock.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_LineScanCamera.o: ../Sources/TFC/TFC_LineScanCamera.c
	@echo 'Building file: $<'
	@echo 'Executing target #21 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_LineScanCamera.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_LineScanCamera.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_Motor.o: ../Sources/TFC/TFC_Motor.c
	@echo 'Building file: $<'
	@echo 'Executing target #22 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_Motor.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_Motor.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_Queue.o: ../Sources/TFC/TFC_Queue.c
	@echo 'Building file: $<'
	@echo 'Executing target #23 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_Queue.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_Queue.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_Servo.o: ../Sources/TFC/TFC_Servo.c
	@echo 'Building file: $<'
	@echo 'Executing target #24 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_Servo.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_Servo.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_Terminal.o: ../Sources/TFC/TFC_Terminal.c
	@echo 'Building file: $<'
	@echo 'Executing target #25 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_Terminal.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_Terminal.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_UART.o: ../Sources/TFC/TFC_UART.c
	@echo 'Building file: $<'
	@echo 'Executing target #26 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_UART.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_UART.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/TFC/TFC_arm_cm0.o: ../Sources/TFC/TFC_arm_cm0.c
	@echo 'Building file: $<'
	@echo 'Executing target #27 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/TFC/TFC_arm_cm0.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/TFC/TFC_arm_cm0.o"
	@echo 'Finished building: $<'
	@echo ' '


