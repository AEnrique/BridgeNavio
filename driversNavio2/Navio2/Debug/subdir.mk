################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ADC_Navio2.cpp \
../AHRS.cpp \
../I2Cdev.cpp \
../LSM9DS1.cpp \
../Led_Navio2.cpp \
../MPU9250.cpp \
../MS5611.cpp \
../PWM.cpp \
../RCInput_Navio2.cpp \
../RCOutput_Navio2.cpp \
../RGBled.cpp \
../Ublox.cpp \
../Util.cpp \
../gpio.cpp 

OBJS += \
./ADC_Navio2.o \
./AHRS.o \
./I2Cdev.o \
./LSM9DS1.o \
./Led_Navio2.o \
./MPU9250.o \
./MS5611.o \
./PWM.o \
./RCInput_Navio2.o \
./RCOutput_Navio2.o \
./RGBled.o \
./Ublox.o \
./Util.o \
./gpio.o 

CPP_DEPS += \
./ADC_Navio2.d \
./AHRS.d \
./I2Cdev.d \
./LSM9DS1.d \
./Led_Navio2.d \
./MPU9250.d \
./MS5611.d \
./PWM.d \
./RCInput_Navio2.d \
./RCOutput_Navio2.d \
./RGBled.d \
./Ublox.d \
./Util.d \
./gpio.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


