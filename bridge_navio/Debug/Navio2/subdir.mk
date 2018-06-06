################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../../Navio2/ADC_Navio2.cpp \
../../Navio2/AHRS.cpp \
../../Navio2/I2Cdev.cpp \
../../Navio2/LSM9DS1.cpp \
../../Navio2/Led_Navio2.cpp \
../../Navio2/MPU9250.cpp \
../../Navio2/MS5611.cpp \
../../Navio2/PWM.cpp \
../../Navio2/RCInput_Navio2.cpp \
../../Navio2/RCOutput_Navio2.cpp \
../../Navio2/RGBled.cpp \
../../Navio2/Ublox.cpp \
../../Navio2/Util.cpp \
../../Navio2/gpio.cpp 

OBJS += \
./Navio2/ADC_Navio2.o \
./Navio2/AHRS.o \
./Navio2/I2Cdev.o \
./Navio2/LSM9DS1.o \
./Navio2/Led_Navio2.o \
./Navio2/MPU9250.o \
./Navio2/MS5611.o \
./Navio2/PWM.o \
./Navio2/RCInput_Navio2.o \
./Navio2/RCOutput_Navio2.o \
./Navio2/RGBled.o \
./Navio2/Ublox.o \
./Navio2/Util.o \
./Navio2/gpio.o 

CPP_DEPS += \
./Navio2/ADC_Navio2.d \
./Navio2/AHRS.d \
./Navio2/I2Cdev.d \
./Navio2/LSM9DS1.d \
./Navio2/Led_Navio2.d \
./Navio2/MPU9250.d \
./Navio2/MS5611.d \
./Navio2/PWM.d \
./Navio2/RCInput_Navio2.d \
./Navio2/RCOutput_Navio2.d \
./Navio2/RGBled.d \
./Navio2/Ublox.d \
./Navio2/Util.d \
./Navio2/gpio.d 


# Each subdirectory must supply rules for building sources it contributes
Navio2/ADC_Navio2.o: ../../Navio2/ADC_Navio2.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/AHRS.o: ../../Navio2/AHRS.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/I2Cdev.o: ../../Navio2/I2Cdev.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/LSM9DS1.o: ../../Navio2/LSM9DS1.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/Led_Navio2.o: ../../Navio2/Led_Navio2.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/MPU9250.o: ../../Navio2/MPU9250.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/MS5611.o: ../../Navio2/MS5611.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/PWM.o: ../../Navio2/PWM.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/RCInput_Navio2.o: ../../Navio2/RCInput_Navio2.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/RCOutput_Navio2.o: ../../Navio2/RCOutput_Navio2.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/RGBled.o: ../../Navio2/RGBled.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/Ublox.o: ../../Navio2/Ublox.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -Wno-psabi -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/Util.o: ../../Navio2/Util.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Navio2/gpio.o: ../../Navio2/gpio.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"../../libshm_navio" -I"../../Navio2" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


