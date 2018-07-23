################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/driversNavio2.cpp 

OBJS += \
./src/driversNavio2.o 

CPP_DEPS += \
./src/driversNavio2.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"/home/pi/Repos/BridgeNavio/libshm_navio" -I"/home/pi/Repos/BridgeNavio/Navio2" -I"/home/pi/Repos/BridgeNavio/driversNavio2/include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


