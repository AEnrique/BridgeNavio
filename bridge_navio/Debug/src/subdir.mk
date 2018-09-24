################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/serial_comm.cpp \
../src/bridge_drivers_navio.cpp \
../src/main.cpp 

OBJS += \
./src/serial_comm.o \
./src/bridge_drivers_navio.o \
./src/main.o 

CPP_DEPS += \
./src/serial_comm.d \
./src/bridge_drivers_navio.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes

src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++1y -I/usr/local/boost/include -I"/home/pi/Repos/BridgeNavio/bridge_navio/include" -I"/home/pi/Repos/BridgeNavio/Navio2" -I"/home/pi/Repos/BridgeNavio/libshm_navio" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


