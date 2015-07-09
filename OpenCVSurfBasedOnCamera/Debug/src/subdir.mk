################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/main.cpp 

OBJS += \
./src/main.o 

CPP_DEPS += \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-6.5/bin/nvcc -G -g -O0 -ccbin arm-linux-gnueabihf-g++ -gencode arch=compute_32,code=sm_32 --target-cpu-architecture ARM -m32 -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-6.5/bin/nvcc -G -g -O0 --compile --target-cpu-architecture ARM -m32 -ccbin arm-linux-gnueabihf-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


