################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CVGraphics.cpp \
../DetectRed.cpp \
../HelloWorld.cpp \
../PSODemo.cpp \
../PolynomialFit.cpp \
../ShapeDetect.cpp \
../main.cpp 

OBJS += \
./CVGraphics.o \
./DetectRed.o \
./HelloWorld.o \
./PSODemo.o \
./PolynomialFit.o \
./ShapeDetect.o \
./main.o 

CPP_DEPS += \
./CVGraphics.d \
./DetectRed.d \
./HelloWorld.d \
./PSODemo.d \
./PolynomialFit.d \
./ShapeDetect.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


