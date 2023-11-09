################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/BLDC_FOC.c \
../Application/CTRL.c \
../Application/PID.c \
../Application/current_ADC.c \
../Application/example.c \
../Application/fdcandriver.c 

OBJS += \
./Application/BLDC_FOC.o \
./Application/CTRL.o \
./Application/PID.o \
./Application/current_ADC.o \
./Application/example.o \
./Application/fdcandriver.o 

C_DEPS += \
./Application/BLDC_FOC.d \
./Application/CTRL.d \
./Application/PID.d \
./Application/current_ADC.d \
./Application/example.d \
./Application/fdcandriver.d 


# Each subdirectory must supply rules for building sources it contributes
Application/%.o Application/%.su Application/%.cyclo: ../Application/%.c Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/CMSIS/Include -I"C:/Users/Daniel/Documents/GitHub/BLDC-FOC/CUBEMX/Application" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application

clean-Application:
	-$(RM) ./Application/BLDC_FOC.cyclo ./Application/BLDC_FOC.d ./Application/BLDC_FOC.o ./Application/BLDC_FOC.su ./Application/CTRL.cyclo ./Application/CTRL.d ./Application/CTRL.o ./Application/CTRL.su ./Application/PID.cyclo ./Application/PID.d ./Application/PID.o ./Application/PID.su ./Application/current_ADC.cyclo ./Application/current_ADC.d ./Application/current_ADC.o ./Application/current_ADC.su ./Application/example.cyclo ./Application/example.d ./Application/example.o ./Application/example.su ./Application/fdcandriver.cyclo ./Application/fdcandriver.d ./Application/fdcandriver.o ./Application/fdcandriver.su

.PHONY: clean-Application

