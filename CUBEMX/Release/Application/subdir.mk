################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/CTRL.c \
../Application/FOC.c \
../Application/current_ADC.c \
../Application/print_serve.c \
../Application/print_server.c 

OBJS += \
./Application/CTRL.o \
./Application/FOC.o \
./Application/current_ADC.o \
./Application/print_serve.o \
./Application/print_server.o 

C_DEPS += \
./Application/CTRL.d \
./Application/FOC.d \
./Application/current_ADC.d \
./Application/print_serve.d \
./Application/print_server.d 


# Each subdirectory must supply rules for building sources it contributes
Application/%.o Application/%.su Application/%.cyclo: ../Application/%.c Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/STM32G4xx_HAL_Driver/Inc -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/CMSIS/Device/ST/STM32G4xx/Include -IC:/Users/Daniel/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application

clean-Application:
	-$(RM) ./Application/CTRL.cyclo ./Application/CTRL.d ./Application/CTRL.o ./Application/CTRL.su ./Application/FOC.cyclo ./Application/FOC.d ./Application/FOC.o ./Application/FOC.su ./Application/current_ADC.cyclo ./Application/current_ADC.d ./Application/current_ADC.o ./Application/current_ADC.su ./Application/print_serve.cyclo ./Application/print_serve.d ./Application/print_serve.o ./Application/print_serve.su ./Application/print_server.cyclo ./Application/print_server.d ./Application/print_server.o ./Application/print_server.su

.PHONY: clean-Application

