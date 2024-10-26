################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ina219/Src/ina219.c 

OBJS += \
./ina219/Src/ina219.o 

C_DEPS += \
./ina219/Src/ina219.d 


# Each subdirectory must supply rules for building sources it contributes
ina219/Src/%.o ina219/Src/%.su ina219/Src/%.cyclo: ../ina219/Src/%.c ina219/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/KIZAGAN/INA219_Current_Sensor/ina219/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ina219-2f-Src

clean-ina219-2f-Src:
	-$(RM) ./ina219/Src/ina219.cyclo ./ina219/Src/ina219.d ./ina219/Src/ina219.o ./ina219/Src/ina219.su

.PHONY: clean-ina219-2f-Src

