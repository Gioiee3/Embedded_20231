################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
OBJS += \
./lib/lcd16x2_BVH/lcd_16x2.o 

C_DEPS += \
./lib/lcd16x2_BVH/lcd_16x2.d 


# Each subdirectory must supply rules for building sources it contributes
lib/lcd16x2_BVH/lcd_16x2.o: D:/STM32/TTKT/lib/lcd16x2_BVH/lcd_16x2.c lib/lcd16x2_BVH/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32/TTKT/lib" -I"D:/STM32/TTKT/lib/lcd16x2_BVH" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-lib-2f-lcd16x2_BVH

clean-lib-2f-lcd16x2_BVH:
	-$(RM) ./lib/lcd16x2_BVH/lcd_16x2.d ./lib/lcd16x2_BVH/lcd_16x2.o ./lib/lcd16x2_BVH/lcd_16x2.su

.PHONY: clean-lib-2f-lcd16x2_BVH

