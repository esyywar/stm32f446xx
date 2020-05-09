################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f446xx_gpio_driver.c 

OBJS += \
./Drivers/Src/stm32f446xx_gpio_driver.o 

C_DEPS += \
./Drivers/Src/stm32f446xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f446xx_gpio_driver.o: ../Drivers/Src/stm32f446xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f446xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

