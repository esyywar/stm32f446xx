################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/led_button.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/led_button.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/led_button.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/led_button.o: ../Src/led_button.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Src" -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/led_button.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Src" -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Src" -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

