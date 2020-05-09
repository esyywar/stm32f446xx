################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/irq_led_btn.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/irq_led_btn.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/irq_led_btn.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/irq_led_btn.o: ../Src/irq_led_btn.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/irq_led_btn.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

