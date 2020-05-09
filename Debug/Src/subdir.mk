################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/sysmem.c \
../Src/usart_write_read_IT.c 

OBJS += \
./Src/sysmem.o \
./Src/usart_write_read_IT.o 

C_DEPS += \
./Src/sysmem.d \
./Src/usart_write_read_IT.d 


# Each subdirectory must supply rules for building sources it contributes
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Src" -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/usart_write_read_IT.o: ../Src/usart_write_read_IT.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Src" -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/MCU1 Course/Firmware/stm32f446xx/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/usart_write_read_IT.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

