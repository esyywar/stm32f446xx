################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc_read.c \
../Src/sysmem.c 

OBJS += \
./Src/adc_read.o \
./Src/sysmem.o 

C_DEPS += \
./Src/adc_read.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/adc_read.o: ../Src/adc_read.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/stm32CubeIDE/stm32f446xx/Drivers/Src" -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/stm32CubeIDE/stm32f446xx/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/adc_read.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -c -I../Inc -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/stm32CubeIDE/stm32f446xx/Drivers/Src" -I"C:/Users/rahul/Desktop/Dev_Projects/Embedded/stm32CubeIDE/stm32f446xx/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

