################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ax25.c \
../Src/cc112x_spi.c \
../Src/cc_tx_Init.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c 

OBJS += \
./Src/ax25.o \
./Src/cc112x_spi.o \
./Src/cc_tx_Init.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o 

C_DEPS += \
./Src/ax25.d \
./Src/cc112x_spi.d \
./Src/cc_tx_Init.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F407xx -I"/home/liknus/git/upsat/upsat-comms-software/upsat-comms-workspace/comms/Inc" -I"/home/liknus/git/upsat/upsat-comms-software/upsat-comms-workspace/comms/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/liknus/git/upsat/upsat-comms-software/upsat-comms-workspace/comms/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/liknus/git/upsat/upsat-comms-software/upsat-comms-workspace/comms/Drivers/CMSIS/Include" -I"/home/liknus/git/upsat/upsat-comms-software/upsat-comms-workspace/comms/Drivers/CMSIS/Device/ST/STM32F4xx/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


