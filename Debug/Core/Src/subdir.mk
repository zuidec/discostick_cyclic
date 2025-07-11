################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bitutils.c \
../Core/Src/calibration.c \
../Core/Src/com_packet.c \
../Core/Src/fifo.c \
../Core/Src/input_filter.c \
../Core/Src/main.c \
../Core/Src/stepper.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/uart.c \
../Core/Src/w25q16jv.c 

OBJS += \
./Core/Src/bitutils.o \
./Core/Src/calibration.o \
./Core/Src/com_packet.o \
./Core/Src/fifo.o \
./Core/Src/input_filter.o \
./Core/Src/main.o \
./Core/Src/stepper.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/uart.o \
./Core/Src/w25q16jv.o 

C_DEPS += \
./Core/Src/bitutils.d \
./Core/Src/calibration.d \
./Core/Src/com_packet.d \
./Core/Src/fifo.d \
./Core/Src/input_filter.d \
./Core/Src/main.d \
./Core/Src/stepper.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/uart.d \
./Core/Src/w25q16jv.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bitutils.cyclo ./Core/Src/bitutils.d ./Core/Src/bitutils.o ./Core/Src/bitutils.su ./Core/Src/calibration.cyclo ./Core/Src/calibration.d ./Core/Src/calibration.o ./Core/Src/calibration.su ./Core/Src/com_packet.cyclo ./Core/Src/com_packet.d ./Core/Src/com_packet.o ./Core/Src/com_packet.su ./Core/Src/fifo.cyclo ./Core/Src/fifo.d ./Core/Src/fifo.o ./Core/Src/fifo.su ./Core/Src/input_filter.cyclo ./Core/Src/input_filter.d ./Core/Src/input_filter.o ./Core/Src/input_filter.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stepper.cyclo ./Core/Src/stepper.d ./Core/Src/stepper.o ./Core/Src/stepper.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/uart.cyclo ./Core/Src/uart.d ./Core/Src/uart.o ./Core/Src/uart.su ./Core/Src/w25q16jv.cyclo ./Core/Src/w25q16jv.d ./Core/Src/w25q16jv.o ./Core/Src/w25q16jv.su

.PHONY: clean-Core-2f-Src

