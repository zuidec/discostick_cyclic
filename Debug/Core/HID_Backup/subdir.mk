################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HID_Backup/backup_usbd_hid.c 

OBJS += \
./Core/HID_Backup/backup_usbd_hid.o 

C_DEPS += \
./Core/HID_Backup/backup_usbd_hid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/HID_Backup/%.o Core/HID_Backup/%.su Core/HID_Backup/%.cyclo: ../Core/HID_Backup/%.c Core/HID_Backup/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Composite -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/COMPOSITE/Inc -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM/Inc -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/App -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Core/Inc -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Target -I"/home/zuidec/programming/stmcube/discostick_cyclic/Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM2" -I"/home/zuidec/programming/stmcube/discostick_cyclic/Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM2/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-HID_Backup

clean-Core-2f-HID_Backup:
	-$(RM) ./Core/HID_Backup/backup_usbd_hid.cyclo ./Core/HID_Backup/backup_usbd_hid.d ./Core/HID_Backup/backup_usbd_hid.o ./Core/HID_Backup/backup_usbd_hid.su

.PHONY: clean-Core-2f-HID_Backup

