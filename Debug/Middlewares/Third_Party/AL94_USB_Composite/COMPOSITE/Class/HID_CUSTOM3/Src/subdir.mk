################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/usbd_hid_custom3.c 

OBJS += \
./Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/usbd_hid_custom3.o 

C_DEPS += \
./Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/usbd_hid_custom3.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/%.o Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/%.su Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/%.cyclo: ../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/%.c Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Composite -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/COMPOSITE/Inc -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM/Inc -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/App -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Core/Inc -I../Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Target -I"/home/zuidec/programming/stmcube/discostick_cyclic/Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM2" -I"/home/zuidec/programming/stmcube/discostick_cyclic/Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3" -I"/home/zuidec/programming/stmcube/discostick_cyclic/Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM2/Inc" -I"/home/zuidec/programming/stmcube/discostick_cyclic/Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-AL94_USB_Composite-2f-COMPOSITE-2f-Class-2f-HID_CUSTOM3-2f-Src

clean-Middlewares-2f-Third_Party-2f-AL94_USB_Composite-2f-COMPOSITE-2f-Class-2f-HID_CUSTOM3-2f-Src:
	-$(RM) ./Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/usbd_hid_custom3.cyclo ./Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/usbd_hid_custom3.d ./Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/usbd_hid_custom3.o ./Middlewares/Third_Party/AL94_USB_Composite/COMPOSITE/Class/HID_CUSTOM3/Src/usbd_hid_custom3.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-AL94_USB_Composite-2f-COMPOSITE-2f-Class-2f-HID_CUSTOM3-2f-Src

