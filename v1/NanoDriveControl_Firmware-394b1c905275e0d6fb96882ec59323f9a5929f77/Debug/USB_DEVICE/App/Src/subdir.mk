################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/App/Src/usb_device.c \
../USB_DEVICE/App/Src/usbd_cdc_if.c \
../USB_DEVICE/App/Src/usbd_desc.c 

OBJS += \
./USB_DEVICE/App/Src/usb_device.o \
./USB_DEVICE/App/Src/usbd_cdc_if.o \
./USB_DEVICE/App/Src/usbd_desc.o 

C_DEPS += \
./USB_DEVICE/App/Src/usb_device.d \
./USB_DEVICE/App/Src/usbd_cdc_if.d \
./USB_DEVICE/App/Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/App/Src/%.o USB_DEVICE/App/Src/%.su USB_DEVICE/App/Src/%.cyclo: ../USB_DEVICE/App/Src/%.c USB_DEVICE/App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Dell Latitude/Desktop/VPL/Co_op/04_USB/NanoDriveControl_Firmware-394b1c905275e0d6fb96882ec59323f9a5929f77/NanoDriveControl_Firmware-394b1c905275e0d6fb96882ec59323f9a5929f77/USB_DEVICE" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_DEVICE-2f-App-2f-Src

clean-USB_DEVICE-2f-App-2f-Src:
	-$(RM) ./USB_DEVICE/App/Src/usb_device.cyclo ./USB_DEVICE/App/Src/usb_device.d ./USB_DEVICE/App/Src/usb_device.o ./USB_DEVICE/App/Src/usb_device.su ./USB_DEVICE/App/Src/usbd_cdc_if.cyclo ./USB_DEVICE/App/Src/usbd_cdc_if.d ./USB_DEVICE/App/Src/usbd_cdc_if.o ./USB_DEVICE/App/Src/usbd_cdc_if.su ./USB_DEVICE/App/Src/usbd_desc.cyclo ./USB_DEVICE/App/Src/usbd_desc.d ./USB_DEVICE/App/Src/usbd_desc.o ./USB_DEVICE/App/Src/usbd_desc.su

.PHONY: clean-USB_DEVICE-2f-App-2f-Src

