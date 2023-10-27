################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/Target/Src/usbd_conf.c 

OBJS += \
./USB_DEVICE/Target/Src/usbd_conf.o 

C_DEPS += \
./USB_DEVICE/Target/Src/usbd_conf.d 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/Target/Src/%.o USB_DEVICE/Target/Src/%.su USB_DEVICE/Target/Src/%.cyclo: ../USB_DEVICE/Target/Src/%.c USB_DEVICE/Target/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Dell Latitude/Desktop/VPL/Co_op/04_USB/NanoDriveControl_Firmware-394b1c905275e0d6fb96882ec59323f9a5929f77/NanoDriveControl_Firmware-394b1c905275e0d6fb96882ec59323f9a5929f77/USB_DEVICE" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_DEVICE-2f-Target-2f-Src

clean-USB_DEVICE-2f-Target-2f-Src:
	-$(RM) ./USB_DEVICE/Target/Src/usbd_conf.cyclo ./USB_DEVICE/Target/Src/usbd_conf.d ./USB_DEVICE/Target/Src/usbd_conf.o ./USB_DEVICE/Target/Src/usbd_conf.su

.PHONY: clean-USB_DEVICE-2f-Target-2f-Src

