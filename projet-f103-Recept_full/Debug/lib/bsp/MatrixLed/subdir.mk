################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../lib/bsp/MatrixLed/WS2812S_asm.s 

C_SRCS += \
../lib/bsp/MatrixLed/WS2812B.c \
../lib/bsp/MatrixLed/WS2812S.c \
../lib/bsp/MatrixLed/matrix_led_32_32.c 

OBJS += \
./lib/bsp/MatrixLed/WS2812B.o \
./lib/bsp/MatrixLed/WS2812S.o \
./lib/bsp/MatrixLed/WS2812S_asm.o \
./lib/bsp/MatrixLed/matrix_led_32_32.o 

C_DEPS += \
./lib/bsp/MatrixLed/WS2812B.d \
./lib/bsp/MatrixLed/WS2812S.d \
./lib/bsp/MatrixLed/matrix_led_32_32.d 


# Each subdirectory must supply rules for building sources it contributes
lib/bsp/MatrixLed/%.o: ../lib/bsp/MatrixLed/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32F1 -DNUCLEO_F103RB -DSTM32F103RBTx -DSTM32 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MPU6050" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/Common" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/hts221" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lis3mdl" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lps22hb" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lps25hb" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lsm6ds0" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lsm6ds3" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/tft_ili9341" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/tft_pcd8544" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MCP23S17" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/hal/inc" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/Common" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/lcd2x16" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MLX90614" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MatrixKeyboard" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MatrixLed" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/CMSIS/core" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/CMSIS/device" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/middleware/FatFs" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/middleware/FatFs/src" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/middleware/FatFs/src/drivers" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/appli"  -Og -g3 -Wall -Wextra -Wconversion -fmessage-length=0 -ffunction-sections -fdata-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lib/bsp/MatrixLed/%.o: ../lib/bsp/MatrixLed/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


