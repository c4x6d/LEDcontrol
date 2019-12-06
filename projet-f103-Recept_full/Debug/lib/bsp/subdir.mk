################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/bsp/stm32f1_adc.c \
../lib/bsp/stm32f1_gpio.c \
../lib/bsp/stm32f1_i2c.c \
../lib/bsp/stm32f1_motorDC.c \
../lib/bsp/stm32f1_pwm.c \
../lib/bsp/stm32f1_rtc.c \
../lib/bsp/stm32f1_sd.c \
../lib/bsp/stm32f1_spi.c \
../lib/bsp/stm32f1_sys.c \
../lib/bsp/stm32f1_timer.c \
../lib/bsp/stm32f1_uart.c \
../lib/bsp/stm32f1_weak_it.c \
../lib/bsp/stm32f1xx_nucleo.c \
../lib/bsp/system_stm32f1xx.c \
../lib/bsp/systick.c 

OBJS += \
./lib/bsp/stm32f1_adc.o \
./lib/bsp/stm32f1_gpio.o \
./lib/bsp/stm32f1_i2c.o \
./lib/bsp/stm32f1_motorDC.o \
./lib/bsp/stm32f1_pwm.o \
./lib/bsp/stm32f1_rtc.o \
./lib/bsp/stm32f1_sd.o \
./lib/bsp/stm32f1_spi.o \
./lib/bsp/stm32f1_sys.o \
./lib/bsp/stm32f1_timer.o \
./lib/bsp/stm32f1_uart.o \
./lib/bsp/stm32f1_weak_it.o \
./lib/bsp/stm32f1xx_nucleo.o \
./lib/bsp/system_stm32f1xx.o \
./lib/bsp/systick.o 

C_DEPS += \
./lib/bsp/stm32f1_adc.d \
./lib/bsp/stm32f1_gpio.d \
./lib/bsp/stm32f1_i2c.d \
./lib/bsp/stm32f1_motorDC.d \
./lib/bsp/stm32f1_pwm.d \
./lib/bsp/stm32f1_rtc.d \
./lib/bsp/stm32f1_sd.d \
./lib/bsp/stm32f1_spi.d \
./lib/bsp/stm32f1_sys.d \
./lib/bsp/stm32f1_timer.d \
./lib/bsp/stm32f1_uart.d \
./lib/bsp/stm32f1_weak_it.d \
./lib/bsp/stm32f1xx_nucleo.d \
./lib/bsp/system_stm32f1xx.d \
./lib/bsp/systick.d 


# Each subdirectory must supply rules for building sources it contributes
lib/bsp/%.o: ../lib/bsp/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32F1 -DNUCLEO_F103RB -DSTM32F103RBTx -DSTM32 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MPU6050" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/Common" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/hts221" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lis3mdl" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lps22hb" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lps25hb" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lsm6ds0" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/iks01a1/lsm6ds3" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/tft_ili9341" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/tft_pcd8544" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MCP23S17" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/hal/inc" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/Common" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/lcd2x16" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MLX90614" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MatrixKeyboard" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/bsp/MatrixLed" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/CMSIS/core" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/CMSIS/device" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/middleware/FatFs" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/middleware/FatFs/src" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/lib/middleware/FatFs/src/drivers" -I"G:/USBFLASHDRIVE/Laptop/EON/projet-f103/appli"  -Og -g3 -Wall -Wextra -Wconversion -fmessage-length=0 -ffunction-sections -fdata-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


