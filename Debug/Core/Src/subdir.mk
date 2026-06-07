################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_freertos.c \
../Core/Src/bed_heater.c \
../Core/Src/bed_thermistor.c \
../Core/Src/ddr_globals.c \
../Core/Src/endstop.c \
../Core/Src/extruder_test.c \
../Core/Src/fan.c \
../Core/Src/gcodefuncs.c \
../Core/Src/globals_m.c \
../Core/Src/heater.c \
../Core/Src/homing.c \
../Core/Src/main.c \
../Core/Src/motion.c \
../Core/Src/parser.c \
../Core/Src/qbc_globals.c \
../Core/Src/sdcard.c \
../Core/Src/stepper.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_hal_timebase_tim.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/sys_id.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/thermistor.c \
../Core/Src/timebase.c \
../Core/Src/uart_debug.c 

OBJS += \
./Core/Src/app_freertos.o \
./Core/Src/bed_heater.o \
./Core/Src/bed_thermistor.o \
./Core/Src/ddr_globals.o \
./Core/Src/endstop.o \
./Core/Src/extruder_test.o \
./Core/Src/fan.o \
./Core/Src/gcodefuncs.o \
./Core/Src/globals_m.o \
./Core/Src/heater.o \
./Core/Src/homing.o \
./Core/Src/main.o \
./Core/Src/motion.o \
./Core/Src/parser.o \
./Core/Src/qbc_globals.o \
./Core/Src/sdcard.o \
./Core/Src/stepper.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_hal_timebase_tim.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/sys_id.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/thermistor.o \
./Core/Src/timebase.o \
./Core/Src/uart_debug.o 

C_DEPS += \
./Core/Src/app_freertos.d \
./Core/Src/bed_heater.d \
./Core/Src/bed_thermistor.d \
./Core/Src/ddr_globals.d \
./Core/Src/endstop.d \
./Core/Src/extruder_test.d \
./Core/Src/fan.d \
./Core/Src/gcodefuncs.d \
./Core/Src/globals_m.d \
./Core/Src/heater.d \
./Core/Src/homing.d \
./Core/Src/main.d \
./Core/Src/motion.d \
./Core/Src/parser.d \
./Core/Src/qbc_globals.d \
./Core/Src/sdcard.d \
./Core/Src/stepper.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_hal_timebase_tim.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/sys_id.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/thermistor.d \
./Core/Src/timebase.d \
./Core/Src/uart_debug.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx '-DCMSIS_device_header=<stm32g0xx.h>' -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I"C:/Users/Dennis/OneDrive - Texas Tech University/Desktop/QBC_RaiderForge_04_21_26/Drivers/csrc" -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/bed_heater.cyclo ./Core/Src/bed_heater.d ./Core/Src/bed_heater.o ./Core/Src/bed_heater.su ./Core/Src/bed_thermistor.cyclo ./Core/Src/bed_thermistor.d ./Core/Src/bed_thermistor.o ./Core/Src/bed_thermistor.su ./Core/Src/ddr_globals.cyclo ./Core/Src/ddr_globals.d ./Core/Src/ddr_globals.o ./Core/Src/ddr_globals.su ./Core/Src/endstop.cyclo ./Core/Src/endstop.d ./Core/Src/endstop.o ./Core/Src/endstop.su ./Core/Src/extruder_test.cyclo ./Core/Src/extruder_test.d ./Core/Src/extruder_test.o ./Core/Src/extruder_test.su ./Core/Src/fan.cyclo ./Core/Src/fan.d ./Core/Src/fan.o ./Core/Src/fan.su ./Core/Src/gcodefuncs.cyclo ./Core/Src/gcodefuncs.d ./Core/Src/gcodefuncs.o ./Core/Src/gcodefuncs.su ./Core/Src/globals_m.cyclo ./Core/Src/globals_m.d ./Core/Src/globals_m.o ./Core/Src/globals_m.su ./Core/Src/heater.cyclo ./Core/Src/heater.d ./Core/Src/heater.o ./Core/Src/heater.su ./Core/Src/homing.cyclo ./Core/Src/homing.d ./Core/Src/homing.o ./Core/Src/homing.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motion.cyclo ./Core/Src/motion.d ./Core/Src/motion.o ./Core/Src/motion.su ./Core/Src/parser.cyclo ./Core/Src/parser.d ./Core/Src/parser.o ./Core/Src/parser.su ./Core/Src/qbc_globals.cyclo ./Core/Src/qbc_globals.d ./Core/Src/qbc_globals.o ./Core/Src/qbc_globals.su ./Core/Src/sdcard.cyclo ./Core/Src/sdcard.d ./Core/Src/sdcard.o ./Core/Src/sdcard.su ./Core/Src/stepper.cyclo ./Core/Src/stepper.d ./Core/Src/stepper.o ./Core/Src/stepper.su ./Core/Src/stm32g0xx_hal_msp.cyclo ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_hal_timebase_tim.cyclo ./Core/Src/stm32g0xx_hal_timebase_tim.d ./Core/Src/stm32g0xx_hal_timebase_tim.o ./Core/Src/stm32g0xx_hal_timebase_tim.su ./Core/Src/stm32g0xx_it.cyclo ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/sys_id.cyclo ./Core/Src/sys_id.d ./Core/Src/sys_id.o ./Core/Src/sys_id.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.cyclo ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/thermistor.cyclo ./Core/Src/thermistor.d ./Core/Src/thermistor.o ./Core/Src/thermistor.su ./Core/Src/timebase.cyclo ./Core/Src/timebase.d ./Core/Src/timebase.o ./Core/Src/timebase.su ./Core/Src/uart_debug.cyclo ./Core/Src/uart_debug.d ./Core/Src/uart_debug.o ./Core/Src/uart_debug.su

.PHONY: clean-Core-2f-Src

