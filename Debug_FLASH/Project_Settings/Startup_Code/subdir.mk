################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
C:/NXP/S32DS.3.5/S32DS/software/S32SDK_S32K1XX_RTM_4.0.1/platform/devices/S32K146/startup/gcc/startup_S32K146.S 

OBJS += \
./Project_Settings/Startup_Code/startup_S32K146.o 


# Each subdirectory must supply rules for building sources it contributes
Project_Settings/Startup_Code/startup_S32K146.o: C:/NXP/S32DS.3.5/S32DS/software/S32SDK_S32K1XX_RTM_4.0.1/platform/devices/S32K146/startup/gcc/startup_S32K146.S
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS Assembler'
	arm-none-eabi-gcc "@Project_Settings/Startup_Code/startup_S32K146.args" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


