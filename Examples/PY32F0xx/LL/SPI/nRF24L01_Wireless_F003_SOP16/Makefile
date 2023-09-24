##### Project #####

PROJECT			?= app
# The path for generated files
BUILD_DIR		= Build


##### Options #####

# Use LL library instead of HAL, y:yes, n:no
USE_LL_LIB ?= y
# Enable printf float %f support, y:yes, n:no
ENABLE_PRINTF_FLOAT	?= n
# Build with FreeRTOS, y:yes, n:no
USE_FREERTOS	?= n
# Build with CMSIS DSP functions, y:yes, n:no
USE_DSP			?= n
# Build with Waveshare e-paper lib, y:yes, n:no
USE_EPAPER		?= n
# Programmer, jlink or pyocd
FLASH_PROGRM	?= jlink

##### Toolchains #######

#ARM_TOOCHAIN	?= /opt/gcc-arm/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi/bin
#ARM_TOOCHAIN	?= /opt/gcc-arm/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi/bin
ARM_TOOCHAIN	?= /opt/gcc-arm/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin

# path to JLinkExe
JLINKEXE		?= /opt/SEGGER/JLink/JLinkExe
# JLink device type, options:
#   PY32F002AX5,
#   PY32F003X4, PY32F003X6, PY32F003X8,
#   PY32F030X4, PY32F030X6, PY32F030X7, PY32F030X8
JLINK_DEVICE	?= PY32F003X6
# path to PyOCD,
PYOCD_EXE		?= pyocd
# PyOCD device type, options:
# 	py32f002ax5,
#   py32f003x4,  py32f003x6, py32f003x8,
#   py32f030x3,  py32f030x4, py32f030x6, py32f030x7, py32f030x8
#   py32f072xb
PYOCD_DEVICE	?= py32f003x6


##### Paths ############

# Link descript file: py32f002ax5.ld, py32f003x6.ld, py32f003x8.ld, py32f030x6.ld, py32f030x8.ld
LDSCRIPT		= Libraries/LDScripts/py32f003x6.ld
# Library build flags:
#   PY32F002Ax5,
#   PY32F003x4, PY32F003x6, PY32F003x8,
#   PY32F030x3, PY32F030x4, PY32F030x6, PY32F030x7, PY32F030x8,
#   PY32F072xB
LIB_FLAGS       = PY32F003x6

# C source folders
CDIRS	:= User \
		Libraries/CMSIS/Device/PY32F0xx/Source
# C source files (if there are any single ones)
CFILES :=

# ASM source folders
ADIRS	:= User
# ASM single files
AFILES	:= Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f003.s

# Include paths
INCLUDES	:= Libraries/CMSIS/Core/Include \
			Libraries/CMSIS/Device/PY32F0xx/Include \
			User

ifeq ($(USE_LL_LIB),y)
CDIRS		+= Libraries/PY32F0xx_LL_Driver/Src \
		Libraries/BSP_LL/Src
INCLUDES	+= Libraries/PY32F0xx_LL_Driver/Inc \
		Libraries/BSP_LL/Inc
LIB_FLAGS   += USE_FULL_LL_DRIVER
else
CDIRS		+= Libraries/PY32F0xx_HAL_Driver/Src \
		Libraries/BSP/Src
INCLUDES	+= Libraries/PY32F0xx_HAL_Driver/Inc \
		Libraries/BSP/Inc
endif

ifeq ($(USE_FREERTOS),y)
CDIRS		+= Libraries/FreeRTOS \
			Libraries/FreeRTOS/portable/GCC/ARM_CM0

CFILES		+= Libraries/FreeRTOS/portable/MemMang/heap_4.c

INCLUDES	+= Libraries/FreeRTOS/include \
			Libraries/FreeRTOS/portable/GCC/ARM_CM0
endif

ifeq ($(USE_DSP),y)
CFILES 		+= Libraries/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c \
		Libraries/CMSIS/DSP/Source/BayesFunctions/BayesFunctions.c \
		Libraries/CMSIS/DSP/Source/CommonTables/CommonTables.c \
		Libraries/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c \
		Libraries/CMSIS/DSP/Source/ControllerFunctions/ControllerFunctions.c \
		Libraries/CMSIS/DSP/Source/DistanceFunctions/DistanceFunctions.c \
		Libraries/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c \
		Libraries/CMSIS/DSP/Source/FilteringFunctions/FilteringFunctions.c \
		Libraries/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.c \
		Libraries/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c \
		Libraries/CMSIS/DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c \
		Libraries/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c \
		Libraries/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.c \
		Libraries/CMSIS/DSP/Source/SVMFunctions/SVMFunctions.c \
		Libraries/CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c
INCLUDES	+= Libraries/CMSIS/DSP/Include \
		Libraries/CMSIS/DSP/PrivateInclude
endif

ifeq ($(USE_EPAPER),y)
CDIRS		+= Libraries/EPaper/Lib \
			Libraries/EPaper/Examples \
			Libraries/EPaper/Fonts \
			Libraries/EPaper/GUI

INCLUDES	+= Libraries/EPaper/Lib \
			Libraries/EPaper/Examples \
			Libraries/EPaper/Fonts \
			Libraries/EPaper/GUI
endif

include ./rules.mk
