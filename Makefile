##### Project #####

PROJECT			?= app
# The path for generated files
BUILD_DIR		= Build


##### Options #####

# Use LL library instead of HAL
USE_LL_LIB ?= n
# Enable printf float %f support, y:yes, n:no
ENABLE_PRINTF_FLOAT	?= n
# Build with CMSIS DSP functions, y:yes, n:no
USE_DSP			?= n
# Programmer, jlink or pyocd
FLASH_PROGRM	?= pyocd

##### Toolchains #######

#ARM_TOOCHAIN	?= /opt/gcc-arm/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi/bin
#ARM_TOOCHAIN	?= /opt/gcc-arm/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi/bin
ARM_TOOCHAIN	?= /opt/gcc-arm/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin

# path to JLinkExe
JLINKEXE		?= /opt/SEGGER/JLink/JLinkExe
# JLink device type, options: PY32F003X4, PY32F003X6, PY32F003X8, PY32F030X4, PY32F030X6, PY32F030X7, PY32F030X8
JLINK_DEVICE	?= PY32F003X8
# path to PyOCD, 
PYOCD_EXE		?= pyocd
# PyOCD device type, options: py32f003x4, py32f003x6, py32f003x8, py32f030x3, py32f030x4, py32f030x6, py32f030x7, py32f030x8
PYOCD_DEVICE	?= py32f003x8


##### Paths ############

# Link descript file: py32f003x6.ld, py32f003x8.ld, py32f030x6.ld, py32f030x8.ld
LDSCRIPT		= Libraries/LDScripts/py32f003x8.ld
# Library build flags: PY32F002x5, PY32F002Ax5, PY32F003x4, PY32F003x6, PY32F003x8, PY32F030x3, PY32F030x4, PY32F030x6, PY32F030x7, PY32F030x8, PY32F072xB
LIB_FLAGS       = PY32F003x8

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
INCLUDES	:= Libraries/CMSIS/Include \
			Libraries/CMSIS/Device/PY32F0xx/Include \
			User

ifeq ($(USE_LL_LIB),y)
CDIRS		+= Libraries/PY32F0xx_LL_Driver/Src \
		Libraries/BSP_LL/Src
INCLUDES	+= Libraries/PY32F0xx_LL_Driver/Inc \
		Libraries/BSP_LL/Inc
else
CDIRS		+= Libraries/PY32F0xx_HAL_Driver/Src \
		Libraries/BSP/Src
INCLUDES	+= Libraries/PY32F0xx_HAL_Driver/Inc \
		Libraries/BSP/Inc
endif

ifeq ($(USE_DSP),y)
LIB_FLAGS	+= ARM_MATH_CM0PLUS
CDIRS 		+= Libraries/CMSIS/DSP_Lib/Source/BasicMathFunctions \
		Libraries/CMSIS/DSP_Lib/Source/CommonTables \
		Libraries/CMSIS/DSP_Lib/Source/ComplexMathFunctions \
		Libraries/CMSIS/DSP_Lib/Source/ControllerFunctions \
		Libraries/CMSIS/DSP_Lib/Source/FastMathFunctions \
		Libraries/CMSIS/DSP_Lib/Source/FilteringFunctions \
		Libraries/CMSIS/DSP_Lib/Source/MatrixFunctions \
		Libraries/CMSIS/DSP_Lib/Source/StatisticsFunctions \
		Libraries/CMSIS/DSP_Lib/Source/SupportFunctions \
		Libraries/CMSIS/DSP_Lib/Source/TransformFunctions

endif

include ./rules.mk
