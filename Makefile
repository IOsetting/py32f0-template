##### Project #####

PROJECT			?= app
# The path for generated files
BUILD_DIR		= Build


##### Options #####

# Enable printf float %f support, y:yes, n:no
ENABLE_PRINTF_FLOAT	?= n
# Programmer, jlink or pyocd
FLASH_PROGRM	?= pyocd

##### Toolchains #######

#ARM_TOOCHAIN	?= /opt/gcc-arm/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi/bin
#ARM_TOOCHAIN	?= /opt/gcc-arm/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi/bin
ARM_TOOCHAIN	?= /opt/gcc-arm/arm-gnu-toolchain-12.2.mpacbti-bet1-x86_64-arm-none-eabi/bin

# path to JLinkExe
JLINKEXE		?= /opt/SEGGER/JLink/JLinkExe
# JLink device type, options: PY32F003X4, PY32F003X6, PY32F003X8, PY32F030X6, PY32F030X7, PY32F030X8
JLINK_DEVICE	?= PY32F003X8
# path to PyOCD, 
PYOCD_EXE		?= pyocd
# PyOCD device type, options: py32f003x4, py32f003x6, py32f003x8, py32f030x3, py32f030x4, py32f030x6, py32f030x7, py32f030x8
PYOCD_DEVICE	?= py32f003x8


##### Paths ############

# Link descript file
LDSCRIPT		= Libraries/LDScripts/py32f003x8.ld
# Library build flags: PY32F030x3, PY32F030x4, PY32F030x6, PY32F030x7, PY32F030x8, PY32F003x4, PY32F003x6, PY32F003x8
LIB_FLAGS       = PY32F003x8

# C source folders
CDIRS	:= User \
		Libraries/CMSIS/Device/PY32F0xx/Source \
		Libraries/PY32F0xx_HAL_Driver/Src \
		Libraries/BSP/Src
# C source files (if there are any single ones)
CFILES := 

# ASM source folders
ADIRS	:= User
# ASM single files
AFILES	:= Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f003.s

# Include paths
INCLUDES	:= Libraries/CMSIS/Include \
			Libraries/CMSIS/Device/PY32F0xx/Include \
			Libraries/PY32F0xx_HAL_Driver/Inc \
			Libraries/BSP/Inc \
			User

include ./rules.mk
