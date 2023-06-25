Following `Makefile` changes are needed to run this example on 'PY32F003W16S6TU SOP16' chip:

```diff
diff --git a/Makefile b/Makefile
index 5c24b12..cca49c1 100644
--- a/Makefile
+++ b/Makefile
@@ -38,13 +38,13 @@ PYOCD_EXE           ?= pyocd
 #   py32f003x4,  py32f003x6, py32f003x8,
 #   py32f030x3,  py32f030x4, py32f030x6, py32f030x7, py32f030x8
 #   py32f072xb
-PYOCD_DEVICE   ?= py32f030x8
+PYOCD_DEVICE   ?= py32f003x6


 ##### Paths ############

 # Link descript file: py32f002x5.ld, py32f003x6.ld, py32f003x8.ld, py32f030x6.ld, py32f030x8.ld
-LDSCRIPT               = Libraries/LDScripts/py32f030x8.ld
+LDSCRIPT               = Libraries/LDScripts/py32f003x6.ld
 # Library build flags:
 #   PY32F002x5, PY32F002Ax5,
 #   PY32F003x4, PY32F003x6, PY32F003x8,
@@ -61,7 +61,7 @@ CFILES :=
 # ASM source folders
 ADIRS  := User
 # ASM single files
-AFILES := Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f030.s
+AFILES := Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f003.s

 # Include paths
 INCLUDES       := Libraries/CMSIS/Core/Include \
```

To enable `printf` based debugging use the following patch:

```
diff --git a/Libraries/BSP/Inc/py32f0xx_bsp_printf.h b/Libraries/BSP/Inc/py32f0xx_bsp_printf.h
index 3637806..80a339c 100644
--- a/Libraries/BSP/Inc/py32f0xx_bsp_printf.h
+++ b/Libraries/BSP/Inc/py32f0xx_bsp_printf.h
@@ -48,8 +48,8 @@ extern "C" {

 #define DEBUG_USART_TX_GPIO_PORT                GPIOA
 #define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
-#define DEBUG_USART_TX_PIN                      GPIO_PIN_2
-#define DEBUG_USART_TX_AF                       GPIO_AF1_USART1
+#define DEBUG_USART_TX_PIN                      GPIO_PIN_7
+#define DEBUG_USART_TX_AF                       GPIO_AF8_USART1
```
