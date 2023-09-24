Following `Makefile` changes are needed to run this example:

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
