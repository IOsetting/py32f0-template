# py32f0-template

* Template project for Puya PY32F0 MCU
* Supports GNU Arm Embedded Toolchain
* Supports J-Link and DAPLink/PyOCD programmers
* Supports IDE: VSCode

# Puya PY32F0 Family

PY32F0 are cost-effective Arm Cortex-M0+ microcontrollers featured with wide range operating voltage from 1.7V to 5.5V. Datesheets and Reference Manuals can be found at [WIKI](https://github.com/IOsetting/py32f0-template/wiki).

## PY32F002A/003/030

Frequency up to 48 MHz, 16 to 64 Kbytes of Flash memory, 3 to 8 Kbytes of SRAM.

* PY32F002A
  * PY32F002Ax5(20KB Flash/3KB RAM)
* PY32F003
  * PY32F003x4(16KB Flash/2KB RAM), PY32F003x6(32KB Flash/4KB RAM), PY32F003x8(64KB Flash/8KB RAM)
* PY32F030
  * PY32F030x4(16KB Flash/2KB RAM), PY32F030x6(32KB Flash/4KB RAM), PY32F030x8(64KB Flash/8KB RAM)

## PY32F002B

Frequency up to 24 MHz, 24 Kbytes of Flash memory, 3 Kbytes of SRAM.

* PY32F001
  * ?
* PY32F002B
  * PY32F002Bx(24KB Flash/3KB RAM)

## PY32F040/071/072

Frequency up to 72 MHz, 128 Kbytes of Flash memory, 16 Kbytes of SRAM, with more peripherals(CAN, USB)

* PY32F040
  * PY32F040xB(128KB Flash/16KB RAM)
* PY32F071
  * PY32F071xB(128KB Flash/16KB RAM)
* PY32F072
  * PY32F072xB(128KB Flash/16KB RAM)

# File Structure

```
├── Build                       # Build results
├── Docs                        # Datesheets and User Manuals
├── Examples
│   ├── PY32F002B               # PY32F002B examples
│   │   ├── HAL                 # HAL library examples
│   │   └── LL                  # LL(Low Layer) library examples
│   ├── PY32F07x                # PY32F07x examples
│   │   └── HAL
│   └── PY32F0xx
│       ├── FreeRTOS            # FreeRTOS examples
│       ├── HAL
│       └── LL
├── Libraries
│   ├── CMSIS
│   ├── EPaper                  # Waveshare e-paper library
│   ├── FreeRTOS                # FreeRTOS library
│   ├── LDScripts               # LD files
│   ├── PY32F002B_HAL_BSP       # PY32F002B HAL BSP
│   ├── PY32F002B_HAL_Driver    # PY32F002B HAL library
│   ├── PY32F002B_LL_BSP        # PY32F002B LL(low layer) BSP
│   ├── PY32F002B_LL_Driver     # PY32F002B LL library
│   ├── PY32F07x_HAL_BSP        # PY32F040/071/072 HAL BSP
│   ├── PY32F07x_HAL_Driver     # PY32F040/071/072 HAL library
│   ├── PY32F0xx_HAL_BSP        # PY32F002A/003/030 HAL BSP
│   ├── PY32F0xx_HAL_Driver     # PY32F002A/003/030 HAL library
│   ├── PY32F0xx_LL_BSP         # PY32F002A/003/030 LL BSP
│   └── PY32F0xx_LL_Driver      # PY32F002A/003/030 LL library
|
├── Makefile                    # Make config
├── Misc
│   ├── Flash
│   │   ├── JLinkDevices        # JLink flash loaders
│   │   └── Sources             # Flash algorithm source code
│   ├── Puya.PY32F0xx_DFP.x.pack # DFP pack file for PyOCD
│   └── SVD                     # SVD files
├── README.md
├── rules.mk                    # Pre-defined rules include in Makefile 
└── User                        # User application code
```

# Requirements

* PY32F0 EVB or boards of PY32F002/003/030 series
* Programmer
  * J-Link: J-Link OB programmer
  * PyOCD: DAPLink or J-Link
* SEGGER J-Link Software and Documentation pack [https://www.segger.com/downloads/jlink/](https://www.segger.com/downloads/jlink/)
* PyOCD [https://pyocd.io/](https://pyocd.io/)
* GNU Arm Embedded Toolchain

# Building

## 1. Install GNU Arm Embedded Toolchain

Download the toolchain from [Arm GNU Toolchain Downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) according to your pc architecture, extract the files

```bash
sudo mkdir -p /opt/gcc-arm/
sudo tar xvf arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz -C /opt/gcc-arm/
cd /opt/gcc-arm/
sudo chown -R root:root arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/
```

## 2. Clone This Repository

Clone this repository to local workspace
```bash
git clone https://github.com/IOsetting/py32f0-template.git
```

## 3. Install SEGGER J-Link Or PyOCD

### Option 1: Install SEGGER J-Link

Download and install JLink from [J-Link / J-Trace Downloads](https://www.segger.com/downloads/jlink/).

```bash
# installation command for .deb
sudo dpkg -i JLink_Linux_V784f_x86_64.deb
# uncompression command for .tar.gz
sudo tar xvf JLink_Linux_V784f_x86_64.tgz -C [target folder]
```
The default installation directory is */opt/SEGGER*

Copy [Project directory]/Misc/Flash/JLinkDevices to [User home]/.config/SEGGER/
```bash
cd py32f0-template
cp -r Misc/Flash/JLinkDevices/ ~/.config/SEGGER/
```
Read more: [https://wiki.segger.com/J-Link_Device_Support_Kit](https://wiki.segger.com/J-Link_Device_Support_Kit)

### Option 2: Install PyOCD

Don't install from apt repository, because the version 0.13.1+dfsg-1 is too low for J-Link probe.

Install PyOCD from pip

```bash
pip uninstall pyocd
```
This will install PyOCD into:
```
/home/[user]/.local/bin/pyocd
/home/[user]/.local/bin/pyocd-gdbserver
/home/[user]/.local/lib/python3.10/site-packages/pyocd-0.34.2.dist-info/*
/home/[user]/.local/lib/python3.10/site-packages/pyocd/*
```
In Ubuntu, .profile will take care of the PATH, run `source ~/.profile` to make pyocd command available

## 4. Edit Makefile

Change the settings in Makefile

* **MCU_TYPE** The MCU type you are using
* **USE_LL_LIB** Puya provides two sets of library, HAL and LL, set `USE_LL_LIB ?= y` to use LL instead of HAL.
  * No LL Library for PY32F07x
* **ENABLE_PRINTF_FLOAT** set it to `y` to `-u _printf_float` to link options. This will increase the binary size.
* **USE_FREERTOS** Set `USE_FREERTOS ?= y` will include FreeRTOS in compilation
* **USE_DSP** Include CMSIS DSP or not
* **FLASH_PROGRM**
  * If you use J-Link, `FLASH_PROGRM` can be jlink or pyocd
  * If you use DAPLink, set `FLASH_PROGRM ?= pyocd`
  * ST-LINK is not supported yet.
* **ARM_TOOCHAIN** Make sure it points to the correct path of arm-none-eabi-gcc

```makefile
##### Project #####

PROJECT			?= app
# The path for generated files
BUILD_DIR		= Build

# MCU types: 
#   PY32F002Ax5
#   PY32F002Bx5
#   PY32F003x6, PY32F003x8, 
#   PY32F030x6, PY32F030x8, 
#   PY32F072xB
MCU_TYPE		= PY32F072xB

##### Options #####

# Use LL library instead of HAL, y:yes, n:no
USE_LL_LIB        ?= n
# Enable printf float %f support, y:yes, n:no
ENABLE_PRINTF_FLOAT ?= n
# Build with FreeRTOS, y:yes, n:no
USE_FREERTOS      ?= n
# Build with CMSIS DSP functions, y:yes, n:no
USE_DSP           ?= n
# Programmer, jlink or pyocd
FLASH_PROGRM      ?= pyocd

##### Toolchains #######
ARM_TOOCHAIN      ?= /opt/gcc-arm/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin

# path to JLinkExe
JLINKEXE		?= /opt/SEGGER/JLink/JLinkExe
# path to PyOCD
PYOCD_EXE		?= pyocd
```

## 5. Compiling And Flashing

```bash
# clean source code
make clean
# build
make
# or make with verbose output
V=1 make
# flash
make flash
```

# Debugging In VSCode

Install Cortex Debug extension, add a new configuration in launch.json, e.g.
```
{
    "armToolchainPath": "/opt/gcc-arm/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin/",
    "toolchainPrefix": "arm-none-eabi",
    "name": "Cortex Debug",
    "cwd": "${workspaceFolder}",
    "executable": "${workspaceFolder}/Build/app.elf",
    "request": "launch",        // can be launch or attach
    "type": "cortex-debug",
    "runToEntryPoint": "Reset_Handler", // "main" or other function name. runToMain is deprecated
    "servertype": "jlink",  // jlink, openocd, pyocd, pe and stutil
    "device": "PY32F030X8",
    "interface": "swd",
    "preLaunchTask": "build",  // Set this to run a task from tasks.json before starting a debug session
    // "preLaunchCommands": ["Build all"], // Uncomment this if not using preLaunchTask
    "svdFile": "${workspaceFolder}/Misc/SVD/py32f030xx.svd",  // svd for this part number
    "showDevDebugOutput": "vscode", // parsed, raw, vscode:vscode log and raw
    "swoConfig":
    {
        "enabled": true,
        "cpuFrequency": 8000000, // Target CPU frequency in Hz
        "swoFrequency":  4000000,
        "source": "probe", // either be “probe” to get directly from the debug probe, 
                           // or a serial port device to use a serial port external to the debug probe.
        "decoders":
        [
            {
                "label": "ITM port 0 output",
                "type": "console",
                "port": 0,
                "showOnStartup": true,
                "encoding": "ascii"
            }
        ]
    }
}
```
If Cortex Debug cannot find JLinkGDBServerCLExe, add the following line to settings.json
```
"cortex-debug.JLinkGDBServerPath": "/opt/SEGGER/JLink/JLinkGDBServerCLExe",
```

# Try Other Examples

More examples can be found in *Examples* folder, copy and replace the files under *User* folder to try different examples.

# Links

* Puya Product Page(Datasheet & SDK download): https://www.puyasemi.com/cpzx3/info_267_aid_242_kid_235.html
