// Copyright 2023 IOsetting <iosetting(at)outlook.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __L3G4200D_H
#define __L3G4200D_H

#include "main.h"

#define L3G4200D_ADDRESS           (0xD2) // SDO->High:11010010(0xD2), SDO->Low:11010000(0xD0)

#define L3G4200D_REG_WHO_AM_I      (0x0F) // Fixed value: 11010011(D3)

#define L3G4200D_REG_CTRL_REG1     (0x20) // [0,2]:X,Y,Z axis enable
                                          // [3]Power down mode
                                          // [4,5]:Bandwidth
                                          // [6,7]:Output Data Rate

#define L3G4200D_REG_CTRL_REG2     (0x21) // [0,3]:High Pass filter Cut Off frequency, 
                                          // [4,5]:High Pass filter Mode

#define L3G4200D_REG_CTRL_REG3     (0x22) // [0]:FIFO Empty interrupt on DRDY/INT2. Default value: 0. (0: Disable; 1: Enable)
                                          // [1]:FIFO Overrun interrupt on DRDY/INT2 Default value: 0. (0: Disable; 1: Enable)
                                          // [2]:FIFO Watermark interrupt on DRDY/INT2. Default value: 0. (0: Disable; 1: Enable)
                                          // [3]:Date Ready on DRDY/INT2. Default value 0. (0: Disable; 1: Enable)
                                          // [4]:Push- Pull / Open drain. Default value: 0. (0: Push- Pull; 1: Open drain)
                                          // [5]:Interrupt active configuration on INT1. Default value 0. (0: High; 1:Low)
                                          // [6]:Boot status available on INT1. Default value 0. (0: Disable; 1: Enable)
                                          // [7]:Interrupt enable on INT1 pin. Default value 0. (0: Disable; 1: Enable)

#define L3G4200D_REG_CTRL_REG4     (0x23) // [0]:SPI Serial Interface Mode, (0: 4-wire, 1: 3-wire)
                                          // [1,2]:Self Test Enable. (00: Self Test Disabled)
                                          // [4,5]:Full Scale selection. (00: 250 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps)
                                          // [6]:Big/Little Endian Data Selection. (0: Data LSB @ lower address; 1: Data MSB @ lower address)
                                          // [7]:Block Data Update. (0: continous update; 1: output registers not updated until MSB and LSB reading)

#define L3G4200D_REG_CTRL_REG5     (0x24) // [0,1]:Out selection configuration
                                          // [2,3]:INT1 selection configuration
                                          // [4]:High Pass filter Enable, (0: HPF disabled; 1: HPF enabled)
                                          // [6]:FIFO enable. (0: FIFO disable; 1: FIFO Enable)
                                          // [7]:Reboot memory content. (0: normal mode; 1: reboot memory content)

#define L3G4200D_REG_REFERENCE     (0x25) // Reference value for Interrupt generation. Default value: 0

#define L3G4200D_REG_OUT_TEMP      (0x26) // Temperature data

#define L3G4200D_REG_STATUS_REG    (0x27) // [0]:X axis new data available. 0:no, 1:yes
                                          // [1]:Y axis new data available. 0:no, 1:yes
                                          // [2]:Z axis new data available. 0:no, 1:yes
                                          // [3]:X, Y, Z -axis new data available. 0:no, 1:yes
                                          // [4]:X axis data overrun. 1: new data has overwritten the previous one
                                          // [5]:Y axis data overrun. 1: new data has overwritten the previous one
                                          // [6]:Z axis data overrun. 1: new data has overwritten the previous one
                                          // [7]:X, Y, Z -axis data overrun. 1: new data has overwritten the previous one

#define L3G4200D_REG_OUT_X_L       (0x28) // X-axis angular rate data
#define L3G4200D_REG_OUT_X_H       (0x29) // X-axis angular rate data
#define L3G4200D_REG_OUT_Y_L       (0x2A) // Y-axis angular rate data
#define L3G4200D_REG_OUT_Y_H       (0x2B) // Y-axis angular rate data
#define L3G4200D_REG_OUT_Z_L       (0x2C) // Z-axis angular rate data
#define L3G4200D_REG_OUT_Z_H       (0x2D) // Z-axis angular rate data

#define L3G4200D_REG_FIFO_CTRL_REG (0x2E) // [0,4]:FIFO threshold. Watermark level setting
                                          // [5,7]:FIFO mode selection, 000:Bypass mode, 001:FIFO mode, 010:Stream mode, 011:Stream-to-FIFO mode, 100:Bypass-to-Stream mode

#define L3G4200D_REG_FIFO_SRC_REG  (0x2F) // [0,4]:FIFO stored data level
                                          // [5]:FIFO empty bit. ( 0: FIFO not empty; 1: FIFO empty)
                                          // [6]:Overrun bit status. (0: FIFO is not completely filled; 1:FIFO is completely filled)
                                          // [7]:Watermark status. (0: FIFO filling is lower than WTM level; 1: FIFO filling is equalor higher than WTM level)

#define L3G4200D_REG_INT1_CFG      (0x30) // [0]:X interrupt on measured accel. lower than preset threshold (0: disable, 1: enable)
                                          // [1]:X interrupt on measured accel. higher than preset threshold (0: disable, 1: enable)
                                          // [2]:Y interrupt on measured accel. lower than preset threshold (0: disable, 1: enable)
                                          // [3]:Y interrupt on measured accel. higher than preset threshold (0: disable, 1: enable)
                                          // [4]:Z interrupt on measured accel. lower than preset threshold (0: disable, 1: enable)
                                          // [5]:Z interrupt on measured accel. higher than preset threshold (0: disable, 1: enable)
                                          // [6]:Latch Interrupt Request. (0:not latched; 1: latched) Cleared by reading INT1_SRC reg.
                                          // [7]:AND/OR combination of Interrupt events. Default value: 0, (0: OR, 1: AND)

#define L3G4200D_REG_INT1_SRC      (0x31) // [0]:Interrupt source X low
                                          // [1]:Interrupt source X high
                                          // [2]:Interrupt source Y low
                                          // [3]:Interrupt source Y high
                                          // [4]:Interrupt source Z low
                                          // [5]:Interrupt source Z high
                                          // [6]:Interrupt active. (0: no interrupt, 1: one or more interrupts have been generated)

#define L3G4200D_REG_INT1_THS_XH   (0x32) // [0,7]:Interrupt threshold X
#define L3G4200D_REG_INT1_THS_XL   (0x33) // [0,7]:Interrupt threshold X
#define L3G4200D_REG_INT1_THS_YH   (0x34) // [0,7]:Interrupt threshold Y
#define L3G4200D_REG_INT1_THS_YL   (0x35) // [0,7]:Interrupt threshold Y
#define L3G4200D_REG_INT1_THS_ZH   (0x36) // [0,7]:Interrupt threshold Z
#define L3G4200D_REG_INT1_THS_ZL   (0x37) // [0,7]:Interrupt threshold Z

#define L3G4200D_REG_INT1_DURATION (0x38) // [0,6]:The minimum duration of the Interrupt event to be recognized. Duration steps and maximum values depend on the ODR chosen.
                                          // [7]:WAIT enable. (0: disable; 1: enable)
                                          //     Wait =’1’: if signal crosses the selected threshold, the interrupt falls only after the duration
                                          //     has counted number of samples at the selected data rate, written into the duration counter register.

#define L3G4200D_ENABLE_X                   0b00000001
#define L3G4200D_ENABLE_Y                   0b00000010
#define L3G4200D_ENABLE_Z                   0b00000100
#define L3G4200D_ENABLE_ALL                 0b00000111
#define L3G4200D_ENABLE_NONE                0b00000000
#define L3G4200D_POWER_ON                   0b00001000

#define L3G4200D_DPS_250                    .00875f
#define L3G4200D_DPS_500                    .0175f
#define L3G4200D_DPS_2000                   .07f


typedef enum
{
    L3G4200D_SCALE_2000DPS = 0b10,
    L3G4200D_SCALE_500DPS  = 0b01,
    L3G4200D_SCALE_250DPS  = 0b00
} l3g4200d_dps_t;

typedef enum
{
    L3G4200D_DATARATE_800HZ_110  = 0b1111,
    L3G4200D_DATARATE_800HZ_50   = 0b1110,
    L3G4200D_DATARATE_800HZ_35   = 0b1101,
    L3G4200D_DATARATE_800HZ_30   = 0b1100,
    L3G4200D_DATARATE_400HZ_110  = 0b1011,
    L3G4200D_DATARATE_400HZ_50   = 0b1010,
    L3G4200D_DATARATE_400HZ_25   = 0b1001,
    L3G4200D_DATARATE_400HZ_20   = 0b1000,
    L3G4200D_DATARATE_200HZ_70   = 0b0111,
    L3G4200D_DATARATE_200HZ_50   = 0b0110,
    L3G4200D_DATARATE_200HZ_25   = 0b0101,
    L3G4200D_DATARATE_200HZ_12_5 = 0b0100,
    L3G4200D_DATARATE_100HZ_25   = 0b0001,
    L3G4200D_DATARATE_100HZ_12_5 = 0b0000
} l3g4200d_odrbw_t;

ErrorStatus L3G4200D_Begin(l3g4200d_dps_t scale, l3g4200d_odrbw_t odrbw);
l3g4200d_dps_t L3G4200D_GetScale(void);
l3g4200d_odrbw_t L3G4200D_GetOdrBw(void);

void L3G4200D_Calibrate(uint8_t samples);
void L3G4200D_SetThreshold(uint8_t multiple);
uint8_t L3G4200D_GetThreshold(void);

void L3G4200D_ReadRaw(int16_t *xbuf);
void L3G4200D_ReadNormalize(int16_t *xbuf);
uint8_t L3G4200D_ReadTemperature(void);


#endif // __L3G4200D_H
