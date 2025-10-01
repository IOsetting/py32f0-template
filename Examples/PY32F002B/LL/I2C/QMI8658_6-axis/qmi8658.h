// Copyright 2021 IOsetting <iosetting(at)outlook.com>
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

#ifndef __QMI8658_H
#define __QMI8658_H

#include "main.h"

#define QMI8658_ADDR                    0X6B << 1
#define QMI8658_ADDR_ALTER              0X6A << 1

#define QMI8658_ID                      0X05

#define QMI8658_REG_WHOAMI              0X00 // Device identifier
#define QMI8658_REG_REVISIONID          0X01 // Device Revision ID

// Setup and Control Registers
#define QMI8658_REG_CTRL1               0x02 // Serial Interface and Sensor Enable
                                             // [7]SIM, 0:4wire spi, 1:3wire spi
                                             // [6]ADDR_AI, 0:Serial interface (SPI, I2C, I3C) address non-increment. 1:auto increment
                                             // [5]BE, 0:Serial interface (SPI, I2C, I3C) read data Little-Endian, 1:Big-Endian
                                             // [4]INT2_EN, 0:INT2 pin is high-Z mode, 1:INT2 pin output is enabled
                                             // [3]INT1_EN, 0:INT1 pin is high-Z mode, 1:INT1 pin output is enabled
                                             // [2]FIFO_INT_SEL, 0:FIFO interrupt is mapped to INT2 pin, 1:FIFO interrupt is mapped to INT1 pin
                                             // [0]SensorDisable, 0:Enable internal high-speed oscillator, 1:Disable internal high-speed oscillator
#define QMI8658_REG_CTRL2               0x03 // Accelerometer settings
                                             // [7]self test, 0:off, 1:on
                                             // [4:6]Full-scale, 0:±2g, 1:±4g, 2:±8g, 3:±16g
                                             // [0:3]ODR, Output Data Rate, 0000:fastest, 1111:slowest
#define QMI8658_REG_CTRL3               0x04 // Gyroscope: Output Data Rate, Full Scale, Self-Test
                                             // [7]self test, 0:off, 1:on
                                             // [4:6]Full-scale, 000 - ±16 dps, 001 - ±32 dps, 010 - ±64 dps, 011 - ±128 dps, 100 - ±256 dps, 101 - ±512 dps, 110 - ±1024dps, 111 - ±2048 dps
                                             // [0:3]ODR, Output Data Rate, 0000:fastest, 1000:slowest
#define QMI8658_REG_CTRL4               0X05 // Reserved
#define QMI8658_REG_CTRL5               0X06 // Low pass filter setting
                                             // [5:6]G_LPF_MODE, 00:2.66% of ODR, 01:3.63% of ODR, 10:5.39% of ODR, 11:13.37% of ODR
                                             // [4]G_LPF_EN, 0: Disable Gyroscope LPF, 1:Enable with the mode given by LPF_MODE
                                             // [1:2]A_LPF_MODE, 00:2.66% of ODR, 01:3.63% of ODR, 10:5.39% of ODR, 11:13.37% of ODR
                                             // [0]A_LPF_EN, 0: Disable Accelerometer LPF, 1:Enable with the mode given by LPF_MODE
#define QMI8658_REG_CTRL7               0x08 // Enable Sensors and Configure Data Reads
                                             // [7]SyncSample, 0: Disable SyncSample mode, 1: Enable SyncSample mode
                                             // [5]DRDY_DIS, 0: DRDY(Data Ready) is enabled, is driven to the INT2 pin, 1: DRDY(Data Ready) is disabled, is blocked from the INT2 pin
                                             // [4]G_SN, 0: Gyroscope in Full Mode (Drive and Sense are enabled). 1: Gyroscope in Snooze Mode (only Drive enabled).
                                             // [1]G_EN, 0: Disable Gyroscope. 1: Enable Gyroscope.
                                             // [0]A_EN, 0: Disable Accelerometer. 1: Enable Accelerometer.
#define QMI8658_REG_CTRL8               0X09 // Motion Detection Control
#define QMI8658_REG_CTRL9               0X0A // Host Commands

// Host Controlled Calibration Registers
#define QMI8658_CAL1_L                  0x0B // Calibration Register
#define QMI8658_CAL1_H                  0x0C
#define QMI8658_CAL2_L                  0x0D
#define QMI8658_CAL2_H                  0x0E
#define QMI8658_CAL3_L                  0x0F
#define QMI8658_CAL3_H                  0x10
#define QMI8658_CAL4_L                  0x11
#define QMI8658_CAL4_H                  0x12

// FIFO Registers
#define QMI8658_FIFO_WTM_TH             0x13 // FIFO watermark level, in ODRs
#define QMI8658_FIFO_CTRL               0x14 // FIFO Setup
#define QMI8658_FIFO_SMPL_CNT           0x15 // FIFO sample count LSBs
#define QMI8658_FIFO_STATUS             0x16 // FIFO Status
#define QMI8658_FIFO_DATA               0x17 // FIFO Data

// Status Registers
#define QMI8658_STATUSINT               0x2D // Sensor Data Availability with the Locking mechanism, CmdDone (CTRL9 protocol bit).
#define QMI8658_STATUS0                 0x2E // Output Data Over Run and Data Availability.
#define QMI8658_STATUS1                 0x2F // Miscellaneous Status: Any Motion, No Motion, Significant Motion, Pedometer, Tap.

// Timestamp Registers
#define QMI8658_TIMESTAMP_LOW           0x30 // Sample Time Stamp, lower 8 bits
#define QMI8658_TIMESTAMP_MID           0x31 // 
#define QMI8658_TIMESTAMP_HIGH          0x32 // upper 8 bits

// Data Output Registers (16 bits 2’s Complement Except COD Sensor Data)
#define QMI8658_TEMP_L                  0x33 // Temperature Output Data
#define QMI8658_TEMP_H                  0x34
#define QMI8658_AX_L                    0x35 // X-axis Acceleration
#define QMI8658_AX_H                    0x36
#define QMI8658_AY_L                    0x37
#define QMI8658_AY_H                    0x38
#define QMI8658_AZ_L                    0x39
#define QMI8658_AZ_H                    0x3A
#define QMI8658_GX_L                    0x3B // X-axis Gyro
#define QMI8658_GX_H                    0x3C
#define QMI8658_GY_L                    0x3D
#define QMI8658_GY_H                    0x3E
#define QMI8658_GZ_L                    0x3F
#define QMI8658_GZ_H                    0x40

// COD Indication and General Purpose Registers
#define QMI8658_COD_STATUS              0x46 // Calibration-On-Demand status register
#define QMI8658_DQW_L                   0x49 // General purpose register
#define QMI8658_DQW_H                   0x4A // General purpose register
#define QMI8658_DQX_L                   0x4B // General purpose register
#define QMI8658_DQX_H                   0x4C // Reserved
#define QMI8658_DQY_L                   0x4D // General purpose register
#define QMI8658_DQY_H                   0x4E // Reserved
#define QMI8658_DQZ_L                   0x4F // Reserved
#define QMI8658_DQZ_H                   0x50 // Reserved
#define QMI8658_DVX_L                   0x51 // General purpose register
#define QMI8658_DVX_H                   0x52 // General purpose register
#define QMI8658_DVY_L                   0x53 // General purpose register
#define QMI8658_DVY_H                   0x54 // General purpose register
#define QMI8658_DVZ_L                   0x55 // General purpose register
#define QMI8658_DVZ_H                   0x56 // General purpose register

// Activity Detection Output Registers
#define QMI8658_TAP_STATUS              0x59 // Axis, direction, number of detected Tap
#define QMI8658_STEP_CNT_LOW            0x5A // Low byte of step count of Pedometer
#define QMI8658_STEP_CNT_MIDL           0x5B // Middle byte of step count of Pedometer
#define QMI8658_STEP_CNT_HIGH           0x5C // High byte of step count of Pedometer

// Reset Register
#define QMI8658_RESET                   0x60 // Soft Reset Register


#define QMI8658_REG_INT_STATUS1   0x0A    /* 中断状态1 */
#define QMI8658_REG_INT_STATUS2   0x0B    /* 中断状态2 */
#define QMI8658_REG_INT_MASK      0x0D    /* 中断屏蔽 */
#define QMI8658_REG_INT_CONFIG    0x0E    /* 中断配置 */
#define QMI8658_REG_INT_MAP       0x19    /* 中断映射 */

#define QMI8658_ACC_RANGE_2G      0       /* ±2g */
#define QMI8658_ACC_RANGE_4G      1       /* ±4g */
#define QMI8658_ACC_RANGE_8G      2       /* ±8g */
#define QMI8658_ACC_RANGE_16G     3       /* ±16g */

#define QMI8658_GYR_RANGE_16DPS   0       /* ±16°/s */
#define QMI8658_GYR_RANGE_32DPS   1       /* ±32°/s */
#define QMI8658_GYR_RANGE_64DPS   2       /* ±64°/s */
#define QMI8658_GYR_RANGE_128DPS  3       /* ±128°/s */
#define QMI8658_GYR_RANGE_256DPS  4       /* ±256°/s */
#define QMI8658_GYR_RANGE_512DPS  5       /* ±512°/s */
#define QMI8658_GYR_RANGE_1024DPS 6       /* ±1024°/s */
#define QMI8658_GYR_RANGE_2048DPS 7       /* ±2048°/s */

#define QMI8658_ODR_1000HZ        0       /* 1000 Hz */
#define QMI8658_ODR_500HZ         1       /* 500 Hz */
#define QMI8658_ODR_250HZ         2       /* 250 Hz */
#define QMI8658_ODR_125HZ         3       /* 125 Hz */
#define QMI8658_ODR_62_5HZ        4       /* 62.5 Hz */
#define QMI8658_ODR_31_25HZ       5       /* 31.25 Hz */
#define QMI8658_ODR_15_625HZ      6       /* 15.625 Hz */
#define QMI8658_ODR_7_8125HZ      7       /* 7.8125 Hz */



typedef struct {
    uint8_t addr;
} QMI8658_t;


void QMI8658_Write(uint8_t reg, uint8_t dat);
uint8_t QMI8658_Read(uint8_t reg);
void QMI8658_BurstRead(uint8_t reg, uint8_t *buff, uint16_t size);
ErrorStatus QMI8658_Detect(uint8_t addr);
void QMI8658_Init(void);
uint8_t QMI8658_GetWhoami(void);
void QMI8658_SetEnSensors(uint8_t enaccel, uint8_t engyro);
void QMI8658_SetAccelConfig(uint8_t range, uint8_t odr);
void QMI8658_SetGyroConfig(uint8_t range, uint8_t odr);
void QMI8658_SetLpf(uint8_t genable, uint8_t glpf, uint8_t aenable, uint8_t alpf);
void QMI8658_ReadAll(int16_t *buff);

#endif // __QMI8658_H
