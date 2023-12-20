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

#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"


#define MPU6050_ADDR                    0xD0 // MPU6050 address for write
/**
 * Register Address
*/
#define MPU6050_REG_SELF_TEST_X         0x0D // RW
#define MPU6050_REG_SELF_TEST_Y         0x0E // RW
#define MPU6050_REG_SELF_TEST_Z         0x0F // RW
#define MPU6050_REG_SELF_TEST_A         0x10 // RW
#define MPU6050_REG_SMPLRT_DIV          0x19 // RW, Sample Rate Divider
#define MPU6050_REG_CONFIG              0x1A // RW, Configuration
#define MPU6050_REG_GYRO_CONFIG         0x1B // RW, Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG        0x1C // RW, Accelerometer Configuration
#define MPU6050_REG_FIFO_EN             0x23 // RW
#define MPU6050_REG_I2C_MST_CTRL        0x24 // RW
#define MPU6050_REG_I2C_SLV0_ADDR       0x25 // RW
#define MPU6050_REG_I2C_SLV0_REG        0x26 // RW
#define MPU6050_REG_I2C_SLV0_CTRL       0x27 // RW
#define MPU6050_REG_I2C_SLV1_ADDR       0x28 // RW
#define MPU6050_REG_I2C_SLV1_REG        0x29 // RW
#define MPU6050_REG_I2C_SLV1_CTRL       0x2A // RW
#define MPU6050_REG_I2C_SLV2_ADDR       0x2B // RW
#define MPU6050_REG_I2C_SLV2_REG        0x2C // RW
#define MPU6050_REG_I2C_SLV2_CTRL       0x2D // RW
#define MPU6050_REG_I2C_SLV3_ADDR       0x2E // RW
#define MPU6050_REG_I2C_SLV3_REG        0x2F // RW
#define MPU6050_REG_I2C_SLV3_CTRL       0x30 // RW
#define MPU6050_REG_I2C_SLV4_ADDR       0x31 // RW
#define MPU6050_REG_I2C_SLV4_REG        0x32 // RW
#define MPU6050_REG_I2C_SLV4_DO         0x33 // RW
#define MPU6050_REG_I2C_SLV4_CTRL       0x34 // RW
#define MPU6050_REG_I2C_SLV4_DI         0x35 // R 
#define MPU6050_REG_I2C_MST_STATUS      0x36 // R 
#define MPU6050_REG_INT_PIN_CFG         0x37 // RW
#define MPU6050_REG_INT_ENABLE          0x38 // RW
#define MPU6050_REG_INT_STATUS          0x3A // R 
#define MPU6050_REG_ACCEL_XOUT_H        0x3B // R 
#define MPU6050_REG_ACCEL_XOUT_L        0x3C // R 
#define MPU6050_REG_ACCEL_YOUT_H        0x3D // R 
#define MPU6050_REG_ACCEL_YOUT_L        0x3E // R 
#define MPU6050_REG_ACCEL_ZOUT_H        0x3F // R 
#define MPU6050_REG_ACCEL_ZOUT_L        0x40 // R 
#define MPU6050_REG_TEMP_OUT_H          0x41 // R 
#define MPU6050_REG_TEMP_OUT_L          0x42 // R 
#define MPU6050_REG_GYRO_XOUT_H         0x43 // R 
#define MPU6050_REG_GYRO_XOUT_L         0x44 // R 
#define MPU6050_REG_GYRO_YOUT_H         0x45 // R 
#define MPU6050_REG_GYRO_YOUT_L         0x46 // R 
#define MPU6050_REG_GYRO_ZOUT_H         0x47 // R 
#define MPU6050_REG_GYRO_ZOUT_L         0x48 // R 
#define MPU6050_REG_EXT_SENS_DATA_00    0x49 // R 
#define MPU6050_REG_EXT_SENS_DATA_01    0x4A // R 
#define MPU6050_REG_EXT_SENS_DATA_02    0x4B // R 
#define MPU6050_REG_EXT_SENS_DATA_03    0x4C // R 
#define MPU6050_REG_EXT_SENS_DATA_04    0x4D // R 
#define MPU6050_REG_EXT_SENS_DATA_05    0x4E // R 
#define MPU6050_REG_EXT_SENS_DATA_06    0x4F // R 
#define MPU6050_REG_EXT_SENS_DATA_07    0x50 // R 
#define MPU6050_REG_EXT_SENS_DATA_08    0x51 // R 
#define MPU6050_REG_EXT_SENS_DATA_09    0x52 // R 
#define MPU6050_REG_EXT_SENS_DATA_10    0x53 // R 
#define MPU6050_REG_EXT_SENS_DATA_11    0x54 // R 
#define MPU6050_REG_EXT_SENS_DATA_12    0x55 // R 
#define MPU6050_REG_EXT_SENS_DATA_13    0x56 // R 
#define MPU6050_REG_EXT_SENS_DATA_14    0x57 // R 
#define MPU6050_REG_EXT_SENS_DATA_15    0x58 // R 
#define MPU6050_REG_EXT_SENS_DATA_16    0x59 // R 
#define MPU6050_REG_EXT_SENS_DATA_17    0x5A // R 
#define MPU6050_REG_EXT_SENS_DATA_18    0x5B // R 
#define MPU6050_REG_EXT_SENS_DATA_19    0x5C // R 
#define MPU6050_REG_EXT_SENS_DATA_20    0x5D // R 
#define MPU6050_REG_EXT_SENS_DATA_21    0x5E // R 
#define MPU6050_REG_EXT_SENS_DATA_22    0x5F // R 
#define MPU6050_REG_EXT_SENS_DATA_23    0x60 // R 
#define MPU6050_REG_I2C_SLV0_DO         0x63 // RW
#define MPU6050_REG_I2C_SLV1_DO         0x64 // RW
#define MPU6050_REG_I2C_SLV2_DO         0x65 // RW
#define MPU6050_REG_I2C_SLV3_DO         0x66 // RW
#define MPU6050_REG_I2C_MST_DELAY_CTRL  0x67 // RW
#define MPU6050_REG_SIGNAL_PATH_RESET   0x68 // RW
#define MPU6050_REG_USER_CTRL           0x6A // RW
#define MPU6050_REG_PWR_MGMT_1          0x6B // RW
#define MPU6050_REG_PWR_MGMT_2          0x6C // RW
#define MPU6050_REG_FIFO_COUNTH         0x72 // RW
#define MPU6050_REG_FIFO_COUNTL         0x73 // RW
#define MPU6050_REG_FIFO_R_W            0x74 // RW
#define MPU6050_REG_WHO_AM_I            0x75 // R #define

typedef enum
{
    MPU6050_Wakeup_Freq_1p25Hz          = 0x00,
    MPU6050_Wakeup_Freq_5Hz             = 0x01,
    MPU6050_Wakeup_Freq_20Hz            = 0x02,
    MPU6050_Wakeup_Freq_40Hz            = 0x03,
} MPU6050_Wakeup_Freq_t;

typedef enum
{
    MPU6050_DLPF_Delay0ms               = 0x00,
    MPU6050_DLPF_Delay2ms               = 0x01,
    MPU6050_DLPF_Delay3ms               = 0x02,
    MPU6050_DLPF_Delay5ms               = 0x03,
    MPU6050_DLPF_Delay8ms               = 0x04,
    MPU6050_DLPF_Delay13ms              = 0x05,
    MPU6050_DLPF_Delay19ms              = 0x06,
} MPU6050_DLPF_t;

typedef enum
{
    MPU6050_Gyro_FullScaleRange_250dps  = 0x00,
    MPU6050_Gyro_FullScaleRange_500dps  = 0x01,
    MPU6050_Gyro_FullScaleRange_1000dps = 0x02,
    MPU6050_Gyro_FullScaleRange_2000dps = 0x03,
} MPU6050_Gyro_FullScaleRange_t;

typedef enum
{
    MPU6050_Acc_FullScaleRange_2g       = 0x00,
    MPU6050_Acc_FullScaleRange_4g       = 0x01,
    MPU6050_Acc_FullScaleRange_8g       = 0x02,
    MPU6050_Acc_FullScaleRange_16g      = 0x03,
} MPU6050_Acc_FullScaleRange_t;

uint8_t MPU6050_Read(uint8_t addr);
uint16_t MPU6050_ReadInt(uint8_t addr);
void MPU6050_ReadAll(int16_t *buf);
void MPU6050_Init(void);
void MPU6050_Reset(void);
void MPU6050_EnterSleepMode(void);
void MPU6050_DisableTemperature(uint8_t state);
void MPU6050_EnableLowPowerMode(MPU6050_Wakeup_Freq_t freq);
void MPU6050_DisableLowPowerMode(void);
void MPU6050_SetSampleRateDiv(uint8_t div);
void MPU6050_SetDLPF(MPU6050_DLPF_t filter);
void MPU6050_SetGyroFullScaleRange(MPU6050_Gyro_FullScaleRange_t range);
void MPU6050_SetAccFullScaleRange(MPU6050_Acc_FullScaleRange_t range);


#endif // __MPU6050_H
