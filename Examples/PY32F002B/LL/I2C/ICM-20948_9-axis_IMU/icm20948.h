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

#ifndef __ICM20948_H
#define __ICM20948_H

#include "main.h"


#define ICM20948_SLAVE_ADDR                     0x68 << 1 // 0xD0

/* ICM-20948 Registers */
#define ICM20948_ID                             0xEA
#define ICM20948_REG_BANK_SEL                   0x7F

// USER BANK 0
#define ICM20948_B0_WHO_AM_I                    0x00        
#define ICM20948_B0_USER_CTRL                   0x03
#define ICM20948_B0_LP_CONFIG                   0x05
#define ICM20948_B0_PWR_MGMT_1                  0x06 // Reset Value: 0x41
                                                     // [7] DEVICE_RESET
                                                     // [6] SLEEP
                                                     // [5] LP_EN
                                                     // [3] TEMP_OFF
                                                     // [2:0] CLKSEL 0: Internal 20 MHz osc, 1-5: Auto selects the best, 6: Internal 20 MHz osc, 7: Stops the clock and keeps timing generator in reset
#define ICM20948_B0_PWR_MGMT_2                  0x07 // Reset Value: 0x00
                                                     // [5:3] DISABLE_ACCEL
                                                     // [2:0] DISABLE_GYRO
#define ICM20948_B0_INT_PIN_CFG                 0x0F // Reset Value: 0x00
                                                     // [7] INT1_ACTL
                                                     // [6] INT1_OPEN
                                                     // [5] INT1_LATCH__EN
                                                     // [4] INT_ANYRD_2CLEAR
                                                     // [3] ACTL_FSYNC
                                                     // [2] FSYNC_INT_MODE_EN
                                                     // [1] BYPASS_EN 1:I2C interface pins (ES_CL and ES_DA) will go into bypass mode
#define ICM20948_B0_INT_ENABLE                  0x10
#define ICM20948_B0_INT_ENABLE_1                0x11
#define ICM20948_B0_INT_ENABLE_2                0x12
#define ICM20948_B0_INT_ENABLE_3                0x13
#define ICM20948_B0_I2C_MST_STATUS              0x17        
#define ICM20948_B0_INT_STATUS                  0x19        
#define ICM20948_B0_INT_STATUS_1                0x1A
#define ICM20948_B0_INT_STATUS_2                0x1B
#define ICM20948_B0_INT_STATUS_3                0x1C
#define ICM20948_B0_DELAY_TIMEH                 0x28
#define ICM20948_B0_DELAY_TIMEL                 0x29
#define ICM20948_B0_ACCEL_XOUT_H                0x2D        
#define ICM20948_B0_ACCEL_XOUT_L                0x2E        
#define ICM20948_B0_ACCEL_YOUT_H                0x2F        
#define ICM20948_B0_ACCEL_YOUT_L                0x30        
#define ICM20948_B0_ACCEL_ZOUT_H                0x31        
#define ICM20948_B0_ACCEL_ZOUT_L                0x32    
#define ICM20948_B0_GYRO_XOUT_H                 0x33    
#define ICM20948_B0_GYRO_XOUT_L                 0x34
#define ICM20948_B0_GYRO_YOUT_H                 0x35
#define ICM20948_B0_GYRO_YOUT_L                 0x36
#define ICM20948_B0_GYRO_ZOUT_H                 0x37
#define ICM20948_B0_GYRO_ZOUT_L                 0x38
#define ICM20948_B0_TEMP_OUT_H                  0x39        
#define ICM20948_B0_TEMP_OUT_L                  0x3A
#define ICM20948_B0_EXT_SLV_SENS_DATA_00        0x3B
#define ICM20948_B0_EXT_SLV_SENS_DATA_01        0x3C
#define ICM20948_B0_EXT_SLV_SENS_DATA_02        0x3D
#define ICM20948_B0_EXT_SLV_SENS_DATA_03        0x3E
#define ICM20948_B0_EXT_SLV_SENS_DATA_04        0x3F
#define ICM20948_B0_EXT_SLV_SENS_DATA_05        0x40
#define ICM20948_B0_EXT_SLV_SENS_DATA_06        0x41
#define ICM20948_B0_EXT_SLV_SENS_DATA_07        0x42
#define ICM20948_B0_EXT_SLV_SENS_DATA_08        0x43
#define ICM20948_B0_EXT_SLV_SENS_DATA_09        0x44
#define ICM20948_B0_EXT_SLV_SENS_DATA_10        0x45
#define ICM20948_B0_EXT_SLV_SENS_DATA_11        0x46
#define ICM20948_B0_EXT_SLV_SENS_DATA_12        0x47
#define ICM20948_B0_EXT_SLV_SENS_DATA_13        0x48
#define ICM20948_B0_EXT_SLV_SENS_DATA_14        0x49
#define ICM20948_B0_EXT_SLV_SENS_DATA_15        0x4A
#define ICM20948_B0_EXT_SLV_SENS_DATA_16        0x4B
#define ICM20948_B0_EXT_SLV_SENS_DATA_17        0x4C
#define ICM20948_B0_EXT_SLV_SENS_DATA_18        0x4D
#define ICM20948_B0_EXT_SLV_SENS_DATA_19        0x4E
#define ICM20948_B0_EXT_SLV_SENS_DATA_20        0x4F
#define ICM20948_B0_EXT_SLV_SENS_DATA_21        0x50
#define ICM20948_B0_EXT_SLV_SENS_DATA_22        0x51
#define ICM20948_B0_EXT_SLV_SENS_DATA_23        0x52
#define ICM20948_B0_FIFO_EN_1                   0x66    
#define ICM20948_B0_FIFO_EN_2                   0x67
#define ICM20948_B0_FIFO_RST                    0x68
#define ICM20948_B0_FIFO_MODE                   0x69
#define ICM20948_B0_FIFO_COUNTH                 0X70
#define ICM20948_B0_FIFO_COUNTL                 0X71
#define ICM20948_B0_FIFO_R_W                    0x72
#define ICM20948_B0_DATA_RDY_STATUS             0x74
#define ICM20948_B0_FIFO_CFG                    0x76    

// USER BANK 1
#define ICM20948_B1_SELF_TEST_X_GYRO            0x02    
#define ICM20948_B1_SELF_TEST_Y_GYRO            0x03
#define ICM20948_B1_SELF_TEST_Z_GYRO            0x04
#define ICM20948_B1_SELF_TEST_X_ACCEL           0x0E    
#define ICM20948_B1_SELF_TEST_Y_ACCEL           0x0F
#define ICM20948_B1_SELF_TEST_Z_ACCEL           0x10
#define ICM20948_B1_XA_OFFS_H                   0x14    
#define ICM20948_B1_XA_OFFS_L                   0x15
#define ICM20948_B1_YA_OFFS_H                   0x17
#define ICM20948_B1_YA_OFFS_L                   0x18
#define ICM20948_B1_ZA_OFFS_H                   0x1A
#define ICM20948_B1_ZA_OFFS_L                   0x1B
#define ICM20948_B1_TIMEBASE_CORRECTION_PLL     0x28    

// USER BANK 2
#define ICM20948_B2_GYRO_SMPLRT_DIV             0x00    
#define ICM20948_B2_GYRO_CONFIG_1               0x01 // Default: 0x01
                                                     // [5:3] GYRO_DLPFCFG, NBW = 0:229.8Hz, 1:187.6Hz, 2:154.3Hz, 3:73.3Hz, 4:35.9Hz, 5:17.8Hz, 6:8.9Hz, 7:376.5Hz
                                                     // [2:0] GYRO_FS_SEL, 00 = ±250 dps, 01= ±500 dps, 10 = ±1000 dps, 11 = ±2000 dps
                                                     // [1:0] GYRO_FCHOICE, 0=Bypass gyro DLPF, 1=Enable gyro DLPF
#define ICM20948_B2_GYRO_CONFIG_2               0x02
#define ICM20948_B2_XG_OFFS_USRH                0x03    
#define ICM20948_B2_XG_OFFS_USRL                0x04
#define ICM20948_B2_YG_OFFS_USRH                0x05
#define ICM20948_B2_YG_OFFS_USRL                0x06
#define ICM20948_B2_ZG_OFFS_USRH                0x07
#define ICM20948_B2_ZG_OFFS_USRL                0x08
#define ICM20948_B2_ODR_ALIGN_EN                0x09    
#define ICM20948_B2_ACCEL_SMPLRT_DIV_1          0x10    
#define ICM20948_B2_ACCEL_SMPLRT_DIV_2          0x11        
#define ICM20948_B2_ACCEL_INTEL_CTRL            0x12        
#define ICM20948_B2_ACCEL_WOM_THR               0x13
#define ICM20948_B2_ACCEL_CONFIG                0x14 // Default: 0x01
                                                     // [5:3] ACCEL_DLPFCFG, NBW = 0:265Hz, 1:265Hz, 2:136Hz, 3:68.8Hz, 4:34.4Hz, 5:17.0Hz, 6:8.3Hz, 7:499Hz
                                                     // [2:0] ACCEL_FS_SEL, 00 = ±2g, 01= ±4g, 10 = ±8g, 11 = ±16g
                                                     // [1:0] ACCEL_FCHOICE, 0=Bypass accel DLPF, 1=Enable accel DLPF
#define ICM20948_B2_ACCEL_CONFIG_2              0x15
#define ICM20948_B2_FSYNC_CONFIG                0x52
#define ICM20948_B2_TEMP_CONFIG                 0x53
#define ICM20948_B2_MOD_CTRL_USR                0X54

// USER BANK 3
#define ICM20948_B3_I2C_MST_ODR_CONFIG          0x00
#define ICM20948_B3_I2C_MST_CTRL                0x01
#define ICM20948_B3_I2C_MST_DELAY_CTRL          0x02    
#define ICM20948_B3_I2C_SLV0_ADDR               0x03
#define ICM20948_B3_I2C_SLV0_REG                0x04        
#define ICM20948_B3_I2C_SLV0_CTRL               0x05
#define ICM20948_B3_I2C_SLV0_DO                 0x06
#define ICM20948_B3_I2C_SLV1_ADDR               0x07        
#define ICM20948_B3_I2C_SLV1_REG                0x08        
#define ICM20948_B3_I2C_SLV1_CTRL               0x09
#define ICM20948_B3_I2C_SLV1_DO                 0x0A
#define ICM20948_B3_I2C_SLV2_ADDR               0x0B        
#define ICM20948_B3_I2C_SLV2_REG                0x0C        
#define ICM20948_B3_I2C_SLV2_CTRL               0x0D
#define ICM20948_B3_I2C_SLV2_DO                 0x0E
#define ICM20948_B3_I2C_SLV3_ADDR               0x0F        
#define ICM20948_B3_I2C_SLV3_REG                0x10        
#define ICM20948_B3_I2C_SLV3_CTRL               0x11
#define ICM20948_B3_I2C_SLV3_DO                 0x12
#define ICM20948_B3_I2C_SLV4_ADDR               0x13    
#define ICM20948_B3_I2C_SLV4_REG                0x14        
#define ICM20948_B3_I2C_SLV4_CTRL               0x15
#define ICM20948_B3_I2C_SLV4_DO                 0x16
#define ICM20948_B3_I2C_SLV4_DI                 0x17
    

/* AK09916 Registers */
#define AK09916_ID                              0x09
#define AK09916_SLAVE_ADDR                      0x0C << 1 // 0x18

#define AK09916_MAG_WIA2                        0x01
#define AK09916_MAG_ST1                         0x10
#define AK09916_MAG_HXL                         0x11
#define AK09916_MAG_HXH                         0x12
#define AK09916_MAG_HYL                         0x13
#define AK09916_MAG_HYH                         0x14
#define AK09916_MAG_HZL                         0x15
#define AK09916_MAG_HZH                         0x16
#define AK09916_MAG_ST2                         0x18
#define AK09916_MAG_CNTL2                       0x31 // [4:0] MODE: 
                                                     //       00000: Power-down mode
                                                     //       00001: Single measurement mode
                                                     //       00010: Continuous measurement mode 10Hz
                                                     //       00100: Continuous measurement mode 20Hz
                                                     //       00110: Continuous measurement mode 50Hz
                                                     //       01000: Continuous measurement mode 100Hz
                                                     //       10000: Self-test mode
#define AK09916_MAG_CNTL3                       0x32 // [0] RST: Soft reset, 0:normal, 1:reset
#define AK09916_MAG_TS1                         0x33
#define AK09916_MAG_TS2                         0x34 // when any of measurement data is read, be sure to read ST2 register at the end


#define ICM20948_B0                             0x00
#define ICM20948_B1                             0x01
#define ICM20948_B2                             0x02
#define ICM20948_B3                             0x03

#define AK09916_MAG_MODE_POWER_DOWN             0x0
#define AK09916_MAG_MODE_TRIGGER                0x01
#define AK09916_MAG_MODE_CONT_10HZ              0x02
#define AK09916_MAG_MODE_CONT_20HZ              0x04
#define AK09916_MAG_MODE_CONT_50HZ              0x06
#define AK09916_MAG_MODE_CONT_100HZ             0x08
#define AK09916_MAG_MODE_SELF_TEST              0x10


typedef struct {
    uint8_t addr, mag_addr, bank;
} ICM20948_t;


void ICM20948_Write(uint8_t reg, uint8_t dat);
uint8_t ICM20948_Read(uint8_t reg);
void ICM20948_ReadInBatch(uint8_t reg, uint8_t *buff, uint16_t size);

ErrorStatus ICM20948_Detect(uint8_t addr);
void ICM20948_Init(void);
void ICM20948_SetBank(uint8_t bank);
uint8_t ICM20948_GetWhoami(void);
void ICM20948_SetSleep(uint8_t status);
void ICM20948_SetLowPower(uint8_t status);
void ICM20948_SetClockSource(uint8_t sel);
void ICM20948_SetAccelConfig(uint8_t fchoice, uint8_t fssel, uint8_t dlpfconf);
void ICM20948_SetGyroConfig(uint8_t fchoice, uint8_t fssel, uint8_t dlpfconf);
void ICM20948_SetEnBypass(uint8_t status);
void ICM20948_ReadAll(int16_t *buff);

/** AK09916 methods, call ICM20948_SetEnBypass(1) before calling below methods */

void AK09916_Write(uint8_t reg, uint8_t dat);
uint8_t AK09916_Read(uint8_t reg);
void AK09916_ReadInBatch(uint8_t reg, uint8_t *buff, uint16_t size);
ErrorStatus AK09916_Detect(uint8_t addr);
uint8_t AK09916_GetWhoami(void);
void AK09916_Reset(void);
void AK09916_SetMode(uint8_t mode);
void AK09916_ReadAll(int16_t *buff);

#endif // __ICM20948_H
