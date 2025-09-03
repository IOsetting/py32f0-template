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

#include "mpu6050.h"
#include "i2c.h"

uint16_t swap(uint16_t num)
{
    return (num >> 8) | (num << 8);
}

void MPU6050_Write(uint8_t addr, uint8_t dat)
{
    APP_I2C_Transmit(MPU6050_ADDR, addr, &dat, 1);
}

uint8_t MPU6050_Read(uint8_t addr)
{
    uint8_t ret;
    APP_I2C_Receive(MPU6050_ADDR, addr, &ret, 1);
    return ret;
}

uint16_t MPU6050_ReadInt(uint8_t addr)
{
    uint16_t ret;
    APP_I2C_Receive(MPU6050_ADDR, addr, (uint8_t *)&ret, 2);
    return swap(ret); // swap high/low bits for correct order
}

void MPU6050_ReadAll(int16_t *buf)
{
    uint8_t i;
    APP_I2C_Receive(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, (uint8_t *)buf, 14);
    for (i = 0; i < 7; i++)
    {
        *(buf + i) = swap(*(buf + i));
    }
}

void MPU6050_Init(void) 
{
    MPU6050_DisableLowPowerMode();
    MPU6050_SetSampleRateDiv(0x07);
    MPU6050_SetDLPF(MPU6050_DLPF_Delay2ms);
    MPU6050_SetGyroFullScaleRange(MPU6050_Gyro_FullScaleRange_500dps);
    MPU6050_SetAccFullScaleRange(MPU6050_Acc_FullScaleRange_4g);
}

void MPU6050_Reset(void)
{
    MPU6050_Write(MPU6050_REG_PWR_MGMT_1, 0x80);
}

void MPU6050_EnterSleepMode(void)
{
    MPU6050_Write(MPU6050_REG_PWR_MGMT_1, 0x40);
}

void MPU6050_DisableTemperature(uint8_t state)
{
    uint8_t reg = MPU6050_Read(MPU6050_REG_PWR_MGMT_1);
    MPU6050_Write(MPU6050_REG_PWR_MGMT_1, (reg & ~0x08) | (state << 3));
}

void MPU6050_EnableLowPowerMode(MPU6050_Wakeup_Freq_t freq)
{
    MPU6050_Write(MPU6050_REG_PWR_MGMT_1, 0x28); // 0010,1000 sleep:0, cycle:1, dis_temp:1
    MPU6050_Write(MPU6050_REG_PWR_MGMT_2, freq << 6 | 0x03); // STBY_XG, STBY_YG, STBY_ZG -> 1
}

void MPU6050_DisableLowPowerMode(void)
{
    MPU6050_Write(MPU6050_REG_PWR_MGMT_1, 0x00);
    MPU6050_Write(MPU6050_REG_PWR_MGMT_2, 0x00);
}

/**
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz
 * when the DLPF is enabled
*/
void MPU6050_SetSampleRateDiv(uint8_t div)
{
    MPU6050_Write(MPU6050_REG_SMPLRT_DIV, div);
}

void MPU6050_SetDLPF(MPU6050_DLPF_t filter)
{
    MPU6050_Write(MPU6050_REG_CONFIG, filter);
}

void MPU6050_SetGyroFullScaleRange(MPU6050_Gyro_FullScaleRange_t range)
{
    MPU6050_Write(MPU6050_REG_GYRO_CONFIG, range << 3);
}

void MPU6050_SetAccFullScaleRange(MPU6050_Acc_FullScaleRange_t range)
{
    MPU6050_Write(MPU6050_REG_ACCEL_CONFIG, range << 3);
}
