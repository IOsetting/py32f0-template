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

#include "l3g4200d.h"
#include "i2c.h"
#include <math.h>

int16_t delta[3], threshold[3];

static void L3G4200D_WriteByte(uint8_t reg, uint8_t value)
{
    APP_I2C_Transmit(L3G4200D_ADDRESS, reg, &value, 1);
    LL_mDelay(1);
}

static uint8_t L3G4200D_ReadByte(uint8_t reg)
{
    uint8_t ret;
    APP_I2C_Receive(L3G4200D_ADDRESS, reg, &ret, 1);
    // Add delay when i2c speed is 100khz
    //LL_mDelay(1);
    return ret;
}

static void L3G4200D_BurstRead(uint8_t reg, uint8_t *buf, uint8_t size)
{
    // bit[7] must be equal to 1 in order to read multiple bytes
    APP_I2C_Receive(L3G4200D_ADDRESS, reg | 0x80, buf, size);
}

ErrorStatus L3G4200D_Begin(l3g4200d_dps_t scale, l3g4200d_odrbw_t odrbw)
{
    // Check L3G4200D Who Am I Register
    if (L3G4200D_ReadByte(L3G4200D_REG_WHO_AM_I) != 0xD3)
    {
	    return ERROR;
    }
    // Enable all axis and setup normal mode
    L3G4200D_WriteByte(L3G4200D_REG_CTRL_REG1, (odrbw << 4) | L3G4200D_ENABLE_ALL | L3G4200D_POWER_ON);
    L3G4200D_WriteByte(L3G4200D_REG_CTRL_REG4, scale << 4);
    return SUCCESS;
}

l3g4200d_dps_t L3G4200D_GetScale(void)
{
    return (l3g4200d_dps_t)((L3G4200D_ReadByte(L3G4200D_REG_CTRL_REG4) >> 4) & 0x03);
}

l3g4200d_odrbw_t L3G4200D_GetOdrBw(void)
{
    return (l3g4200d_odrbw_t)((L3G4200D_ReadByte(L3G4200D_REG_CTRL_REG1) >> 4) & 0x0F);
}

void L3G4200D_Calibrate(uint8_t samples)
{
    int32_t sum[3] = {0,0,0}, sigma[3] = {0,0,0};

    // Read samples
    for (uint8_t i = 0; i < samples; ++i)
    {
        L3G4200D_ReadRaw(delta);
        sum[0] += delta[0];
        sum[1] += delta[1];
        sum[2] += delta[2];

        sigma[0] += delta[0] * delta[0];
        sigma[1] += delta[1] * delta[1];
        sigma[2] += delta[2] * delta[2];

        LL_mDelay(5);
    }
    // Calculate delta
    delta[0] = sum[0] / samples;
    delta[1] = sum[1] / samples;
    delta[2] = sum[2] / samples;
    // Calculate threshold
    threshold[0] = sqrt((sigma[0] / samples) - (delta[0] * delta[0]));
    threshold[1] = sqrt((sigma[1] / samples) - (delta[1] * delta[1]));
    threshold[2] = sqrt((sigma[2] / samples) - (delta[2] * delta[2]));
}

void L3G4200D_SetThreshold(uint8_t multiple)
{
}

uint8_t L3G4200D_GetThreshold(void)
{
    return 0;
}

void L3G4200D_ReadRaw(int16_t *xbuf)
{
    L3G4200D_BurstRead(L3G4200D_REG_OUT_X_L, (uint8_t *)xbuf, 6);
}

void L3G4200D_ReadNormalize(int16_t *xbuf)
{
    uint8_t i;
    L3G4200D_ReadRaw(xbuf);
    for (i = 0; i < 3; i++)
    {
        *(xbuf + i) -= *(delta + i);
    }
}

/**
 * Temperature value is negative coefficient with °C
*/
uint8_t L3G4200D_ReadTemperature(void)
{
    return L3G4200D_ReadByte(L3G4200D_REG_OUT_TEMP);
}
