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


#include "spl06.h"
#include <stdio.h>

/** Compensation Scale Factors */
const static uint32_t scale_factor[8] = {
    524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960
};

static SPL06_t spl06;

void SPL06_Write(uint8_t addr, uint8_t dat)
{
    BSP_I2cTransmit(spl06.addr, addr, &dat, 1);
}

uint8_t SPL06_Read(uint8_t addr)
{
    uint8_t ret;
    BSP_I2cReceive(spl06.addr, addr, &ret, 1);
    return ret;
}

void SPL06_ReadInBatch(uint8_t *buf, uint8_t size)
{
    BSP_I2cReceive(spl06.addr, 0x00, buf, 14);
}

void SPL06_GetBoth(int32_t *data)
{
    uint8_t *u8d = (uint8_t *)data;
    BSP_I2cReceive(spl06.addr, SPL06_PSR_B2, u8d, 6);
    /* 
     * convert big-endian to little-endian
     *      7  6  5  4  3  2  1  0
     *     __ __ aa bb cc dd ee ff
     *  -> __ cc bb aa __ ff ee dd
     */
    // 3 bytes for temperature
    *(u8d + 7) = *(u8d + 5); // for swap
    *(u8d + 6) = *(u8d + 3);
    *(u8d + 5) = *(u8d + 4);
    *(u8d + 4) = *(u8d + 7);
    // 3 bytes for pressure
    *(u8d + 7) = *(u8d + 2); // for swap
    *(u8d + 2) = *(u8d + 0);
    *(u8d + 0) = *(u8d + 7);
    *(u8d + 3) = *(u8d + 2) & 0x80 ? 0xFF : 0x00;
    *(u8d + 7) = *(u8d + 6) & 0x80 ? 0xFF : 0x00;
}

/**
 * Traw_sc = Traw / Tscale_factor
 * Tcomp(°C) = C0 * 0.5 + C1 * Traw_sc
 */
float SPL06_GetTemperature(int32_t traw)
{
    float t, c;
    t = traw / (float) scale_factor[spl06.psrate];
    c = (spl06.c0 * 0.5) + (spl06.c1 * t);
    return c;
}

/**
 * Traw_sc = Traw / Tscale_factor
 * Praw_sc = Praw / Pscale_factor
 * Pcomp(Pa) = C00 
 *           + Traw_sc * C01
 *           + Praw_sc * (C10 + Praw_sc * (C20 + Praw_sc * C30)) 
 *           + Traw_sc * Praw_sc * (C11 + Praw_sc * C21)
  */
float SPL06_GetPressure(int32_t traw, int32_t praw)
{
    float t, p, c;
    t = traw / (float) scale_factor[spl06.tsrate];
    p = praw / (float) scale_factor[spl06.psrate];
    c = spl06.c00 
        + (spl06.c01 * t) 
        + (p * (spl06.c10 + p *(spl06.c20 + (spl06.c30 * p))))
        + (t * p * (spl06.c11 + (spl06.c21 * p)));
    return c;
}

uint8_t SPL06_Detect(uint8_t address)
{
    spl06.addr = address;
    if (SPL06_Read(SPL06_DEVICE_ID) == 0x10)
    {
        return 1;
    }
    return 0;
}

void SPL06_Init(uint8_t address, SPL06_Mode_t mode)
{
    uint8_t status;
    spl06.addr = address;
    spl06.pmrate = SPL06_Rate_X8;
    spl06.psrate = SPL06_Rate_X8;
    spl06.tmrate = SPL06_Rate_X8;
    spl06.tsrate = SPL06_Rate_X8;
    SPL06_Write(SPL06_SOFT_RESET, 0x09);
    LL_mDelay(10);
    SPL06_Write(SPL06_MEAS_CFG, mode);
    do
    {
        status = SPL06_Read(SPL06_MEAS_CFG);
        BSP_UartTxHex(&status, 1);
        BSP_UartTxChar(' ');
    } while (!(status & 0x80) || !(status & 0x40));
    SPL06_ReadCalibParam();
}

void SPL06_SetPressureRate(SPL06_Rate_t measurementRate, SPL06_Rate_t overSamplingRate)
{
    spl06.pmrate = measurementRate;
    spl06.psrate = overSamplingRate;
    SPL06_Write(SPL06_PSR_CFG, (measurementRate << 4) | overSamplingRate);
}

void SPL06_SetTemperatureRate(SPL06_Rate_t measurementRate, SPL06_Rate_t overSamplingRate)
{
    spl06.tmrate = measurementRate;
    spl06.tsrate = overSamplingRate;
    SPL06_Write(SPL06_TMP_CFG, (measurementRate << 4) | overSamplingRate);
}

void SPL06_ChangeMode(SPL06_Mode_t mode)
{
    SPL06_Write(SPL06_MEAS_CFG, mode);
}

void SPL06_Reset(void)
{
    SPL06_Write(SPL06_SOFT_RESET, 0x09);
}

void SPL06_ReadCalibParam(void)
{
    uint8_t buf[18];
    BSP_I2cReceive(spl06.addr, SPL06_COEF_C0, buf, 18);
    // C0
    spl06.c0 = (int16_t)(buf[0] << 4) | buf[1] >> 4;
    if (spl06.c0 & 0x0800) spl06.c0 |= 0xF000;
    BSP_UartTxHex((uint8_t *)&spl06.c0, 2);
    BSP_UartTxChar(' ');
    // C1
    spl06.c1 = (int16_t)((buf[1] & 0x0F) << 8) | buf[2];
    if (spl06.c1 & 0x0800) spl06.c1 |= 0xF000;
    BSP_UartTxHex((uint8_t *)&spl06.c1, 2);
    BSP_UartTxChar(' ');
    // C00
    spl06.c00 = (int32_t)(buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    if (spl06.c00 & 0x080000) spl06.c00 |= 0xFFF00000;
    BSP_UartTxHex((uint8_t *)&spl06.c00, 4);
    BSP_UartTxChar(' ');
    // C10
    spl06.c10 = (int32_t)((buf[5] & 0x0F) << 16) | (buf[6] << 8) | buf[7];
    if (spl06.c10 & 0x080000) spl06.c10 |= 0xFFF00000;
    BSP_UartTxHex((uint8_t *)&spl06.c10, 4);
    BSP_UartTxChar(' ');
    // C01, C11, C20, C21, C30
    spl06.c01 = (int16_t)(buf[8] << 8) | buf[9];
    BSP_UartTxHex((uint8_t *)&spl06.c01, 2);
    BSP_UartTxChar(' ');
    spl06.c11 = (int16_t)(buf[10] << 8) | buf[11];
    BSP_UartTxHex((uint8_t *)&spl06.c11, 2);
    BSP_UartTxChar(' ');
    spl06.c20 = (int16_t)(buf[12] << 8) | buf[13];
    BSP_UartTxHex((uint8_t *)&spl06.c20, 2);
    BSP_UartTxChar(' ');
    spl06.c21 = (int16_t)(buf[14] << 8) | buf[15];
    BSP_UartTxHex((uint8_t *)&spl06.c21, 2);
    BSP_UartTxChar(' ');
    spl06.c30 = (int16_t)(buf[16] << 8) | buf[17];
    BSP_UartTxHex((uint8_t *)&spl06.c30, 2);
    BSP_UartTxString("\r\n");
}