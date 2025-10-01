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

#include "icm20948.h"
#include "bsp_i2c.h"
#include "bsp_uart.h"

ICM20948_t icm20948;

void ICM20948_Write(uint8_t reg, uint8_t dat)
{
    BSP_I2cTransmit(icm20948.addr, reg, &dat, 1);
    BSP_RccNop(1);
}

uint8_t ICM20948_Read(uint8_t reg)
{
    uint8_t ret;
    BSP_I2cReceive(icm20948.addr, reg, &ret, 1);
    BSP_RccNop(1);
    return ret;
}

void ICM20948_ReadInBatch(uint8_t reg, uint8_t *buff, uint16_t size)
{
    BSP_I2cReceive(icm20948.addr, reg, buff, size);
    BSP_RccNop(1);
}

ErrorStatus ICM20948_Detect(uint8_t addr)
{
    icm20948.addr = addr;
    if (ICM20948_Read(ICM20948_B0_WHO_AM_I) == ICM20948_ID)
    {
        
        return SUCCESS;
    }
    return ERROR;
}

void ICM20948_Init(void)
{
    icm20948.bank = 0x00;
    ICM20948_Write(ICM20948_REG_BANK_SEL, icm20948.bank);
    ICM20948_Write(ICM20948_B0_PWR_MGMT_2, 0x00);
}

void ICM20948_SetBank(uint8_t bank)
{
  bank = (bank << 4) & 0x30; // bits 5:4 of REG_BANK_SEL
  if (icm20948.bank == bank) return;

  icm20948.bank = bank;
  ICM20948_Write(ICM20948_REG_BANK_SEL, icm20948.bank);
}

uint8_t ICM20948_GetWhoami(void)
{
  ICM20948_SetBank(ICM20948_B0);
  return ICM20948_Read(ICM20948_B0_WHO_AM_I);
}

void ICM20948_SetSleep(uint8_t status)
{
    uint8_t dat;
    ICM20948_SetBank(ICM20948_B0);
    dat = ICM20948_Read(ICM20948_B0_PWR_MGMT_1);
    if (status == 0x00) dat &= ~(0x01 << 6);
    else dat |= (0x01 << 6);
    ICM20948_Write(ICM20948_B0_PWR_MGMT_1, dat);
}

void ICM20948_SetLowPower(uint8_t status)
{
    uint8_t dat;
    ICM20948_SetBank(ICM20948_B0);
    dat = ICM20948_Read(ICM20948_B0_PWR_MGMT_1);
    if (status == 0x00) dat &= ~(0x01 << 5);
    else dat |= (0x01 << 5);
    ICM20948_Write(ICM20948_B0_PWR_MGMT_1, dat);
}

void ICM20948_SetClockSource(uint8_t sel)
{
    uint8_t dat;
    ICM20948_SetBank(ICM20948_B0);
    dat = ICM20948_Read(ICM20948_B0_PWR_MGMT_1);
    dat &= ~0x07;
    dat |= sel & 0x07;
    ICM20948_Write(ICM20948_B0_PWR_MGMT_1, dat);
}

void ICM20948_ReadAll(int16_t *buff)
{
    uint8_t i, tmp;
    uint8_t *buff8b = (uint8_t *)buff;
    ICM20948_SetBank(ICM20948_B0);
    ICM20948_ReadInBatch(ICM20948_B0_ACCEL_XOUT_H, buff8b, 14);
    for (i = 0; i < 7; i++)
    {
        tmp = buff8b[i * 2];
        buff8b[i * 2] = buff8b[i * 2 + 1];
        buff8b[i * 2 + 1] = tmp;
    }
}

void ICM20948_SetGyroConfig(uint8_t fchoice, uint8_t fssel, uint8_t dlpfconf)
{
    uint8_t tmp = (fchoice & 0x01) | ((fssel & 0x03) << 1) | ((dlpfconf & 0x07) << 3);
    ICM20948_SetBank(ICM20948_B2);
    ICM20948_Write(ICM20948_B2_GYRO_CONFIG_1, tmp);
}

void ICM20948_SetAccelConfig(uint8_t fchoice, uint8_t fssel, uint8_t dlpfconf)
{
	uint8_t tmp = (fchoice & 0x01) | ((fssel & 0x03) << 1) | ((dlpfconf & 0x07) << 3);
    ICM20948_SetBank(ICM20948_B2);
	ICM20948_Write(ICM20948_B2_ACCEL_CONFIG, tmp);
}

void ICM20948_SetEnBypass(uint8_t status)
{
	uint8_t dat;
    ICM20948_SetBank(ICM20948_B0);
    dat = ICM20948_Read(ICM20948_B0_INT_PIN_CFG);
    dat &= ~(0x01 << 1);
    dat |= (status & 0x01) << 1;
    ICM20948_Write(ICM20948_B0_INT_PIN_CFG, dat);
}

/** AK09916 methods */

void AK09916_Write(uint8_t reg, uint8_t dat)
{
    BSP_I2cTransmit(icm20948.mag_addr, reg, &dat, 1);
    BSP_RccNop(1);
}

uint8_t AK09916_Read(uint8_t reg)
{
    uint8_t ret;
    BSP_I2cReceive(icm20948.mag_addr, reg, &ret, 1);
    BSP_RccNop(1);
    return ret;
}

void AK09916_ReadInBatch(uint8_t reg, uint8_t *buff, uint16_t size)
{
    BSP_I2cReceive(icm20948.mag_addr, reg, buff, size);
    BSP_RccNop(1);
}

ErrorStatus AK09916_Detect(uint8_t addr)
{
    icm20948.mag_addr = addr;
    if (AK09916_Read(AK09916_MAG_WIA2) == AK09916_ID)
    {
        return SUCCESS;
    }
    return ERROR;
}

uint8_t AK09916_GetWhoami(void)
{
  return AK09916_Read(AK09916_MAG_WIA2);
}

void AK09916_Reset(void)
{
	AK09916_Write(AK09916_MAG_CNTL3, 0x01);
}

void AK09916_SetMode(uint8_t mode)
{
	AK09916_Write(AK09916_MAG_CNTL2, mode);
}

void AK09916_ReadAll(int16_t *buff)
{
    uint8_t *buff8b = (uint8_t *)buff;
    AK09916_ReadInBatch(AK09916_MAG_HXL, buff8b, 6);
    // when any of measurement data is read, be sure to read ST2 register at the end
    AK09916_Read(AK09916_MAG_ST2);
}
