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

#include "qmi8658.h"
#include "bsp_i2c.h"
#include "bsp_uart.h"

QMI8658_t qmi8658;

void QMI8658_Write(uint8_t reg, uint8_t dat)
{
    BSP_I2cTransmit(qmi8658.addr, reg, &dat, 1);
    BSP_RccNop(1);
}

uint8_t QMI8658_Read(uint8_t reg)
{
    uint8_t ret;
    BSP_I2cReceive(qmi8658.addr, reg, &ret, 1);
    BSP_RccNop(1);
    return ret;
}

void QMI8658_BurstRead(uint8_t reg, uint8_t *buff, uint16_t size)
{
    BSP_I2cReceive(qmi8658.addr, reg, buff, size);
    BSP_RccNop(1);
}

ErrorStatus QMI8658_Detect(uint8_t addr)
{
    qmi8658.addr = addr;
    if (QMI8658_Read(QMI8658_REG_WHOAMI) == QMI8658_ID)
    {
        return SUCCESS;
    }
    return ERROR;
}

void QMI8658_Init(void)
{
    uint8_t data;
    QMI8658_Write(QMI8658_REG_CTRL7, 0x03);
    data = QMI8658_Read(QMI8658_REG_CTRL1);
    data |= (0x03 << 5);
    data &= ~(0x01);
    QMI8658_Write(QMI8658_REG_CTRL1, data);
}

uint8_t QMI8658_GetWhoami(void)
{
  return QMI8658_Read(QMI8658_REG_WHOAMI);
}

void QMI8658_SetEnSensors(uint8_t enaccel, uint8_t engyro)
{
    uint8_t data = QMI8658_Read(QMI8658_REG_CTRL7);
    data &= ~(0x07); // G_SN, G_EN, A_EN
    data |= ((engyro & 0x01) << 1) | (enaccel & 0x01);
    QMI8658_Write(QMI8658_REG_CTRL7, data);
}

void QMI8658_SetAccelConfig(uint8_t range, uint8_t odr)
{
    uint8_t data = ((range & 0x03) << 4) | (odr & 0x0F);
    QMI8658_Write(QMI8658_REG_CTRL2, data);
}

void QMI8658_SetGyroConfig(uint8_t range, uint8_t odr)
{
    uint8_t data = ((range & 0x03) << 4) | (odr & 0x0F);
    QMI8658_Write(QMI8658_REG_CTRL3, data);
}

void QMI8658_SetLpf(uint8_t genable, uint8_t glpf, uint8_t aenable, uint8_t alpf)
{
    uint8_t data = ((glpf & 0x03) << 5) | ((genable & 0x01) << 4) | ((alpf & 0x03) << 1) | (aenable & 0x01);
    QMI8658_Write(QMI8658_REG_CTRL5, data);
}

void QMI8658_ReadAll(int16_t *buff)
{
    uint8_t *buff8b = (uint8_t *)buff;
    QMI8658_BurstRead(QMI8658_TEMP_L, buff8b, 14);
}

