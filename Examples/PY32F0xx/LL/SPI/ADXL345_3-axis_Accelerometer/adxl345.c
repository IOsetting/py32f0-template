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


#include "adxl345.h"

uint8_t xbuf[3];

uint8_t ADXL345_ReadByte(uint8_t addr)
{
    ADXL345_CS_LOW;
    xbuf[0] = addr | 0x80;
    xbuf[1] = 0xFF;
    SPI_TxRxByte(xbuf[0]);
    xbuf[1] = SPI_TxRxByte(xbuf[1]);
    ADXL345_CS_HIGH;
    return xbuf[1];
}

int16_t ADXL345_ReadInt(uint8_t addr)
{
    int16_t output;
    ADXL345_CS_LOW;
    xbuf[0] = addr | 0xC0;
    xbuf[1] = 0xFF;
    xbuf[2] = 0xFF;
    SPI_TxRxByte(xbuf[0]);
    xbuf[1] = SPI_TxRxByte(xbuf[1]);
    xbuf[2] = SPI_TxRxByte(xbuf[2]);
    ADXL345_CS_HIGH;
    output = (int16_t)((((uint16_t)xbuf[2]) << 8) + xbuf[1]);
    return output;
}

void ADXL345_WriteByte(uint8_t addr, uint8_t dat)
{
    ADXL345_CS_LOW;
    xbuf[0] = addr;
    xbuf[1] = dat;
    SPI_TxRxByte(xbuf[0]);
    SPI_TxRxByte(xbuf[1]);
    ADXL345_CS_HIGH;
}

ErrorStatus ADXL345_Init(
    ADXL345_DataRate_t dataRate,
    ADXL345_SPI_Wire_t spiWire,
    ADXL345_IntActive_t intLevel,
    ADXL345_DataResolve_t resolve,
    ADXL345_DataAlignment_t alignment,
    ADXL345_G_Range_t range)
{
    if (ADXL345_ReadByte(ADXL345_REG_DEVID) == ADXL345_DEVICE_ID)
    {
        ADXL345_WriteByte(ADXL345_REG_BW_RATE, dataRate);
        ADXL345_WriteByte(ADXL345_REG_DATA_FORMAT,
            spiWire|intLevel|resolve|alignment|range);
        ADXL345_WriteByte(ADXL345_REG_POWER_CTL, 0x08); // BIT3=0/1:(测量模式/待机模式)；BIT2=0/1:(工作/休眠)；
        return SUCCESS;
    }
    else
    {
        return ERROR;
    }
}

void ADXL345_SetInterrupts(uint8_t interrupts)
{
    ADXL345_WriteByte(ADXL345_REG_INT_ENABLE, interrupts);
}

void ADXL345_RemapInterrupts(uint8_t interrupts)
{
    ADXL345_WriteByte(ADXL345_REG_INT_MAP, interrupts);
}

uint8_t ADXL345_IsInterrupt(uint8_t interrupt)
{
    uint8_t int_src = ADXL345_ReadByte(ADXL345_REG_INT_SOURCE);
    return (int_src & interrupt);
}

void ADXL345_EnableTapDetectOnAxes(uint8_t axes)
{
    ADXL345_WriteByte(ADXL345_REG_TAP_AXES, axes);
}