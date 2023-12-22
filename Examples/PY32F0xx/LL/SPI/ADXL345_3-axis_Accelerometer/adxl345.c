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

uint8_t ADXL345_ReadByte(uint8_t addr)
{
    uint8_t dat;

    ADXL345_CS_LOW;
    SPI_TxRxByte(addr | 0x80);
    dat = SPI_TxRxByte(0xFF);
    ADXL345_CS_HIGH;
    return dat;
}

int16_t ADXL345_ReadInt(uint8_t addr)
{
    uint8_t dath, datl;

    int16_t output;
    ADXL345_CS_LOW;
    SPI_TxRxByte(addr | 0xC0);
    datl = SPI_TxRxByte(0xFF);
    dath = SPI_TxRxByte(0xFF);
    ADXL345_CS_HIGH;
    output = (int16_t)((((uint16_t)dath) << 8) + datl);
    return output;
}

void ADXL345_BurstRead(uint8_t addr, uint8_t *pBuf, uint8_t size)
{
    ADXL345_CS_LOW;
    SPI_TxRxByte(addr | 0xC0);
    while (size--)
    {
        *(pBuf++) = SPI_TxRxByte(0xFF);
    }
    ADXL345_CS_HIGH;
}

void ADXL345_WriteByte(uint8_t addr, uint8_t dat)
{
    ADXL345_CS_LOW;
    SPI_TxRxByte(addr);
    SPI_TxRxByte(dat);
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

/**
 * It is recommended that the functions and interrupt mapping be done before enabling the interrupts.
 * When changing the configuration of an interrupt, it is recommended that the interrupt be disabled
 * first, by clearing the bit corresponding to that function in the INT_ENABLE register, and then the 
 * function be reconfigured before enabling the interrupt again.
*/
void ADXL345_SetInterrupts(uint8_t interrupts)
{
    ADXL345_WriteByte(ADXL345_REG_INT_ENABLE, interrupts);
}

/**
 * Any bits set to 0 in ADXL345_REG_INT_MAP register send their respective interrupts to the INT1 
 * pin, whereas bits set to 1 send their respective interrupts to the INT2 pin.
*/
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