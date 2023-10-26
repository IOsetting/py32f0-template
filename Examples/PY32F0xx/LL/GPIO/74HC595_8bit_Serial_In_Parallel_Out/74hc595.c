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

#include "74hc595.h"


void HC595_Write(uint8_t *data, uint8_t size)
{
    uint8_t i;

    HC595_STCP_LOW();
    /* Add nops to accommodate 74hc595 speed */
    HC595_NOP();
    while(size--)
    {
        i = 8;
        // iterate through the bits in each byte
        while(i--)
        {
            HC595_SRCLK_LOW();
            HC595_NOP();
            if (*(data + size) & (1 << i))
            {
                HC595_DS_HIGH();
            }
            else
            {
                HC595_DS_LOW();
            }
            HC595_NOP();HC595_NOP();
            HC595_SRCLK_HIGH();
            HC595_NOP();HC595_NOP();HC595_NOP();
        }
    }
    HC595_STCP_HIGH();
    HC595_NOP();
}

void HC595_WriteByte(uint8_t data)
{
    uint8_t i = 8;

    HC595_STCP_LOW();
    HC595_NOP();
    while(i--)
    {
        HC595_SRCLK_LOW();
        HC595_NOP();
        if (data & (1 << i))
        {
            HC595_DS_HIGH();
        }
        else
        {
            HC595_DS_LOW();
        }
        HC595_NOP();HC595_NOP();
        HC595_SRCLK_HIGH();
        HC595_NOP();HC595_NOP();HC595_NOP();
    }
    HC595_STCP_HIGH();
    HC595_NOP();
}