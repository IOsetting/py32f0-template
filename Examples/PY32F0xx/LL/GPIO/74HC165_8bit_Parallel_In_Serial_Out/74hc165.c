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

#include <stdio.h>
#include "74hc165.h"

uint8_t HC165_Read(void)
{
    uint8_t i, data = 0;

    HC165_LD_LOW();  // Pull down LD to load parallel inputs
    HC165_LD_HIGH(); // Pull up to inhibit parallel loading

    for (i = 0; i < 8; i++)
    {
        data = data << 1;
        HC165_SCK_LOW();
        __NOP(); // NOP to ensure reading correct value
        if (HC165_DATA_READ())
        {
            data |= 0x01;
        }
        HC165_SCK_HIGH();
    }
    return data;
}