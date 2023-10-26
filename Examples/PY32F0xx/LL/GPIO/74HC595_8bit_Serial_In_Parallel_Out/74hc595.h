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

#ifndef __FW_74HC595_H__
#define __FW_74HC595_H__

#include "main.h"
#include "string.h"

/* Storage register clock, RCLK or STCP */
#define HC595_STCP_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0)
#define HC595_STCP_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0)
/* shift register clock, SRCLK or SHCP */
#define HC595_SRCLK_LOW()       LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define HC595_SRCLK_HIGH()      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1)
/* serial input, SER or DS */
#define HC595_DS_LOW()          LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define HC595_DS_HIGH()         LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4)
/* NOP */
#define HC595_NOP()             __NOP()

void HC595_Write(uint8_t *data, uint8_t size);
void HC595_WriteByte(uint8_t data);

#endif
