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

#ifndef __FW_74HC165_H__
#define __FW_74HC165_H__

#include "main.h"
#include "string.h"


#define HC165_LD_LOW()          LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0)
#define HC165_LD_HIGH()         LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0)

#define HC165_SCK_LOW()         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define HC165_SCK_HIGH()        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1)

#define HC165_DATA_READ()       LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)

uint8_t HC165_Read(void);

#endif
