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

#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "py32f0xx_ll_i2c.h"
#include "py32f0xx_ll_utils.h"

void BSP_I2C_Config(void);
void BSP_I2C_Scan(void);
ErrorStatus BSP_I2C_IsDeviceReady(uint8_t devAddress, uint16_t timeout);
ErrorStatus BSP_I2C_MasterTransmit(uint16_t devAddress, uint8_t *pData, uint16_t len, uint16_t timeout);
ErrorStatus BSP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t len, uint16_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_H */
