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

#ifndef __SPL06_H
#define __SPL06_H

#include "main.h"

#define SPL06_ADDR                      0x77 << 1   // Default I2C address from datasheet
#define SPL06_ADDR_ALT                  0x76 << 1   // Alternate I2C address if SDO pin is pulled-down to GND
#define SPL06_PRODUCT_ID                0x10        // SPL06-001 Product ID

/**
 * Register Address
*/
#define SPL06_PSR_B2            0x00 // Pressure[23:16](r), MSB
#define SPL06_PSR_B1            0x01 // Pressure[15:8](r)
#define SPL06_PSR_B0            0x02 // Pressure[7:0](r), LSB

#define SPL06_TMP_B2            0x03 // Temperature[23:16](r), MSB
#define SPL06_TMP_B1            0x04 // Temperature[15:8](r)
#define SPL06_TMP_B0            0x05 // Temperature[7:0](r), LSB

#define SPL06_PSR_CFG           0x06 // PM_RATE [6:4](rw), PM_PRC [3:0](rw)
                                     // - Pressure measurement rate: 
                                     //   000 1/s, 001 2/s, 010 4/s, 011 8/s, 100 16/s, 101 32/s, 110 64/s, 111 128/s
                                     // - Pressure oversampling rate:
                                     //   0000 1 time 3.6ms, 0001 2 times(Low Power) 5.2ms, 0010 4 8.4ms, 0011 8 14.8ms, 
                                     //   0100 16(Standard) 27.6ms, 0101 32 53.2ms, 0110 64 (High Precision) 104.4ms, 0111 128 206.8ms

#define SPL06_TMP_CFG           0x07 // TMP_EXT[7](rw), TMP_RATE [6:4](rw), TM_PRC [2:0](rw)
                                     // same as PSR_CFG

#define SPL06_MEAS_CFG          0x08 // COEF_RDY[7](r), SENSOR_RDY[6](r), TMP_RDY[5](r), PRS_RDY[4](r), MEAS_CRTL [2:0](rw)
                                     // - Measurement control: 000 Idle/Stop, 001 Pressure, 010 Temperature, 011 na, 100 na,
                                     //   101 Continuous pressure, 110 Continuous temperature, 111 Continuous both

#define SPL06_CFG_REG           0x09 // INT_HL[7](rw), INT_SEL[6:4](rw), TMP_SHIFT_EN[3](rw), PRS_SHIFT_EN[2](rw), FIFO_EN[1](rw)
                                     // - INT_HL: Interrupt (on SDO pin) active level: 0 - Active low, 1 - Active high.
                                     // - INT_FIFO: Generate interrupt when the FIFO is full: 0 - Disable, 1 - Enable.

#define SPL06_INT_STS           0x0A // INT_FIFO_FULL[2](r), INT_TMP[1](r), INT_PRS[0](r)
#define SPL06_FIFO_STS          0x0B // FIFO_FULL[1](r), FIFO_EMPTY[0](r)
#define SPL06_SOFT_RESET        0x0C // FIFO_FLUSH[7](w), SOFT_RST [3:0] (w)
#define SPL06_DEVICE_ID         0x0D // PROD_ID [7:4](r), REV_ID [3:0](r)
#define SPL06_COEF_C0           0x10 // C0[11:4]MSB
#define SPL06_COEF_C0C1         0x11 // C0[7:4]LSB, C1[3:0]MSB
#define SPL06_COEF_C1           0x12 // C1[7:0]
#define SPL06_COEF_C00a         0x13 // C00[7:0]MSB
#define SPL06_COEF_C00b         0x14 // C00[7:0]
#define SPL06_COEF_C00C10       0x15 // C00[7:4]LSB, C10[3:0]MSB
#define SPL06_COEF_C10a         0x16 // C10[7:0]
#define SPL06_COEF_C10b         0x17 // C10[7:0]LSB
#define SPL06_COEF_C01a         0x18 // C01[7:0]MSB
#define SPL06_COEF_C01b         0x19 // C01[7:0]LSB
#define SPL06_COEF_C11a         0x1A // C11[7:0]MSB
#define SPL06_COEF_C11b         0x1B // C11[7:0]LSB
#define SPL06_COEF_C20a         0x1C // C20[7:0]MSB
#define SPL06_COEF_C20b         0x1D // C20[7:0]LSB
#define SPL06_COEF_C21a         0x1E // C21[7:0]MSB
#define SPL06_COEF_C21b         0x1F // C21[7:0]LSB
#define SPL06_COEF_C30a         0x20 // C30[7:0]MSB
#define SPL06_COEF_C30b         0x21 // C30[7:0]LSB


typedef struct {
    uint8_t addr, pmrate,psrate, tmrate, tsrate;
    int16_t c0, c1, c01, c11, c20, c21, c30;
    int32_t c00, c10;
} SPL06_t;

typedef enum
{
    SPL06_Mode_Stop                     = 0x00,
    SPL06_Mode_Pressure                 = 0x01,
    SPL06_Mode_Temperature              = 0x02,
    SPL06_Mode_ContinuousPressure       = 0x05,
    SPL06_Mode_ContinuousTemperature    = 0x06,
    SPL06_Mode_ContinuousBoth           = 0x07,
} SPL06_Mode_t;

typedef enum
{
    SPL06_Rate_X1     = 0x00,
    SPL06_Rate_X2     = 0x01,
    SPL06_Rate_X4     = 0x02,
    SPL06_Rate_X8     = 0x03,
    SPL06_Rate_X16    = 0x04,
    SPL06_Rate_X32    = 0x05,
    SPL06_Rate_X64    = 0x06,
    SPL06_Rate_X128   = 0x07,
} SPL06_Rate_t;


uint8_t SPL06_Read(uint8_t addr);
void SPL06_ReadInBatch(uint8_t *buf, uint8_t size);
void SPL06_Write(uint8_t addr, uint8_t dat);
uint8_t SPL06_Detect(uint8_t address);
void SPL06_Init(uint8_t address, SPL06_Mode_t mode);
void SPL06_ReadCalibParam(void);
void SPL06_SetPressureRate(SPL06_Rate_t measurementRate, SPL06_Rate_t overSamplingRate);
void SPL06_SetTemperatureRate(SPL06_Rate_t measurementRate, SPL06_Rate_t overSamplingRate);
void SPL06_ChangeMode(SPL06_Mode_t mode);
void SPL06_Reset(void);
/** data[0]:pressure, data[1]:temperature */
void SPL06_GetBoth(int32_t *data);
float SPL06_GetTemperature(int32_t traw);
float SPL06_GetPressure(int32_t traw, int32_t praw);


#endif // __SPL06_H
