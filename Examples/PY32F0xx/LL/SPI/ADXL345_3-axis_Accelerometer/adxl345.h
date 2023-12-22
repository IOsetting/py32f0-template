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

#ifndef __ADXL345_H
#define __ADXL345_H

#include "main.h"

#define ADXL345_CS_HIGH             LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define ADXL345_CS_LOW              LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)


#define ADXL345_DEFAULT_ADDRESS     (0x53) // Assumes ALT address pin low
#define ADXL345_DEVICE_ID           (0xE5)

#define ADXL345_REG_DEVID           (0x00) // R,   DEVID register holds a fixed device ID code of 0xE5 (345 octal)
#define ADXL345_REG_THRESH_TAP      (0x1D) // R/W, Tap threshold
#define ADXL345_REG_OFSX            (0x1E) // R/W, X-axis offset, The value stored in the offset registers is 
                                           //   automatically added to the acceleration data, and the resulting 
                                           //   value is stored in the output data registers
#define ADXL345_REG_OFSY            (0x1F) // R/W, Y-axis offset
#define ADXL345_REG_OFSZ            (0x20) // R/W, Z-axis offset
#define ADXL345_REG_DUR             (0x21) // R/W, Tap duration
#define ADXL345_REG_LATENT          (0x22) // R/W, Tap latency
#define ADXL345_REG_WINDOW          (0x23) // R/W, Tap window
#define ADXL345_REG_THRESH_ACT      (0x24) // R/W, Activity threshold
#define ADXL345_REG_THRESH_INACT    (0x25) // R/W, Inactivity threshold
#define ADXL345_REG_TIME_INACT      (0x26) // R/W, Inactivity time
#define ADXL345_REG_ACT_INACT_CTL   (0x27) // R/W, Axis enable control for activity and inactivity detection
#define ADXL345_REG_THRESH_FF       (0x28) // R/W, Free-fall threshold
#define ADXL345_REG_TIME_FF         (0x29) // R/W, Free-fall time
#define ADXL345_REG_TAP_AXES        (0x2A) // R/W, Axis control for single/double tap
#define ADXL345_REG_ACT_TAP_STATUS  (0x2B) // R,   Source for single/double tap
#define ADXL345_REG_BW_RATE         (0x2C) // R/W, Data rate and power mode control
#define ADXL345_REG_POWER_CTL       (0x2D) // R/W, Power-saving features control
#define ADXL345_REG_INT_ENABLE      (0x2E) // R/W, Interrupt enable control
#define ADXL345_REG_INT_MAP         (0x2F) // R/W, Interrupt mapping control
#define ADXL345_REG_INT_SOURCE      (0x30) // R,   Source of interrupts
#define ADXL345_REG_DATA_FORMAT     (0x31) // R/W, Data format control
#define ADXL345_REG_DATAX0          (0x32) // R,   X-axis data 0
#define ADXL345_REG_DATAX1          (0x33) // R,   X-axis data 1
#define ADXL345_REG_DATAY0          (0x34) // R,   Y-axis data 0
#define ADXL345_REG_DATAY1          (0x35) // R,   Y-axis data 1
#define ADXL345_REG_DATAZ0          (0x36) // R,   Z-axis data 0
#define ADXL345_REG_DATAZ1          (0x37) // R,   Z-axis data 1
#define ADXL345_REG_FIFO_CTL        (0x38) // R/W, FIFO control
#define ADXL345_REG_FIFO_STATUS     (0x39) // R,   FIFO status

#define ADXL345_MG2G_MULTIPLIER     (0.004) // 4mg per lsb

/**
 * Interrupt bits
*/
#define ADXL345_INT_DATA_READY      (0x80)
#define ADXL345_INT_SINGLE_TAP      (0x40)
#define ADXL345_INT_DOUBLE_TAP      (0x20)
#define ADXL345_INT_ACTIVITY        (0x10)
#define ADXL345_INT_INACTIVITY      (0x08)
#define ADXL345_INT_FREE_FALL       (0x04)
#define ADXL345_INT_WATERMARK       (0x02)
#define ADXL345_INT_OVERRUN         (0x01)

#define ADXL345_TAP_DETECT_AXIS_Z   (0x01)
#define ADXL345_TAP_DETECT_AXIS_Y   (0x02)
#define ADXL345_TAP_DETECT_AXIS_X   (0x04)

/**
 * @brief Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth
*/
typedef enum {
    ADXL345_DATARATE_3200_HZ  = 0x0F, // 1600Hz Bandwidth
    ADXL345_DATARATE_1600_HZ  = 0x0E, //  800Hz Bandwidth
    ADXL345_DATARATE_800_HZ   = 0x0D, //  400Hz Bandwidth
    ADXL345_DATARATE_400_HZ   = 0x0C, //  200Hz Bandwidth, 90 µA
    ADXL345_DATARATE_200_HZ   = 0x0B, //  100Hz Bandwidth, 60 µA
    ADXL345_DATARATE_100_HZ   = 0x0A, //   50Hz Bandwidth, 50 µA (default)
    ADXL345_DATARATE_50_HZ    = 0x09, //   25Hz Bandwidth, 45 µA
    ADXL345_DATARATE_25_HZ    = 0x08, // 12.5Hz Bandwidth, 40 µA
    ADXL345_DATARATE_12_5_HZ  = 0x07, // 6.25Hz Bandwidth, 34 µA
    ADXL345_DATARATE_6_25HZ   = 0x06, // 3.13Hz Bandwidth
    ADXL345_DATARATE_3_13_HZ  = 0x05, // 1.56Hz Bandwidth
    ADXL345_DATARATE_1_56_HZ  = 0x04, // 0.78Hz Bandwidth
    ADXL345_DATARATE_0_78_HZ  = 0x03, // 0.39Hz Bandwidth
    ADXL345_DATARATE_0_39_HZ  = 0x02, // 0.20Hz Bandwidth
    ADXL345_DATARATE_0_20_HZ  = 0x01, // 0.10Hz Bandwidth
    ADXL345_DATARATE_0_10_HZ  = 0x00, // 0.05Hz Bandwidth
} ADXL345_DataRate_t;


/**
 * @brief  Used with register ADXL345_REG_DATA_FORMAT to set SPI wires
 */
typedef enum {
    ADXL345_SELF_TEST_OFF    = 0x00,
    ADXL345_SELF_TEST_ON     = 0x80,
} ADXL345_SelfTest_t;

/**
 * @brief  Used with register ADXL345_REG_DATA_FORMAT to set SPI wires
 */
typedef enum {
    ADXL345_SPI_WIRE_4  = 0x00,
    ADXL345_SPI_WIRE_3  = 0x40,
} ADXL345_SPI_Wire_t;

/**
 * @brief  Used with register ADXL345_REG_DATA_FORMAT to set interrupt active level
 */
typedef enum {
    ADXL345_INT_ACTIVE_HIGH = 0x00,
    ADXL345_INT_ACTIVE_LOW  = 0x20,
} ADXL345_IntActive_t;

/**
 * @brief  Used with register ADXL345_REG_DATA_FORMAT to set resolution mode
 */
typedef enum {
    ADXL345_DATA_RESOLVE_10BIT   = 0x00,
    ADXL345_DATA_RESOLVE_FULL    = 0x08,
} ADXL345_DataResolve_t;

/**
 * @brief  Used with register ADXL345_REG_DATA_FORMAT to set data alignment
 */
typedef enum {
    ADXL345_DATA_ALIGNMENT_RIGHT = 0x00,
    ADXL345_DATA_ALIGNMENT_LEFT  = 0x04,
} ADXL345_DataAlignment_t;

/**
 * @brief  Used with register ADXL345_REG_DATA_FORMAT to set g range
 */
typedef enum {
    ADXL345_G_RANGE_2G   = 0x00, // +/- 2g (default)
    ADXL345_G_RANGE_4G   = 0x01, // +/- 4g
    ADXL345_G_RANGE_8G   = 0x02, // +/- 8g
    ADXL345_G_RANGE_16G  = 0x03, // +/- 16g
} ADXL345_G_Range_t;

uint8_t ADXL345_ReadByte(uint8_t addr);
int16_t ADXL345_ReadInt(uint8_t addr);
void ADXL345_BurstRead(uint8_t addr, uint8_t *pBuf, uint8_t size);

void ADXL345_WriteByte(uint8_t addr, uint8_t dat);

ErrorStatus ADXL345_Init(
    ADXL345_DataRate_t dataRate,
    ADXL345_SPI_Wire_t spiWire,
    ADXL345_IntActive_t intLevel,
    ADXL345_DataResolve_t resolve,
    ADXL345_DataAlignment_t alignment,
    ADXL345_G_Range_t range);
/**
 * Enable interrupts
*/
void ADXL345_SetInterrupts(uint8_t interrupts);
/**
 * Remap interrupts to INT2 (default is INT1)
*/
void ADXL345_RemapInterrupts(uint8_t interrupts);

uint8_t ADXL345_IsInterrupt(uint8_t interrupt);

void ADXL345_EnableTapDetectOnAxes(uint8_t axes);

#endif // __ADXL345_H
