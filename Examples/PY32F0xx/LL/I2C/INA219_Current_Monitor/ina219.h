#ifndef __INA219_H
#define __INA219_H

#include "main.h"

#ifdef __cplusplus
extern "C"{
#endif

#define INA219_ADDRESS                  INA219_ADDRESS_0

#define INA219_REG_CONF                 0x00 // [15]:reset
                                             // [13]:bus Voltage Range, 0 = 16V FSR, 1 = 32V FSR (default value)
                                             // [11,12]:PGA gain and range, 00:+/-40mV, 01:80, 02:160, 03:320
                                             // [7, 10]:bus ADC resolution
                                             // [3,  6]:shunt ADC resolution
                                             // [0,  2]:operation mode: continuous, triggered, or power-down
#define INA219_REG_SHUNT_VOLTAGE        0x01 // shunt voltage, the voltage drop across the shunt resistor, 1 LSB step size = 10 μV

#define INA219_REG_BUS_VOLTAGE          0x02 // bus voltage, voltage between VIN- and ground
                                             // bus Voltage bits are **not right-aligned**. In order to compute the value of the 
                                             // Bus Voltage, value must be shifted right by three bits.
                                             // [3,15]:bus voltage, 1 LSB step size = 4 mV
                                             // [1]:Conversion Ready. set after all conversions, averaging, and multiplications are 
                                             //     complete. Clear when writing a new mode into the Operating Mode bits, or reading 
                                             //     the Power Register
                                             // [0]:Math Overflow Flag, set when the Power or Current calculations are out of range

#define INA219_REG_POWER                0x03 // power, = (bus Voltage * current) / 5000

#define INA219_REG_CURRENT              0x04 // current, = (shunt Voltage *  calibration register) / 4096

#define INA219_REG_CALIBRATION          0x05 // calibration register
                                             // calibration = 0.04096 / (current_LSB * 0.1Ω)
                                             // current_LSB = Max expected current / 2^15

/**
 * @brief ina219 address enumeration definition
 */
typedef enum
{
    INA219_ADDRESS_0 = (0x40 << 1),        /**< A0 = GND, A1 = GND */
    INA219_ADDRESS_1 = (0x41 << 1),        /**< A0 = VS+, A1 = GND */
    INA219_ADDRESS_2 = (0x42 << 1),        /**< A0 = SDA, A1 = GND */
    INA219_ADDRESS_3 = (0x43 << 1),        /**< A0 = SCL, A1 = GND */
    INA219_ADDRESS_4 = (0x44 << 1),        /**< A0 = GND, A1 = VS+ */
    INA219_ADDRESS_5 = (0x45 << 1),        /**< A0 = VS+, A1 = VS+ */
    INA219_ADDRESS_6 = (0x46 << 1),        /**< A0 = SDA, A1 = VS+ */
    INA219_ADDRESS_7 = (0x47 << 1),        /**< A0 = SCL, A1 = VS+ */
    INA219_ADDRESS_8 = (0x48 << 1),        /**< A0 = GND, A1 = SDA */
    INA219_ADDRESS_9 = (0x49 << 1),        /**< A0 = VS+, A1 = SDA */
    INA219_ADDRESS_A = (0x4A << 1),        /**< A0 = SDA, A1 = SDA */
    INA219_ADDRESS_B = (0x4B << 1),        /**< A0 = SCL, A1 = SDA */
    INA219_ADDRESS_C = (0x4C << 1),        /**< A0 = GND, A1 = SCL */
    INA219_ADDRESS_D = (0x4D << 1),        /**< A0 = VS+, A1 = SCL */
    INA219_ADDRESS_E = (0x4E << 1),        /**< A0 = SDA, A1 = SCL */
    INA219_ADDRESS_F = (0x4F << 1)         /**< A0 = SCL, A1 = SCL */
} ina219_address_t;

/**
 * @brief ina219 bus voltage enumeration definition
 */
typedef enum
{
    INA219_BUS_VOLTAGE_RANGE_16V = 0,        /**< ±16V */
    INA219_BUS_VOLTAGE_RANGE_32V = 1,        /**< ±32V */
} ina219_bus_voltage_range_t;

/**
 * @brief ina219 pga enumeration definition
 */
typedef enum
{
    INA219_PGA_40_MV  = 0,        /**< ±40 mV */
    INA219_PGA_80_MV  = 1,        /**< ±80 mV */
    INA219_PGA_160_MV = 2,        /**< ±160 mV */
    INA219_PGA_320_MV = 3,        /**< ±320 mV */
} ina219_pga_t;

/**
 * @brief ina219 adc mode enumeration definition
 */
typedef enum
{
    INA219_ADC_MODE_9_BIT_1_SAMPLES    = 0x0,        /**< 9 bit / 1 samples */
    INA219_ADC_MODE_10_BIT_1_SAMPLES   = 0x1,        /**< 10 bit / 1 samples */
    INA219_ADC_MODE_11_BIT_1_SAMPLES   = 0x2,        /**< 11 bit / 1 samples */
    INA219_ADC_MODE_12_BIT_1_SAMPLES   = 0x3,        /**< 12 bit / 1 samples */
    INA219_ADC_MODE_12_BIT_2_SAMPLES   = 0x9,        /**< 12 bit / 2 samples */
    INA219_ADC_MODE_12_BIT_4_SAMPLES   = 0xA,        /**< 12 bit / 4 samples */
    INA219_ADC_MODE_12_BIT_8_SAMPLES   = 0xB,        /**< 12 bit / 8 samples */
    INA219_ADC_MODE_12_BIT_16_SAMPLES  = 0xC,        /**< 12 bit / 16 samples */
    INA219_ADC_MODE_12_BIT_32_SAMPLES  = 0xD,        /**< 12 bit / 32 samples */
    INA219_ADC_MODE_12_BIT_64_SAMPLES  = 0xE,        /**< 12 bit / 64 samples */
    INA219_ADC_MODE_12_BIT_128_SAMPLES = 0xF,        /**< 12 bit / 128 samples */
} ina219_adc_mode_t;

/**
 * @brief ina219 mode enumeration definition
 */
typedef enum
{
    INA219_MODE_POWER_DOWN                   = 0x0,        /**< power down */
    INA219_MODE_SHUNT_VOLTAGE_TRIGGERED      = 0x1,        /**< shunt voltage triggered */
    INA219_MODE_BUS_VOLTAGE_TRIGGERED        = 0x2,        /**< bus voltage triggered */
    INA219_MODE_SHUNT_BUS_VOLTAGE_TRIGGERED  = 0x3,        /**< shunt and bus triggered */
    INA219_MODE_ADC_OFF                      = 0x4,        /**< adc off */
    INA219_MODE_SHUNT_VOLTAGE_CONTINUOUS     = 0x5,        /**< shunt voltage continuous */
    INA219_MODE_BUS_VOLTAGE_CONTINUOUS       = 0x6,        /**< bus voltage continuous */
    INA219_MODE_SHUNT_BUS_VOLTAGE_CONTINUOUS = 0x7,        /**< shunt and bus voltage continuous */
} ina219_mode_t;

typedef enum
{
    INA219_RESET_NO                     = 0x0,
    INA219_RESET_YES                    = 0x1,
} ina219_reset_t;

uint16_t INA219_Read(uint8_t addr);
void INA219_Write(uint8_t addr, uint16_t dat);

#ifdef __cplusplus
}
#endif

#endif