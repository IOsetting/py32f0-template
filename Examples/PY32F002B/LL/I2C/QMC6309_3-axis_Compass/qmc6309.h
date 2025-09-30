#ifndef __QMC6309_H
#define __QMC6309_H

#include "main.h"

#define QMC6309_ADDR                0x7C << 1 // 0111 1100 -> 1111 100 0
#define QMC6309_CHIP_ID             0x90

// Registers
#define QMC6309_REG_CHIP_ID         0x00 // Chip ID is always 0x90
#define QMC6309_REG_X_LSB           0X01
#define QMC6309_REG_X_MSB           0x02
#define QMC6309_REG_Y_LSB           0x03
#define QMC6309_REG_Y_MSB           0x04
#define QMC6309_REG_Z_LSB           0x05
#define QMC6309_REG_Z_MSB           0x06
#define QMC6309_REG_STATUS          0x09 // [4]NVM_LOAD_DONE, [3]NVM_RDY, [2]ST_RDY, [1]OVFL, [0]DRDY
#define QMC6309_REG_CONTROL_1       0x0A // [7:5]OSR2, [4:3]OSR1, [1:0]MODE
#define QMC6309_REG_CONTROL_2       0x0B // [7]SOFT_RST, [6:4]ODR, [3:2]RNG, [1:0]SET/RESETMODE
#define QMC6309_REG_SELF_RESET      0x0E // [7]SELF TEST

// Modes
#define QMC6309_MODE_SUSPEND        0x00
#define QMC6309_MODE_NORMAL         0x01
#define QMC6309_MODE_SINGLE         0x02
#define QMC6309_MODE_CONTINUOUS     0x03

// Over Sample Rates
#define QMC6309_OSR_8               0x00
#define QMC6309_OSR_4               0x01
#define QMC6309_OSR_2               0x02
#define QMC6309_OSR_1               0x03

// Low Pass Filter
#define QMC6309_LPF_1               0x00
#define QMC6309_LPF_2               0x01
#define QMC6309_LPF_4               0x02
#define QMC6309_LPF_8               0x03
#define QMC6309_LPF_16              0x04

// Range scales
#define QMC6309_RANGE_32G           0x00
#define QMC6309_RANGE_16G           0x01
#define QMC6309_RANGE_8G            0x02

// Output Data Rates
#define QMC6309_ODR_1HZ             0x00
#define QMC6309_ODR_10HZ            0x01
#define QMC6309_ODR_50HZ            0x02
#define QMC6309_ODR_100HZ           0x03
#define QMC6309_ODR_200HZ           0x04

#ifndef M_PI 
#define M_PI 3.14159265358979323846264338327950288f 
#endif

#define QMC6309_DECLINATION_ANGLE  93.67 * 180 / 1000 / M_PI  // degree, Tekirdag/Turkey


typedef struct {
    uint8_t addr;
} QMC6309_t;

uint8_t QMC6309_Read(uint8_t addr);
void QMC6309_ReadInBatch(uint8_t addr, uint8_t *buf, uint8_t size);
void QMC6309_Write(uint8_t addr, uint8_t dat);

ErrorStatus QMC6309_Detect(uint8_t address);
void QMC6309_Init(void);
void QMC6309_ReadAll(int16_t *buf);
void QMC6309_SetMode(uint8_t mode, uint8_t osr, uint8_t lpf, uint8_t rng, uint8_t odr);
int32_t QMC6309_Heading(int16_t Xraw, int16_t Yraw, int16_t Zraw);

#endif
