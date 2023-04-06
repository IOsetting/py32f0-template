#ifndef _LCD1602_CONFIG_H
#define _LCD1602_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LCD1602_I2C

#if defined (LCD1602_I2C)

#define LCD_I2C        I2cHandle
#define ADRESS_I2C_LCD (0x27 << 1) //  (0x27 << 1)  (0x3F << 1)

#endif

#ifdef __cplusplus
}
#endif

#endif
