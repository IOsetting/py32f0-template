#ifndef _LCD1602_H
#define _LCD1602_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "py32f0xx_hal.h"
#include "py32f0xx_hal_i2c.h"

#include "main.h"
#include "LCD1602_config.h"

extern uint8_t portLcd;

#ifdef LCD1602_I2C
extern I2C_HandleTypeDef I2cHandle;
#endif

void LCD_Init(void);

void LCD_PrintString( char* str );

void LCD_Clear(void);

void LCD_SetCursor( uint8_t x, uint8_t y );

#ifdef __cplusplus
}
#endif

#endif
