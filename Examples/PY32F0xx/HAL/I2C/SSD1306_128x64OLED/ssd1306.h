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

#ifndef __SSD1306_H__
#define __SSD1306_H__

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif


#include <stdlib.h>
#include <string.h>
#include "ascii_fonts.h"

/* I2C address
* address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
*/
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR         0x78
//#define SSD1306_I2C_ADDR       0x7A
#endif

/* SSD1306 settings */
/* SSD1306 width in pixels */
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH            128
#endif
/* SSD1306 LCD height in pixels */
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT           64
#endif

#ifndef SSD1306_TIMEOUT
#define SSD1306_TIMEOUT					20000
#endif

#define SSD1306_COLOR_BLACK 0x00
#define SSD1306_COLOR_WHITE 0x01

uint8_t SSD1306_Init(void);
void SSD1306_UpdateScreen(void);
void SSD1306_ToggleInvert(void);
void SSD1306_Fill(uint8_t Color);
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color);
void SSD1306_GotoXY(uint16_t x, uint16_t y);
char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color);
char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color);
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t c);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c);
void SSD1306_Image(uint8_t *img, uint8_t frame, uint8_t x, uint8_t y);

/**
 * @brief  Writes single byte command to slave
 * @param  command: command to be written
 * @retval None
 */
void SSD1306_WriteCommand(uint8_t command);

/**
 * @brief  Writes single byte data to slave
 * @param  data: data to be written
 * @retval None
 */
void SSD1306_WriteData(uint8_t data);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
