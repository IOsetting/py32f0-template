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

/*-------------------------------------------------------------
*   Instruction     D7  D6  D5  D4  D3  D2  D1  D0 
*   ============================================== 
*   Display clear   0   0   0   0   0   0   0   1  
*   Cursor home     0   0   0   0   0   0   1   *  
*   Entry Mode Set  0   0   0   0   0   1  I/D  S  
*   Display On/Off  0   0   0   0   1   D   C   B  
*   Curs/Disp shift 0   0   0   1  S/C R/L  *   *  
*   Function Set    0   0   1   DL  N   F   *   *  
*   CG RAM addr set 0   1   ---------Acg---------  
*   DD RAM addr set 1   -------------Add---------  
*                                                              
*   Meaning:                                                   
*   *     - nonvalid bit                                       
*   Acg   - CG RAM address (CHARACTER GENERATOR)               
*   Add   - DD RAM address (DATA DISPLAY)                      
*   AC    - adress counter                                     
*                                                              
*   I/D   - 1-increment, 0-decrement                           
*   S     - 1-display shift, 0-no display shift                
*   D     - 1-display ON, 0-display OFF                        
*   C     - 1-cursor ON, 0-cursor OFF                          
*   B     - 1-blink ON, 0-blink OFF                            
*   S/C   - 1-display shift, 0-cursor movement                 
*   R/L   - 1-right shift, 0-left shift                        
*   DL    - 1-8 bits data transfer, 0-4 bits data transfer     
*   N     - 1-two lines, 0-one line                            
*   F     - 1-5x10 dot matrix, 0-5x8 dot matrix                
*   BF    - 1-internal operation in progress, 0-display ready  
*                                                              
\**************************************************************/

#ifndef __PCF8574_LCD_H__
#define __PCF8574_LCD_H__

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <stdlib.h>
#include <string.h>
#include "main.h"


/* I2C address
 * - 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * - 0x4E or 0x7E
*/
#define LCD1602_I2C_ADDR  0x7E
/* Delay in millisecond */
#define LCD1602_DELAY 5
/* Register selection */
#define PIN_RS    (1 << 0)
/* Read/Write */
#define PIN_RW    (1 << 1)
/* Chip enable */
#define PIN_EN    (1 << 2)
/* Back light - might not be available on some PCF8574 modules */
#define BACKLIGHT (1 << 3)

/* Clear display */
#define LCD1602_CMD_CLEAR_DISPLAY               0b00000001
/* Move cursor home */
#define LCD1602_CMD_HOME                        0b00000010

// Entry Mode, Set cursor/display moving direction
#define LCD1602_CMD_DIRECTION_RIGHT             0b00000110
#define LCD1602_CMD_DIRECTION_LEFT              0b00000100
#define LCD1602_CMD_DIRECTION_RIGHT_SHIFT       0b00000111
#define LCD1602_CMD_DIRECTION_LEFT_SHIFT        0b00000101
// Display mode
#define LCD1602_CMD_MODE_OFF                    0b00001000
#define LCD1602_CMD_MODE_ON_CURSOR_OFF          0b00001100
#define LCD1602_CMD_MODE_ON_CURSOR_ON           0b00001110
#define LCD1602_CMD_MODE_ON_CURSOR_BLNK         0b00001111
// Cursor/Display Shift
#define LCD1602_CMD_CURSOR_MOVE_LEFT            0b00010000
#define LCD1602_CMD_CURSOR_MOVE_RIGHT           0b00010100
#define LCD1602_CMD_DISPLAY_SHIFT_LEFT          0b00011000
#define LCD1602_CMD_DISPLAY_SHIFT_RIGHT         0b00011100

/* Function set: 4-bit, 1 row, 5X8 matrix */
#define LCD1602_CMD_FUNC_4B_1L_5X8              0b00100000
/* Function set: 4-bit, 2 row, 5X8 matrix */
#define LCD1602_CMD_FUNC_4B_2L_5X8              0b00101000
/* Function set: 8-bit, 1 row, 5X8 matrix */
#define LCD1602_CMD_FUNC_8B_1L_5X8              0b00110000
/* Function set: 8-bit, 2 row, 5X8 matrix */
#define LCD1602_CMD_FUNC_8B_2L_5X8              0b00111000
/* Set/Read CGRAM address */
#define LCD1602_CMD_CGRAM_ADDR                  0b01000000
/* Set/Read DDRAM address */
#define LCD1602_CMD_DDRAM_ADDR                  0b10000000

/* First row address */
#define LCD1602_DDRAM_ROW0                      0b10000000
/* Second row address */
#define LCD1602_DDRAM_ROW1                      0b11000000


void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);
void LCD_SendData(uint8_t lcd_addr, uint8_t data);
void LCD_Init(uint8_t lcd_addr);
void LCD_SendString(uint8_t lcd_addr, char *str);
void LCD_SetCGRAM(uint8_t lcd_addr, uint8_t char_addr, const uint8_t *char_font);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
