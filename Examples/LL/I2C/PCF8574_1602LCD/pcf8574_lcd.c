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

#include "pcf8574_lcd.h"


ErrorStatus LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags)
{
  ErrorStatus status;
  for(;;)
  {
    status = BSP_I2C_IsDeviceReady(lcd_addr, 5000);
    if(status == SUCCESS)
    {
      break;
    }
  }

  uint8_t up = data & 0xF0;
  uint8_t lo = (data << 4) & 0xF0;

  uint8_t data_arr[4];
  data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
  data_arr[1] = up|flags|BACKLIGHT;
  data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
  data_arr[3] = lo|flags|BACKLIGHT;

  status = BSP_I2C_MasterTransmit(lcd_addr, data_arr, sizeof(data_arr), 5000);
  LL_mDelay(LCD1602_DELAY);
  return status;
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd)
{
  LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data)
{
  LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_SetCGRAM(uint8_t lcd_addr, uint8_t char_addr, const uint8_t *char_font)
{
  uint8_t i = 8;

  LCD_SendCommand(lcd_addr, LCD1602_CMD_CGRAM_ADDR | (char_addr << 3));

  while(i--)
  {
    LCD_SendData(lcd_addr, *char_font++ );
  }

  LCD_SendCommand(lcd_addr, LCD1602_CMD_CGRAM_ADDR);
}

void LCD_Init(uint8_t lcd_addr)
{
  // 4-bit mode, 2 lines, 5x8 format
  LCD_SendCommand(lcd_addr, LCD1602_CMD_FUNC_4B_2L_5X8);
  // display & cursor home
  LCD_SendCommand(lcd_addr, LCD1602_CMD_HOME);
  // display on, right shift, underline off, blink off
  LCD_SendCommand(lcd_addr, LCD1602_CMD_MODE_ON_CURSOR_BLNK);
  // move direction right
  LCD_SendCommand(lcd_addr, LCD1602_CMD_DIRECTION_RIGHT);
  // clear display (optional here)
  LCD_SendCommand(lcd_addr, LCD1602_CMD_CLEAR_DISPLAY);
}

void LCD_SendString(uint8_t lcd_addr, char *str)
{
  while (*str)
  {
    LCD_SendData(lcd_addr, (uint8_t)(*str));
    str++;
  }
}