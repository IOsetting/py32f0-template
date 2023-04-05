#include "LCD1602.h"

uint8_t portLcd = 0;

static void LCD_WriteByte(uint8_t bt) {
  HAL_I2C_Master_Transmit(&LCD_I2C, ADRESS_I2C_LCD, &bt, 1, 1000);
}

static void LCD_SendCmd_8bit(uint8_t bt) {
  LCD_WriteByte(portLcd |= 0x04);
  HAL_Delay(1);
  LCD_WriteByte(portLcd | bt);
  LCD_WriteByte(portLcd &= ~ 0x04);
  HAL_Delay(1);
}

static void LCD_SendCmd(uint8_t bt) {
  bt <<= 4;
  LCD_SendCmd_8bit(bt);
}

static void LCD_SendByte(uint8_t bt, uint8_t mode) {
  if (mode == 0) {
    LCD_WriteByte(portLcd &= ~ 0x01); // RS = 0;
  }
  else {
    LCD_WriteByte(portLcd |= 0x01); // RS = 1;
  }

  uint8_t tempBuf = 0;
  tempBuf = bt >> 4;

  LCD_SendCmd(tempBuf);
  LCD_SendCmd(bt);
}

void LCD_Init(void) {
  HAL_Delay(50);
  LCD_SendCmd(0x03);
  HAL_Delay(5);
  LCD_SendCmd(0x03);
  HAL_Delay(1);
  LCD_SendCmd(0x03);
  HAL_Delay(10);

  LCD_SendCmd(0x02); // 0x02 // 4bit mode
  HAL_Delay(2);

  LCD_SendByte(0x38, 0);
  HAL_Delay(2);

  LCD_SendByte(0x02, 0);
  HAL_Delay(2);
  LCD_SendByte(0x0C, 0);
  HAL_Delay(2);
  LCD_SendByte(0x01, 0);
  HAL_Delay(2);
  LCD_SendByte(0x06, 0);
  HAL_Delay(1);
  //---------------------------------------------------------------------------------

  LCD_WriteByte(portLcd |= 0x08);
  LCD_WriteByte (portLcd &= ~ 0x02);
}

void LCD_PrintString(char* str) {
  uint8_t i = 0;

  while (str[i] != 0) {
    LCD_SendByte(str[i], 1);
    i++;
  }
}

void LCD_Clear(void) {

  LCD_SendByte(0x01, 0);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t x, uint8_t y) {
  switch (y) {
    case 0:
      LCD_SendByte(x | 0x80, 0);
      HAL_Delay(1);
      break;
    case 1:
      LCD_SendByte(( 0x40 + x) | 0x80, 0);
      HAL_Delay(1);
      break;
    case 2:
      LCD_SendByte(( 0x14 + x) | 0x80, 0);
      HAL_Delay(1);
      break;
    case 3:
      LCD_SendByte(( 0x54 + x) | 0x80, 0);
      HAL_Delay(1);
      break;
    default:
      break;
  }
}
