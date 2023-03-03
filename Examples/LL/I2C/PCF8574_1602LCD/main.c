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

/***
 * Demo: I2C - PCF8574 1602 LCD 
 * 
 * PY32          PCF8574 1602 LCD
 *  PF1/PA9       SCL
 *  PF0/PA10      SDA
 * 
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "pcf8574_lcd.h"

const uint8_t cgrom[] = { 
  0x1f, 0x1f, 0x1f, 0x1f, 0x00, 0x00, 0x00, 0x00, /* ""   0 */
  0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, /* ,,   1 */
  0x01, 0x07, 0x0f, 0x0f, 0x1f, 0x1f, 0x1f, 0x1f, /* /|   2 */
  0x10, 0x1c, 0x1e, 0x1e, 0x1f, 0x1f, 0x1f, 0x1f, /* |\   3 */
  0x1f, 0x1f, 0x1f, 0x1f, 0x0f, 0x0f, 0x07, 0x01, /* \|   4 */
  0x1f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1e, 0x1c, 0x10, /* |/   5 */
  0x00, 0x00, 0x00, 0x00, 0x10, 0x1c, 0x1e, 0x1e, /* |\   6 */
  0x00, 0x00, 0x00, 0x0f, 0x0f, 0x0f, 0x0f, 0x00  /* .    7 */
};

static void APP_GPIO_Config(void);

int main(void)
{
  uint8_t i;

  BSP_RCC_HSI_24MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: PCF8574 1602 LCD\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIO_Config();
  BSP_I2C_Config();

  printf("Scanning I2C bus...\r\n");
  BSP_I2C_Scan();

  LCD_Init(LCD1602_I2C_ADDR);

  for(;;)
  {
    // clear display
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_CMD_CLEAR_DISPLAY);
    LL_mDelay(500);
    // move cursor to 0,0
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_DDRAM_ROW0|0);
    LCD_SendString(LCD1602_I2C_ADDR, " Using 1602 LCD");
    // move cursor to 1,0
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_DDRAM_ROW1|0);
    LCD_SendString(LCD1602_I2C_ADDR, "  over I2C bus");
    LL_mDelay(1500);

    // CGRAM test
    for (i = 0; i < 8; i++)
    {
      LCD_SetCGRAM(LCD1602_I2C_ADDR, i, &cgrom[i * 8]);
    }
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_CMD_CLEAR_DISPLAY);
    LL_mDelay(500);
    // move cursor to 0,0
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_DDRAM_ROW0|0);
    LCD_SendString(LCD1602_I2C_ADDR, "Custom chars");
    LL_mDelay(500);
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_DDRAM_ROW1|0);
    for (i = 0; i < 8; i++)
    {
      LCD_SendData(LCD1602_I2C_ADDR, i);
      LL_mDelay(200);
    }
    LCD_SendString(LCD1602_I2C_ADDR, " done");
    LL_mDelay(1500);

    // Shift display test
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_CMD_CLEAR_DISPLAY);
    LL_mDelay(500);
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_DDRAM_ROW0|9);
    LCD_SendString(LCD1602_I2C_ADDR, "Shift");
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_DDRAM_ROW1|8);
    LCD_SendString(LCD1602_I2C_ADDR, "<<<->>>");
    LL_mDelay(500);
    for (i = 0; i < 8; i++)
    {
      LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_CMD_DISPLAY_SHIFT_LEFT);
      LL_mDelay(200);
    }
    LL_mDelay(500);
    for (i = 0; i < 8; i++)
    {
      LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_CMD_DISPLAY_SHIFT_RIGHT);
      LL_mDelay(200);
    }
    LL_mDelay(1500);

    // Move cursor test
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_CMD_CLEAR_DISPLAY);
    LL_mDelay(500);
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_DDRAM_ROW0|0);
    LCD_SendString(LCD1602_I2C_ADDR, "Move cursor");
    LL_mDelay(500);
    for (i = 0; i < 11; i++)
    {
      LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_CMD_CURSOR_MOVE_LEFT);
      LL_mDelay(200);
    }
    LL_mDelay(500);
    for (i = 0; i < 12; i++)
    {
      LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_CMD_CURSOR_MOVE_RIGHT);
      LL_mDelay(200);
    }
    LL_mDelay(500);
    LCD_SendCommand(LCD1602_I2C_ADDR, LCD1602_DDRAM_ROW1|11);
    LCD_SendString(LCD1602_I2C_ADDR, "done");
    
    LL_mDelay(2000);
  }
}

static void APP_GPIO_Config(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /**
   * SCL: PF1 & AF_12, PA9 & AF_6
   * SDA: PF0 & AF_12, PA10 & AF_6
   * 
   * Change pins to PF1 / PF0 for parts have no PA9 / PA10
  */
  // PF1 SCL
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  // PF0 SDA
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
