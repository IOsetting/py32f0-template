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
 * Demo: PY32F003Wx(SOP16) I2C - PCF8574 1602 LCD
 * 
 * PY32F003W     PCF8574 1602 LCD
 *  PF1          SCL
 *  PF0          SDA
 * 
 * Note: 
 *    1. This demo is specific for PY32F003Wx of SOP16 packaging
 *       If PF0/PF2 is connected to a reset button, make sure there is no 
 *       capacitor connected to it, otherwise it will ruin the SDA signal. 
 *       And make sure no extra resistors and capacitors connected to PF1/PA14.
 * 
 *    2. After running this demo, RESET button will stop working, run demo
 *       "RestoreOptionBytes" to restore the RESET function on PF0/PF2.
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
static void APP_FlashSetOptionBytes(void);

int main(void)
{
  uint8_t i;

  BSP_RCC_HSI_24MConfig();
  /** 
   * Important: 
   * delay 2 seconds before SWD port stop working, so you will have 
   * enougth time to re-flash the MCU
  */
  LL_mDelay(2000);

  /* Check if PF0/PF2 pin has been set as GPIO pin*/
  if(READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE) == OB_RESET_MODE_RESET)
  {
    /* If not, turn off the RESET function on pin(PF0/PF2), this will reset the MCU */
    APP_FlashSetOptionBytes();
  }
  else
  {
    BSP_USART_Config(115200);
    printf("I2C Demo: PCF8574 1602 LCD\r\nClock: %ld\r\n", SystemCoreClock);
    printf("RESET has been configurated as GPIO\r\n");
  }

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
   * SCL: PF1 & AF_12
   * SDA: PF0 & AF_12
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

  /**
   * According to datasheet Page-20, other pins should be set as ANALOG.
  */
  // PA14
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_14, LL_GPIO_MODE_ANALOG);
  // PF2
  LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);

  /* Reset I2C */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
}

/**
 * Write Option Bytes to turn off the RESET function on pin(PF0/PF2), this will reset the MCU
*/
static void APP_FlashSetOptionBytes(void)
{
  FLASH_OBProgramInitTypeDef OBInitCfg;

  LL_FLASH_Unlock();
  LL_FLASH_OB_Unlock();

  OBInitCfg.OptionType = OPTIONBYTE_USER;
  OBInitCfg.USERType = OB_USER_BOR_EN | OB_USER_BOR_LEV | OB_USER_IWDG_SW | OB_USER_WWDG_SW | OB_USER_NRST_MODE | OB_USER_nBOOT1;
  /*
   * The default value: OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW | OB_WWDG_SW | OB_RESET_MODE_RESET | OB_BOOT1_SYSTEM;
  */
  OBInitCfg.USERConfig = OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW | OB_WWDG_SW | OB_RESET_MODE_GPIO | OB_BOOT1_SYSTEM;
  LL_FLASH_OBProgram(&OBInitCfg);

  LL_FLASH_Lock();
  LL_FLASH_OB_Lock();
  /* Reload option bytes */
  LL_FLASH_OB_Launch();
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
