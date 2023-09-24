#include "py32f0xx_hal_dma.h"
#include "py32f0xx_hal_i2c.h"
#include "py32f0xx_bsp_printf.h"
#include "ssd1306.h"

#define I2C_ADDRESS        0xA0     /* host address */
#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2

I2C_HandleTypeDef I2cHandle;

void APP_ErrorHandler(void);
static void APP_I2C_Config(void);

int main(void)
{
  int y1, y2;

  HAL_Init();                                 

  BSP_USART_Config();
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  APP_I2C_Config();

  uint8_t res = SSD1306_Init();
  printf("OLED init: %d\n", res);

  SSD1306_DrawLine(0,   0, 127,  0, 1);
  SSD1306_DrawLine(0,   0,   0, 63, 1);
  SSD1306_DrawLine(127, 0, 127, 63, 1);
  SSD1306_DrawLine(0,  63, 127, 63, 1);
  SSD1306_GotoXY(5, 5);
  SSD1306_Puts("OLED:128x64", &Font_11x18, 1);
  SSD1306_GotoXY(10, 52);
  SSD1306_Puts("Font size: 11x18", &Font_6x10, 1);
  SSD1306_UpdateScreen(); // display
  HAL_Delay(1000);

  SSD1306_Fill(0);
  SSD1306_GotoXY(5, 5);
  SSD1306_Puts("OLED:128x64", &Font_11x18, 1);
  SSD1306_GotoXY(10, 52);
  SSD1306_Puts("SSD1306 Demo", &Font_6x12, 1);
  SSD1306_UpdateScreen();
  HAL_Delay(1000);

  SSD1306_ToggleInvert(); // Invert display
  SSD1306_UpdateScreen();
  HAL_Delay(1000);

  SSD1306_ToggleInvert(); // Invert display
  SSD1306_UpdateScreen();
  HAL_Delay(1000);

  SSD1306_Fill(0);
  y1 = 64, y2 = 0;
  while (y1 > 0)
  {
      SSD1306_DrawLine(0, y1, 127, y2, 1);
      SSD1306_UpdateScreen();
      y1 -= 2;
      y2 += 2;
  }
  HAL_Delay(1000);

  SSD1306_Fill(0);
  y1 = 127, y2 = 0;
  while (y1 > 0)
  {
      SSD1306_DrawLine(y1, 0, y2, 63, 1);
      SSD1306_UpdateScreen();
      y1 -= 2;
      y2 += 2;
  }
  HAL_Delay(1000);

  SSD1306_Fill(1);
  SSD1306_UpdateScreen();
  SSD1306_DrawCircle(64, 32, 25, 0);
  SSD1306_UpdateScreen();
  SSD1306_DrawCircle(128, 32, 25, 0);
  SSD1306_UpdateScreen();
  SSD1306_DrawCircle(0, 32, 25, 0);
  SSD1306_UpdateScreen();
  SSD1306_DrawCircle(32, 32, 25, 0);
  SSD1306_UpdateScreen();
  SSD1306_DrawCircle(96, 32, 25, 0);
  SSD1306_UpdateScreen();
  HAL_Delay(1000);

  SSD1306_Fill(0);
  SSD1306_UpdateScreen();
  int32_t i = -100;
  char buf[10];
  while (i <= 100)
  {
      memset(&buf[0], 0, sizeof(buf));
      sprintf(buf, "%ld", i);
      SSD1306_GotoXY(50, 27);
      SSD1306_Puts(buf, &Font_6x10, 1);
      SSD1306_DrawLine(64, 10, (i + 100) * 128 / 200, (i + 100) * 64 / 200, 1);
      SSD1306_UpdateScreen();
      SSD1306_Fill(0);
      i++;
  }
  SSD1306_GotoXY(50, 27);
  sprintf(buf, "END");
  SSD1306_Puts(buf, &Font_6x10, 1);
  SSD1306_UpdateScreen();

  while(1);
}

static void APP_I2C_Config(void)
{
  I2cHandle.Instance             = I2C;
  I2cHandle.Init.ClockSpeed      = 100000;        // 100KHz ~ 400KHz
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

void APP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t len)
{
  HAL_I2C_Mem_Write(&I2cHandle, devAddress, memAddress, I2C_MEMADD_SIZE_8BIT, pData, len, 5000);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Export assert error source and line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1);
}
#endif /* USE_FULL_ASSERT */
