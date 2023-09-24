/***
 * Demo: I2C - SSD1306 OLED 
 * 
 * PY32          SSD1306
 *  PF1/PA9       SCL
 *  PF0/PA10      SDA
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "ssd1306.h"

#define I2C_ADDRESS        0xA0     /* host/client address */
#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2


__IO uint32_t   i2cState  = I2C_STATE_READY;

static void APP_I2C_Config(void);

int main(void)
{
  int y1, y2;

  BSP_RCC_HSI_24MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: \r\nClock: %ld\r\n", SystemCoreClock);

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
  LL_mDelay(1000);

  SSD1306_Fill(0);
  SSD1306_GotoXY(5, 5);
  SSD1306_Puts("OLED:128x64", &Font_11x18, 1);
  SSD1306_GotoXY(10, 52);
  SSD1306_Puts("SSD1306 Demo", &Font_6x12, 1);
  SSD1306_UpdateScreen();
  LL_mDelay(1000);

  SSD1306_ToggleInvert(); // Invert display
  SSD1306_UpdateScreen();
  LL_mDelay(1000);

  SSD1306_ToggleInvert(); // Invert display
  SSD1306_UpdateScreen();
  LL_mDelay(1000);

  SSD1306_Fill(0);
  y1 = 64, y2 = 0;
  while (y1 > 0)
  {
      SSD1306_DrawLine(0, y1, 127, y2, 1);
      SSD1306_UpdateScreen();
      y1 -= 2;
      y2 += 2;
  }
  LL_mDelay(1000);

  SSD1306_Fill(0);
  y1 = 127, y2 = 0;
  while (y1 > 0)
  {
      SSD1306_DrawLine(y1, 0, y2, 63, 1);
      SSD1306_UpdateScreen();
      y1 -= 2;
      y2 += 2;
  }
  LL_mDelay(1000);

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
  LL_mDelay(1000);

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

  LL_I2C_InitTypeDef I2C_InitStruct;
  /* 
   * Clock speed:
   * - standard = 100khz, if PLL is on, set system clock <= 16MHz, or i2c cannot work normally
   * - fast     = 400khz
  */
  I2C_InitStruct.ClockSpeed      = LL_I2C_MAX_SPEED_FAST;
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1     = I2C_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  LL_I2C_Init(I2C1, &I2C_InitStruct);

  /* Enale clock stretch (reset default: on) */
  // LL_I2C_EnableClockStretching(I2C1);
  
  /* Enable general call (reset default: off) */
  // LL_I2C_EnableGeneralCall(I2C1);
}

void APP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t len)
{
  while (i2cState == I2C_STATE_BUSY_TX);

  LL_I2C_DisableBitPOS(I2C1);

  i2cState = I2C_STATE_BUSY_TX;

  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  while (LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, memAddress);
  while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);

  /* Transfer data */
  while (len > 0)
  {
    while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
    LL_I2C_TransmitData8(I2C1, *pData++);
    len--;

    if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (len != 0U))
    {
      LL_I2C_TransmitData8(I2C1, *pData++);
      len--;
    }

    while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
  }

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);
  i2cState = I2C_STATE_READY;
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
