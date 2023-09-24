/***
 * Demo: ST7567 LCD
 * 
 * PY32          ST7567
 * PA0   ------> Reset
 * PA1   ------> CLK/SCK
 * PA4   ------> Backlight
 * PA5   ------> DC/AO
 * PA6   ------> CSN/CE
 * PA7   ------> MOSI
 * 
 * PA2   ------> UART TX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"
#include "st7567.h"


static void APP_SPIConfig(void);

int main(void)
{
  int y1, y2;
  uint8_t d1, d2;

  BSP_RCC_HSI_24MConfig();

  BSP_USART_Config(115200);
  printf("SPI Demo: ST7567 LCD\r\nClock: %ld\r\n", SystemCoreClock);

  APP_SPIConfig();

  ST7567_Init();

  ST7567_Fill(0);
  LL_mDelay(2000);

  ST7567_DrawLine(0,   0, 127,  0, 1);
  ST7567_DrawLine(0,   0,   0, 63, 1);
  ST7567_DrawLine(127, 0, 127, 63, 1);
  ST7567_DrawLine(0,  63, 127, 63, 1);

  ST7567_GotoXY(5, 5);
  ST7567_Puts("LCD:ST7567", &Font_12x24, 1);
  ST7567_GotoXY(10, 42);
  ST7567_Puts("Font size: 12x24", &Font_6x10, 1);
  ST7567_UpdateScreen(); // display
  LL_mDelay(2000);

  ST7567_Fill(0);
  ST7567_GotoXY(3, 5);
  ST7567_Puts("ST7567 DEMO", &Font_5x7, 1);
  ST7567_GotoXY(3, 21);
  ST7567_Puts("It's a demo of ST7567 12864 LCD", &Font_3x5, 1);
  ST7567_GotoXY(3, 27);
  ST7567_Puts("Font size 3x5, nums:01234567890", &Font_3x5, 1);
  ST7567_GotoXY(5, 52);
  ST7567_Puts("Font size: 5x7", &Font_5x7, 1);
  ST7567_UpdateScreen(); 
  LL_mDelay(2000);

  ST7567_Fill(0);
  ST7567_GotoXY(5, 5);
  ST7567_Puts("ST7567 is a single-", &Font_6x8, 1);
  ST7567_GotoXY(5, 14);
  ST7567_Puts("chip dot matrix LCD", &Font_6x8, 1);
  ST7567_GotoXY(5, 23);
  ST7567_Puts("driver", &Font_6x8, 1);
  ST7567_GotoXY(10, 52);
  ST7567_Puts("PY32F0 Demo", &Font_6x12, 1);
  ST7567_UpdateScreen();
  LL_mDelay(2000);

  y1 = 10;
  while (y1 <= 0x30)
  {
      ST7567_SetContrast(y1++);
      LL_mDelay(100);
  }
  while (y1 >= 10)
  {
      ST7567_SetContrast(y1--);
      LL_mDelay(100);
  }
  while (y1 <= 0x20)
  {
      ST7567_SetContrast(y1++);
      LL_mDelay(100);
  }
  LL_mDelay(2000);

  ST7567_ToggleInvert();
  ST7567_UpdateScreen();
  LL_mDelay(2000);

  ST7567_ToggleInvert();
  ST7567_UpdateScreen();
  LL_mDelay(2000);

  ST7567_Fill(0);
  y1 = 64, y2 = 0;
  while (y1 > 0)
  {
      ST7567_DrawLine(0, y1, 127, y2, 1);
      ST7567_UpdateScreen();
      y1 -= 2;
      y2 += 2;
      LL_mDelay(30);
  }
  LL_mDelay(1000);

  ST7567_Fill(0);
  y1 = 127, y2 = 0;
  while (y1 > 0)
  {
      ST7567_DrawLine(y1, 0, y2, 63, 1);
      ST7567_UpdateScreen();
      y1 -= 2;
      y2 += 2;
      LL_mDelay(30);
  }
  LL_mDelay(500);

  ST7567_Fill(1);
  ST7567_UpdateScreen();
  ST7567_DrawCircle(64, 32, 25, 0);
  ST7567_UpdateScreen();
  LL_mDelay(100);
  ST7567_DrawCircle(128, 32, 25, 0);
  ST7567_UpdateScreen();
  LL_mDelay(100);
  ST7567_DrawCircle(0, 32, 25, 0);
  ST7567_UpdateScreen();
  LL_mDelay(100);
  ST7567_DrawCircle(32, 32, 25, 0);
  ST7567_UpdateScreen();
  LL_mDelay(100);
  ST7567_DrawCircle(96, 32, 25, 0);
  ST7567_UpdateScreen();
  LL_mDelay(500);

  ST7567_Fill(0);
  ST7567_UpdateScreen();
  int32_t i = -100;
  char buf[10];
  while (i <= 100)
  {
      memset(&buf[0], 0, sizeof(buf));
      sprintf(buf, "%ld", i);
      ST7567_GotoXY(50, 27);
      ST7567_Puts(buf, &Font_6x10, 1);
      ST7567_DrawLine(64, 10, (i + 100) * 128 / 200, (i + 100) * 64 / 200, 1);
      ST7567_UpdateScreen();
      ST7567_Fill(0);
      i++;
      LL_mDelay(30);
  }
  ST7567_Fill(0);
  LL_mDelay(1000);

  /* Infinite loop */
  uint32_t loop;
  y1 = 0, y2 = 0, d1 = 0, d2 = 0;
  while(1)
  {
      for (loop = 0; loop < 0x08; loop++)
      {
          ST7567_GotoXY(y1, y2);
          ST7567_Puts(buf, &Font_6x12, 1);
          ST7567_GotoXY(y1+10, y2+17);
          ST7567_Puts("ST7567", &Font_6x8, 1);
          ST7567_GotoXY(y1+20, y2+33);
          ST7567_Puts("IOsetting", &Font_6x10, 1);
          ST7567_UpdateScreen();
          ST7567_Fill(0);
      }
      LL_mDelay(100);
      if (d1 == 0)
      {
          y1++;
          if (y1 == 54)
          {
              d1 = 1;
              if (d2 == 0)
              {
                  y2 +=4;
                  if (y2 == 20) d2 = 1;
              }
              else
              {
                  y2 -= 4;
                  if (y2 == 0) d2 = 0;
              }
          }
      }
      else
      {
          y1--;
          if (y1 == 0) d1 = 0;
      }
  }
}

uint8_t SPI_TxRxByte(uint8_t data)
{
  uint8_t SPITimeout = 0xFF;
  /* Check the status of Transmit buffer Empty flag */
  while (READ_BIT(SPI1->SR, SPI_SR_TXE) == RESET)
  {
    if (SPITimeout-- == 0) return 0;
  }
  LL_SPI_TransmitData8(SPI1, data);
  SPITimeout = 0xFF;
  while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == RESET)
  {
    if (SPITimeout-- == 0) return 0;
  }
  // Read from RX buffer
  return LL_SPI_ReceiveData8(SPI1);
}

static void APP_SPIConfig(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  // PA6 CS
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  // PA5 DC/AO
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  // PA0 RESET
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
  // PA4   ------> Backlight
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);

  // PA1 SCK
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // PA7 MOSI
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_Enable(SPI1);
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
