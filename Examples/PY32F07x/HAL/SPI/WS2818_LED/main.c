/**
 * PY32F07x          WS2812/B
 * PA7          -->  DIN
*/
#include "main.h"
#include "py32f07x_bsp_printf.h"
#include "py32f07x_bsp_clock.h"
#include "ws2812_spi.h"


SPI_HandleTypeDef Spi1Handle = {0};

static void APP_SPI_Config(void);

int main(void)
{
  uint8_t i = 0, r = 0, g = 0x60, b = 0xC0;

  HAL_Init();
  BSP_HSI_24MHzClockConfig();

  BSP_USART_Config();
  printf("PY32F07x WS2818 Example\r\nClock: %ld\r\n", SystemCoreClock);
  
  APP_SPI_Config();

  ws2812_pixel_all(r, g, b);
  ws2812_send_spi();
  while (1)
  {
    i = (i + 1) % WS2812_NUM_LEDS;
    ws2812_pixel(i, r++, g++, b++);
    ws2812_send_spi();
    HAL_Delay(20);
  }
}

static void APP_SPI_Config(void)
{
  Spi1Handle.Instance               = SPI1;
  /* The frequency after prescale should be below 8.25MHz */
  Spi1Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  Spi1Handle.Init.Direction         = SPI_DIRECTION_2LINES;
  Spi1Handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  Spi1Handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  Spi1Handle.Init.DataSize          = SPI_DATASIZE_8BIT;
  Spi1Handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  Spi1Handle.Init.NSS               = SPI_NSS_SOFT;
  Spi1Handle.Init.Mode = SPI_MODE_MASTER;
  Spi1Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_DeInit(&Spi1Handle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_SPI_Init(&Spi1Handle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
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
