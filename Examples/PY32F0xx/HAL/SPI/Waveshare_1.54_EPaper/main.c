/***
 * Demo: Waveshare 1.54' E-Paper
 * 
 * This demo requires 7.1 KByte RAM
 * 
 * PY32          E-Paper
 * PA0   ------> Reset 
 * PA1   ------> SCL/SCK
 * PA4   ------> Busy
 * PA5   ------> DC/A0
 * PA6   ------> CSN
 * PA7   ------> SDA/MOSI
 * 
 * PA2   ------> TX
 * PA3   ------> RX
 */
#include "main.h"
#include "py32f0xx_hal_spi.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"
#include "EPD_Test.h"


SPI_HandleTypeDef spi1Handle;

static void APP_GPIO_Config(void);
static void APP_SPI_Config(void);

int main(void)
{
  HAL_Init();
  BSP_HSI_24MHzClockConfig();
  BSP_USART_Config();
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  APP_GPIO_Config();
  APP_SPI_Config();

  EPD_test();

  while (1);
}

void SPI_TxRxByte(uint8_t data)
{
  HAL_SPI_Transmit(&spi1Handle, &data, 1, HAL_MAX_DELAY);
}

static void APP_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  // PA0 RESET
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA5 DC/A0
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA6 CS
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA4 Busy (input) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void APP_SPI_Config(void)
{
  spi1Handle.Instance               = SPI1;
  spi1Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  spi1Handle.Init.Direction         = SPI_DIRECTION_1LINE;
  spi1Handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  spi1Handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  spi1Handle.Init.DataSize          = SPI_DATASIZE_8BIT;
  spi1Handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  spi1Handle.Init.NSS               = SPI_NSS_SOFT;
  spi1Handle.Init.Mode              = SPI_MODE_MASTER;
  if (HAL_SPI_DeInit(&spi1Handle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_SPI_Init(&spi1Handle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
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
