/***
 * Demo: SPI / WS2812(B) color LED
 * 
 * PY32          WS2812(B)
 * PA7   ------> DIN
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "ws2812_spi.h"


static void APP_SPIConfig(void);

int main(void)
{
  uint8_t i = 0, r = 0, g = 0x60, b = 0xC0;

  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("SPI Demo: WS2812 LED\r\nClock: %ld\r\n", SystemCoreClock);

  APP_SPIConfig();

  ws2812_pixel_all(r, g, b);
  ws2812_send_spi();
  while (1)
  {
    i = (i + 1) % WS2812_NUM_LEDS;
    ws2812_pixel(i, r++, g++, b++);
    ws2812_send_spi();
    LL_mDelay(20);
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

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);

  // PA7 MOSI
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* The frequency after prescaler should be below 8.25MHz */
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
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
