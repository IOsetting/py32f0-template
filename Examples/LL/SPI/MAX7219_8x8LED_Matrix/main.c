/***
 * Demo: SPI / MAX7219 8x8 LED Martix
 * 
 * PY32          MAX7219
 * PB2   ------> CS
 * PB3   ------> CLK/SCK
 * PB5   ------> DIN/MOSI
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"


#define DECODE_MODE  0x09
#define INTENSITY    0x0A
#define SCAN_LIMIT   0x0B
#define SHUT_DOWN    0x0C
#define DISPLAY_TEST 0x0F

const uint8_t numbers[] = {
    0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xCE, 0xD6, 0xD6, // -0-.
    0xE6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x38, 0x78, 0x18, 0x18, 0x18, // -1-
    0x18, 0x18, 0x18, 0x7E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7C, 0xC6, 0x06, 0x0C, 0x18, 0x30, // -2-
    0x60, 0xC0, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7C, 0xC6, 0x06, 0x06, 0x3C, 0x06, // -3-
    0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0C, 0x1C, 0x3C, 0x6C, 0xCC, 0xFE, // -4-
    0x0C, 0x0C, 0x0C, 0x1E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFE, 0xC0, 0xC0, 0xC0, 0xFC, 0x0E, // -5-
    0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x38, 0x60, 0xC0, 0xC0, 0xFC, 0xC6, // -6-
    0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFE, 0xC6, 0x06, 0x06, 0x0C, 0x18, // -7-
    0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7C, 0xC6, // -8-
    0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, // -9-
    0x06, 0x06, 0x0C, 0x78, 0x00, 0x00, 0x00, 0x00};


static void APP_SPIConfig(void);
static void APP_SystemClockConfig(void);
uint8_t SPI_TxRxByte(uint8_t data);

void MAX7219_write(uint8_t addr, uint8_t dat)
{
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
  SPI_TxRxByte(addr);
  SPI_TxRxByte(dat);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);
}

void MAX7219_init(void)
{
    MAX7219_write(SHUT_DOWN, 0x01);    // 0x00:shutdown, 0x01:normal
    MAX7219_write(DECODE_MODE, 0x00);  // Bypass code B decoder, no-decode operation
    MAX7219_write(SCAN_LIMIT, 0x07);   // Scan-limit, 0:1-digit, 1:2-digits, ... 7:8-digits
    MAX7219_write(INTENSITY, 0x01);    // 0x00:min, 0xFF:max
    MAX7219_write(DISPLAY_TEST, 0x00); // 0x00:normal, 0x01:test mode
}


int main(void)
{
  uint8_t pos = 0, size = sizeof(numbers), i, j;

  APP_SystemClockConfig();

  BSP_USART_Config(115200);
  printf("SPI Demo: MAX7219 8x8 LED\r\nClock: %ld\r\n", SystemCoreClock);

  APP_SPIConfig();

  MAX7219_init();

  while (1)
  {
    for (i = 0; i < 8; i++)
    {
        j = (pos + i) % size;
        MAX7219_write(i + 1, numbers[j]);
    }
    pos = (pos + 1) % size;
    LL_mDelay(200);
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
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  // PB2 CS
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
  // PB3 SCK
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // PB4 MISO (not connected)
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // PB5 MOSI
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_Enable(SPI1);
}

static void APP_SystemClockConfig(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSI_Enable();
  /* Change this value to adjust clock frequency, larger is faster */
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz + 15);
  while (LL_RCC_HSI_IsReady() != 1);

  UTILS_ClkInitStruct.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
  UTILS_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_1;
  LL_PLL_ConfigSystemClock_HSI(&UTILS_ClkInitStruct);

  /* Re-init frequency of SysTick source, reload = freq/ticks = 48000000/1000 = 48000 */
  LL_InitTick(48000000, 1000U);
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
