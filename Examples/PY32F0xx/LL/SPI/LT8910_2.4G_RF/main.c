/***
 * Demo: LT8910/LT8920 2.4GHz RF
 * 
 * PY32                LT8910/8920
 * PA0   ------------> MISO
 * PA7   ------------> MOSI
 * PA1   ------------> CLK/SCK
 * PA6   ------------> SS
 * PA4   ------------> RESET
 * PB5   ------------> PKT
 * 
 * PA2(TX)   ------> RX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "lt8910.h"

// 0:RX, 1:TX
#define APP_MODE 0

uint8_t pBuf[128];


static void APP_GPIO_Config(void);
static void APP_SPI_Config(void);

int main(void)
{
  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  BSP_USART_Config(115200);
  printf("SPI Demo: LT8910 Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIO_Config();
  APP_SPI_Config();

  LT8910_Init();
  LT8910_SetChannel(76);
  LT8910_WhatsUp();

#if APP_MODE == 0
  uint8_t rx_len, i, j;
  LT8910_SetRx();
  while(1)
  {
    for (j = 0; j < 255; j++)
    {
      LL_mDelay(0);
      rx_len = LT8910_Rx(pBuf);
      if (rx_len > 0)
      {
        printf("%d %02X\r\n", j, *(pBuf + rx_len - 1));
        // for (i = 0; i < rx_len; i++)
        // {
        //   printf("%02X ", *(pBuf + i));
        // }
        // printf("\r\n");
        break;
      }
    }
  }

#elif APP_MODE == 1
  uint8_t seq = 0;
  while (1)
  {
    pBuf[0] = 0x01;
    pBuf[1] = 0x02;
    pBuf[2] = 0x03;
    pBuf[3] = 0x04;
    pBuf[4] = 0x05;
    pBuf[5] = 0x06;
    pBuf[6] = 0x07;
    pBuf[7] = 0x08;
    pBuf[8] = 0x09;
    pBuf[9] = 0x0A;
    pBuf[10] = 0x0B;
    pBuf[11] = 0x0C;
    pBuf[12] = 0x0D;
    pBuf[13] = 0x0E;
    pBuf[14] = 0x0F;
    pBuf[15] = seq++;
    LT8910_Tx(pBuf, 16);
    //LT8910_WhatsUp();
    LL_mDelay(0);
  }
#endif
}

static void APP_GPIO_Config(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* PB5 PKT */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PA6 SS
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  // PA4 RST
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
}

/**
 * SPI1 Alternative Function Pins
 * SPI1_SCK:  PA1_AF0, PA2_AF10, PA5_AF0, PA9_AF10, PB3_AF0
 * SPI1_MISO: PA0_AF10, PA6_AF0, PA7_AF10, PA11_AF0, PA13_AF10, PB4_AF0
 * SPI1_MOSI: PA1_AF10, PA2_AF0, PA3_AF10, PA7_AF0, PA8_AF10, PA12_AF0, PB5_AF0
 * SPI1_NSS:  PA4_AF0, PA10_AF10, PA15_AF0, PB0_AF0, PF1_AF10, PF3_AF10
*/
static void APP_SPI_Config(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);

  // PA1 SCK
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // PA0 MISO
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // PA7 MOSI
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*
   * Full duplex mode, MOSI and MISO both connect to DATA,
   * Add one 1KR between MOSI and DATA
  */
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_Enable(SPI1);
}

uint8_t SPI_TxRxByte(uint8_t data)
{
  uint8_t SPITimeout = 0xFF;
  /* Check the status of Transmit buffer Empty flag */
  while (READ_BIT(SPI1->SR, SPI_SR_TXE) == RESET)
  {
    if (SPITimeout-- == 0)
      return 0;
  }
  LL_SPI_TransmitData8(SPI1, data);
  SPITimeout = 0xFF;
  while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == RESET)
  {
    if (SPITimeout-- == 0)
      return 0;
  }
  // Read from RX buffer
  return LL_SPI_ReceiveData8(SPI1);
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
