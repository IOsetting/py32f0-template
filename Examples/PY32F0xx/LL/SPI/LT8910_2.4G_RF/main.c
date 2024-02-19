/***
 * Demo: LT8910/LT8920 2.4GHz RF
 * 
 * PY32               LT8910/8920
 * PA0       ------>    MISO
 * PA1       ------>    CLK/SCK
 * PA4       ------>    RESET
 * PA5       ------>    PKT
 * PA6       ------>    SS
 * PA7       ------>    MOSI
 * 
 *                    USB2TTL(for printf output)
 * PA2(TX)   ------>    RX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "lt8910.h"

// 0:RX, 1:TX
#define APP_MODE 0

uint8_t pBuf[128] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
  0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
  0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
  0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F
};


static void APP_GPIO_Config(void);
static void APP_SPI_Config(void);

int main(void)
{
  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();
  BSP_USART_Config(115200);
  printf("SPI Demo: LT8910/LT8920 2.4GHz Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIO_Config();
  APP_SPI_Config();

  LT8910_Init();
  LT8910_SetChannel(76);
  LT8910_WhatsUp();

#if APP_MODE == 0
  uint8_t rx_len, cycle, prev = 0;
  uint32_t recv = 0, miss = 0, size = 0;
  LT8910_SetRx();
  while(1)
  {
    for (cycle = 0; cycle < 255; cycle++)
    {
      LL_mDelay(0);
      rx_len = LT8910_Rx(pBuf);
      if (rx_len > 0)
      {
        /* printf will significantly slow down the rx speed */

        // printf("%d %02X\r\n", cycle, *(pBuf + rx_len - 1));
        // for (uint8_t i = 0; i < rx_len; i++)
        // {
        //   printf("%02X ", *(pBuf + i));
        // }
        // printf("\r\n");

        recv++;
        size += rx_len;
        /* The last byte increases by 1 for each packets, use this byte to
         * calculate the missing packets
         */
        if (*(pBuf + rx_len - 1) > 0)
        {
          miss += *(pBuf + rx_len - 1) - prev - 1;
        }
        else
        {
          miss += 256 - prev - 1;
        }
        prev = *(pBuf + rx_len - 1);
        break;
      }
    }
    if (recv == 1000)
    {
      printf("Total %ld bytes, missing packets %ld/%ld\r\n", size, miss, recv);
      recv = 0;
      size = 0;
      miss = 0;
    }
  }

#elif APP_MODE == 1

  uint8_t seq = 0;
  while (1)
  {
    pBuf[62] = seq++;
    LT8910_Tx(pBuf, 63);
    //LT8910_WhatsUp();
    if (seq == 0)
    {
      printf("tx 256 packets\r\n");
    }
    LL_mDelay(1);
  }
#endif
}

static void APP_GPIO_Config(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  // PA4 RST, GPIO output
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
  // PA5 PKT, GPIO input
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // PA6 SS, GPIO output
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);

  /* SPI related pins */
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

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);

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
