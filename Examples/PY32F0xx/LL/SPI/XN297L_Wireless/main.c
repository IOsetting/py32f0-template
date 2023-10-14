/***
 * Demo: XN297LBW 2.4GHz Wireless
 *   XN297L datasheet and SDK can be downloaded from 
 *   https://wiki.panchip.com/ble-lite/2-4g-t-rx/xn297l_series/
 * 
 * PY32                XN297LBW SOP8
 * PA0   ------------> DATA/MOSI
 * PA7   ---> 1KR ---> DATA/MOSI
 * PA1   ------------> CLK/SCK
 * PA6   ------------> CSN/NSS
 * 
 *                     USB2TTL
 * PA2(TX) ----------> RX
 * PA3(RX) ----------> TX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "xn297l.h"

// 0:RX, 1:TX, 2:TX_FAST
#define XN297L_MODE 0

const uint8_t TX_ADDRESS[5] = {0x11,0x33,0x33,0x33,0x11};
const uint8_t RX_ADDRESS[5] = {0x33,0x55,0x33,0x44,0x33};
uint8_t tmp[] = {
    0x1F, 0x80, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
    0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
    0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x37, 0x48};

static void APP_GPIO_Config(void);
static void APP_SPI_Config(void);

extern uint8_t xbuf[XN297L_PLOAD_WIDTH + 1];

int main(void)
{
  uint8_t i = 0, j = 0, status;

  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  APP_GPIO_Config();
  APP_SPI_Config();

  BSP_USART_Config(115200);
  printf("SPI Demo: XN297L Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  while (XN297L_SPI_Test() == ERROR)
  {
    printf(" - check failed\r\n");
    LL_mDelay(1000);
  }
  printf(" - check passed\r\n");

  XN297L_Init();

#if XN297L_MODE == 0
  // RX
  XN297L_SetTxAddress(RX_ADDRESS);
  XN297L_SetRxAddress(TX_ADDRESS);
  XN297L_SetRxMode();
  printf("XN297L RX Initialized\r\n");

  while (1)
  {
    while (--j)
    {
      status = XN297L_DumpRxData();
      if (status & XN297L_FLAG_RX_DR)
      {
        // for (i = 0; i < 32; i++)
        // {
        //   printf("%02X", *(xbuf + i));
        // }
        printf("%02X %02X %02X\r\n", *(xbuf + 0), *(xbuf + 1), *(xbuf + 31));
      }
      else
      {
        printf(".");
      }
      //XN297L_PrintStatus();
      //LL_mDelay(500);
    }
    printf("\r\n");
  }
  
#elif XN297L_MODE == 1
  // TX
  XN297L_SetTxAddress(RX_ADDRESS);
  XN297L_SetRxAddress(TX_ADDRESS);
  XN297L_SetTxMode();
  printf("XN297L TX Initialized\r\n");

  while (1)
  {
    // XN297L_PrintStatus();
    status = XN297L_TxData(tmp, XN297L_PLOAD_WIDTH);
    i++;
    if (status & XN297L_FLAG_TX_DS)
    {
      j++;
    }
    if (i == 0xFF)
    {
      printf("%02X\r\n", j);
      i = 0;
      j = 0;
    }
  }
#else
  // TX fast
  XN297L_SetTxAddress(RX_ADDRESS);
  XN297L_SetRxAddress(TX_ADDRESS);
  XN297L_SetTxMode();
  printf("XN297L TX Initialized\r\n");

  while(1)
  {
    i++;
    if (XN297L_TxFast(tmp, XN297L_PLOAD_WIDTH) == SUCCESS)
    {
      j++;
    }
    else
    {
      XN297L_ReuseTX();
    }

    if (i == 0xFF)
    {
      // Indicate success rate on 255 times of tx
      printf("%02X\r\n", j);
      i = 0;
      j = 0;
    }
  }
#endif
}

static void APP_GPIO_Config(void)
{
  // PA6 CSN
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
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
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT; 
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
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
