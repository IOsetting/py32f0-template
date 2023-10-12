/***
 * Demo: XN297LBW 2.4GHz Wireless
 *   XN297L datasheet and SDK can be downloaded from 
 *   https://wiki.panchip.com/ble-lite/2-4g-t-rx/xn297l_series/
 * 
 * PY32          XN297LBW SOP8
 * PA1   ------> CLK/SCK
 * PA6   ------> CSN/NSS
 * PA7   ------> DATA/MOSI
 * 
 *               USB2TTL
 * PA2(TX) ----> RX
 * PA3(RX) ----> TX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "xn297l.h"

// 0:RX, 1:TX
#define XN297L_MODE 0

const uint8_t TX_ADDRESS[5] = {0x11,0x33,0x33,0x33,0x11};
const uint8_t RX_ADDRESS[5] = {0x33,0x55,0x33,0x44,0x33};
uint8_t tmp[] = {
    0x1F, 0x80, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
    0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
    0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x37, 0x48};

static void APP_GPIOConfig(void);

extern uint8_t xbuf[XN297L_PLOAD_WIDTH + 1];

int main(void)
{
  uint8_t i = 0, j = 0, status;

  /* Set clock = 48MHz */
  BSP_RCC_HSI_24MConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  APP_GPIOConfig();

  BSP_USART_Config(115200);
  printf("GPIO Demo: XN297L Wireless\r\nClock: %ld\r\n", SystemCoreClock);

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
        // printf(".");
        // for (i = 0; i < 32; i++)
        // {
        //   printf("%02X", *(xbuf + i));
        // }
        printf("%02X %02X %02X\r\n", *(xbuf + 0), *(xbuf + 1), *(xbuf + 31));
      }
      //XN297L_PrintStatus();
      //LL_mDelay(500);
    }
    printf("\r\n");
  }
  
#else
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
#endif
}

static void APP_GPIOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* PA1 CLK */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA6 CSN */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA7 DATA */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
