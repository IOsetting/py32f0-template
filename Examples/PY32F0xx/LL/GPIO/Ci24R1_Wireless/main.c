/***
 * Demo: Ci24R1 2.4GHz Wireless
 * 
 * PY32          Ci24R1 SOP8
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
#include "ci24r1.h"

// 0:RX, 1:TX
#define XL2400_MODE 0

const uint8_t TX_ADDRESS[5] = {0x11,0x33,0x33,0x33,0x11};
const uint8_t RX_ADDRESS[5] = {0xAA,0x11,0xBB,0x22,0xCC};
uint8_t tmp[] = {
    0x1F, 0x80, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
    0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
    0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x37, 0x48};

static void APP_GPIOConfig(void);

extern uint8_t xbuf[CI24R1_PLOAD_MAX_WIDTH + 1];

int main(void)
{
  uint8_t i = 0, j = 0, status;
  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  APP_GPIOConfig();

  BSP_USART_Config(115200);
  printf("GPIO Demo: Ci24R1 Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  while (CI24R1_SPI_Test() == ERROR)
  {
    printf(" - check failed\r\n");
    LL_mDelay(1000);
  }
  printf(" - check passed\r\n");

  CI24R1_Init();

#if XL2400_MODE == 0
  // RX
  CI24R1_SetChannel(60);
  CI24R1_SetRxMode();
  CI24R1_SetTxAddress(RX_ADDRESS);
  CI24R1_SetRxAddress(TX_ADDRESS);
  printf("CI24R1 RX Initialized\r\n");

  while(1)
  {
      status = CI24R1_Rx();
      printf("%02X %02X%02X...%02X\r\n", status, *xbuf, *(xbuf + 1), *(xbuf + CI24R1_PLOAD_WIDTH - 1));
  }
#else
  // TX
  CI24R1_SetChannel(60);
  CI24R1_SetTxMode();
  CI24R1_SetTxAddress(TX_ADDRESS);
  CI24R1_SetRxAddress(RX_ADDRESS);
  printf("CI24R1 TX Initialized\r\n");

  while (1)
  {
    status = CI24R1_Tx2(tmp, CI24R1_PLOAD_WIDTH);

    i++;
    if (status & CI24R1_FLAG_TX_SENT)
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
