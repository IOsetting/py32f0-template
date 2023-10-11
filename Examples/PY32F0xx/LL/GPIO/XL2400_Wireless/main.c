/***
 * Demo: XL2400 2.4GHz Wireless
 * 
 * PY32          XL2400 SOP8
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
#include "xl2400.h"

// 0:TX, 1:RX
#define XL2400_MODE 1

const uint8_t TX_ADDRESS[5] = {0x11,0x33,0x33,0x33,0x11};
const uint8_t RX_ADDRESS[5] = {0x33,0x55,0x33,0x44,0x33};
uint8_t tmp[] = {
    0x1F, 0x80, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
    0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
    0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x37, 0x48};

static void APP_GPIOConfig(void);

extern uint8_t xbuf[XL2400_PL_WIDTH_MAX + 1];

int main(void)
{
  uint8_t i = 0, j = 0, status;

  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  APP_GPIOConfig();

  BSP_USART_Config(115200);
  printf("GPIO Demo: XL2400 Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  while (XL2400_SPI_Test() == ERROR)
  {
    printf(" - check failed\r\n");
    LL_mDelay(1000);
  }
  printf(" - check passed\r\n");

  XL2400_Init();
  XL2400_SetPower(XL2400_RF_0DB);

#if XL2400_MODE == 0
  XL2400_SetChannel(78);
  XL2400_SetTxAddress(RX_ADDRESS);
  XL2400_SetRxAddress(TX_ADDRESS);
  XL2400_SetTxMode();
  printf("XL2400 TX Initialized\r\n");

  while (1)
  {
    // XL2400_PrintStatus();
    status = XL2400_Tx(tmp, XL2400_PLOAD_WIDTH);

    i++;
    if (status == 0x20)
    {
      j++;
    }
    if (i == 0xFF)
    {
      printf("%02X\r\n", j);
      i = 0;
      j = 0;
    }
    // >= 2ms
    LL_mDelay(3);
  }
#else
  // RX
  XL2400_SetChannel(77);
  XL2400_SetTxAddress(RX_ADDRESS);
  XL2400_SetRxAddress(TX_ADDRESS);
  XL2400_WakeUp();
  printf("XL2400 RX Initialized\r\n");

  while (1)
  {
    XL2400_SetRxMode();
    while (--j)
    {
      status = XL2400_Rx();
      if (status & RX_DR_FLAG)
      {
        printf(".");
        for (i = 0; i < 32; i++)
        {
          printf("%02X", *(xbuf + i));
        }
      }
      //LL_mDelay(1);
    }
    printf("\r\n");
    //XL2400_PrintStatus();
    //XL2400_Sleep();
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
