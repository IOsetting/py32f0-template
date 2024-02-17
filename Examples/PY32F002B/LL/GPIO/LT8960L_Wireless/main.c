/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "main.h"
#include "py32f002b_bsp_printf.h"
#include "py32f002b_bsp_clock.h"
#include "LT8960Ldrv.h"

/* 0:RX 1:TX 2:Carrier 3:Sleep & Wakeup */
#define WORK_MODE		        0

uint8_t pBuf[64];

int main(void)
{
  unsigned int i;
  uint8_t len,checksum;

  BSP_RCC_HSI_24MConfig();

  BSP_USART_Config(115200);
  printf("PY32F0xx LT8960L Example\r\nClock: %ld\r\n", SystemCoreClock);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  // LED PA7
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
  // CLK PA5
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  // DAT PA6
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);

  LT8960L_INIT();

#if WORK_MODE == 0

  printf("RX MODE\r\n");
  while(1)
  {
    len = LT8960L_RxData(LT8960L_RF_CHANNEL, pBuf, LT8960L_PACKET_SIZE);
    if (len)
    {
      checksum = 0;
      for (i = 0; i < (len - 1); i++)
      {
        printf("%02X ",pBuf[i]);
        checksum ^= pBuf[i];
      }

      if (checksum == pBuf[len - 1])
      {
        printf(" ok\r\n");
      }
    }
    LL_mDelay(0);
  }

#elif WORK_MODE == 1

  printf("Tx MODE");
  pBuf[0]  = 0xF1;
  pBuf[1]  = 0xF2;
  pBuf[2]  = 0xF3;
  pBuf[3]  = 0xF4;
  pBuf[4]  = 0xF5;

  pBuf[5]  = 0x00;
  pBuf[6]  = 0xFF;
  pBuf[7]  = 0x00;

  pBuf[8]  = 0x01;
  pBuf[9]  = 0x02;
  pBuf[10] = 0x03;
  pBuf[11] = 0x04;
  pBuf[12] = 0x05;
  pBuf[13] = 0x06;
  pBuf[14] = 0x07;

  while(1)
  {
    pBuf[5]++;
    pBuf[6]--;
    pBuf[7] += 2;

    pBuf[LT8960L_PACKET_SIZE - 1] = 0;
    for (i = 0; i < (LT8960L_PACKET_SIZE - 1); i++)
    {
      pBuf[LT8960L_PACKET_SIZE - 1] ^= pBuf[i];
    }

    LT8960L_TxData(LT8960L_RF_CHANNEL, pBuf, LT8960L_PACKET_SIZE);
    printf("TX: ");
    for (i = 0; i < LT8960L_PACKET_SIZE; i++)
    {
      printf("%02X ", pBuf[i]);
    }
    printf("\r\n");
    LL_mDelay(500);
  }

#elif WORK_MODE == 2

  printf("CarrierWave MODE");
  LT8960L_Carrier_Wave(LT8960L_RF_CHANNEL);
  while (1)
  {
    LL_mDelay(200);
  }

#elif WORK_MODE == 3

  printf("Sleep-Wakeup MODE");

  pBuf[0] = 0xa1;
  pBuf[1] = 0xa2;
  pBuf[2] = 0xa3;
  pBuf[3] = 0xa4;

  pBuf[4] = 0x30;
  pBuf[5] += 5;
  pBuf[6]--;
  pBuf[7] += 3;
  pBuf[8] = 0x1;
  pBuf[9] = 0x2;
  pBuf[10] = 0x3;
  pBuf[11] = 0x4;
  pBuf[12] = 0x5;
  pBuf[13] = 0x6;
  pBuf[14] = 0x7;
  while (1)
  {
    LT8960L_Sleep();
    LL_mDelay(3000 - 300);
    LT8960L_Wakeup();

    pBuf[5] += 5;
    pBuf[6]--;
    pBuf[7] += 3;
    pBuf[LT8960L_PACKET_SIZE - 1] = 0;
    for (i = 0; i < (LT8960L_PACKET_SIZE - 1); i++)
    {
      pBuf[LT8960L_PACKET_SIZE - 1] ^= pBuf[i];
    }

    LT8960L_TxData(LT8960L_RF_CHANNEL, pBuf, LT8960L_PACKET_SIZE);
    LL_mDelay(100);
  }

#endif

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
