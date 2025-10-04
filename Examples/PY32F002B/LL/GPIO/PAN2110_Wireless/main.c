/**
 * B5: CSN
 * B0: CSK
 * A1: DATA
 */

#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"
#include "pan211.h"
#include <string.h>

// 0:TX, 1:RX
#define APP_MODE 1

uint8_t buff[32] = {
  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};

static void APP_GPIO_Init(void);

int main(void)
{
  uint8_t i = 0, irqflag;

  BSP_RCC_HSI_48MConfig();
  BSP_USART_Config(115200);
  printf("PAN211 demo\r\n");

  APP_GPIO_Init();

  while (PAN211_Init(60) != 1) /* PAN211 init */
  {
    printf("PAN211 init failed.\r\n");
    LL_mDelay(500);
  }
  printf("PAN211 init succeeded.\r\n");

  PAN211_ClearIRQFlags(0xFF);

#if APP_MODE == 0
  while (1)
  {
    buff[31] = i++;
    PAN211_WriteFIFO(buff, sizeof(buff));
    PAN211_TxStart();

    while (!IRQDetected()); /* Optional, wait till SPI_DATA/IIC_SDA becomes low */

    while (1)
    {
      irqflag = PAN211_GetIRQFlags();
      if (irqflag & RF_IT_TX_IRQ)     /* tx finish flag */
      {
        PAN211_ClearIRQFlags(RF_IT_ALL_IRQ);  /* clear flag */
        printf("Tx: %02X\r\n", i);
        break;
      }
    }

    LL_mDelay(1500);
  }

#else

  PAN211_RxStart();           /* rx mode */

  while (1)
  {
    while (!IRQDetected()); /* wait till sending finishes */

    irqflag = PAN211_GetIRQFlags();
    if (irqflag & RF_IT_RX_IRQ)     /* rx flag */
    {
      PAN211_ReadFIFO(buff, sizeof(buff)); /*  read to buffer */
      PAN211_ClearIRQFlags(RF_IT_RX_IRQ);    /* clear irq flag */
      for (i = 0; i < sizeof(buff); i++)
      {
        printf("%02X", *(buff + i));
      }
      printf("\r\n");
    }
    else if (irqflag == RF_IT_CRC_ERR_IRQ) /* CRC error */
    {
      PAN211_ClearIRQFlags(irqflag); /* clear irq flag */
      printf(">> RF_IT_CRC_ERR_IRQ[0x%02x]\r\n", irqflag);
    }
  }

#endif
}

static void APP_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIOB clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  /* GPIO output - B0:SCK, B5:CSN */
  GPIO_InitStruct.Pin       = LL_GPIO_PIN_0 | LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode      = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull      = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* A1:DATA */
  GPIO_InitStruct.Pin       = LL_GPIO_PIN_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void APP_ErrorHandler(void)
{
  while (1);
}
