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
#define APP_MODE 0

const uint8_t dummy_addr[5] = {0xCC, 0x35, 0x12, 0x23, 0x67};
uint8_t buff[32] = {
  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};
uint32_t app_tick = 0, last_sent = 0;

static void APP_GPIO_Init(void);

int main(void)
{
  uint8_t i = 0, j = 0, irqflag;
  uint8_t rxbuf[32];

  BSP_RCC_HSI_48MConfig();
  BSP_USART_Config(115200);
  printf("PAN211 demo\r\n");

  APP_GPIO_Init();

  while (PAN211_Init(78) != 1) /* PAN211 init */
  {
    printf("PAN211 init failed.\r\n");
    LL_mDelay(500);
  }
  printf("PAN211 init succeeded.\r\n");
  
  PAN211_SetTxPower(-16); // rf power: -16 dBm
  PAN211_SetEnhancedMode(1); // switch to enhanced mode
  PAN211_ClearIRQFlags(0xFF); // clear all irq flags

  LL_SYSTICK_EnableIT();

#if APP_MODE == 0
  while (1)
  {
    if (app_tick - last_sent > 1000)
    {
      buff[0] = j;
      buff[31] = j++;
      PAN211_WriteFIFO(buff, sizeof(buff));
      PAN211_TxStart();
      last_sent = app_tick;
    }
    else if (app_tick < last_sent)
    {
      last_sent = 0;
    }

    if(IRQDetected())
    {
      irqflag = PAN211_GetIRQFlags();
      if (irqflag & RF_IT_TX_IRQ) /* tx flag */
      {
          PAN211_ClearIRQFlags(RF_IT_TX_IRQ);
          printf(">> RF_IT_TX_IRQ[0x%02X], TxLen = %d\r\n", irqflag, sizeof(buff));
          irqflag &= ~RF_IT_TX_IRQ;
      }
      if (irqflag & RF_IT_RX_IRQ) /* rx flag */
      {
          uint8_t RxLen;
          RxLen = PAN211_GetRecvLen();
          if (RxLen > 0)
          {
              PAN211_ReadFIFO(rxbuf, RxLen);
          }
          PAN211_ClearIRQFlags(RF_IT_RX_IRQ);
          printf(">> RF_IT_RX_IRQ[0x%02X], RxLen = %d: ", irqflag, RxLen);
          for (i = 0; i < RxLen; i++)
          {
            printf("%02X", *(rxbuf + i));
          }
          printf("\r\n");
          irqflag &= ~RF_IT_RX_IRQ;
      }
      if (irqflag & RF_IT_MAX_RT_IRQ) /* max retry flag */
      {
          PAN211_ClearIRQFlags(RF_IT_MAX_RT_IRQ);
          printf("RF_IT_MAX_RT_IRQ[0x%02X].\r\n", irqflag);
          irqflag &= ~RF_IT_MAX_RT_IRQ;
      }
      if (irqflag != 0)
      {
          PAN211_ClearIRQFlags(irqflag);
          printf("OTHER_IRQ[0x%02X].\r\n", irqflag);
      }
    }
  }

#else

  PAN211_WriteReg(PAN211_P0_TX_PLLEN_CFG, 0x02); /* [7:0]TxLen */
  PAN211_SetTxAddr(dummy_addr, 5);
  PAN211_RxStart();           /* rx mode */

  while (1)
  {
    while (!IRQDetected()); /* wait till sending finishes */

    irqflag = PAN211_GetIRQFlags();
    if (irqflag & RF_IT_RX_IRQ)     /* rx flag */
    {
      uint8_t RxLen;

      __disable_irq();
      RxLen = PAN211_GetRecvLen();
      // send ack if NoAck = 0 (PAN211_P0_WMODE_CFG0 [1] = 0)
      buff[0] = j++;
      PAN211_WriteFIFO(buff, 1); // write ack data
      __enable_irq();

      PAN211_ReadFIFO(rxbuf, RxLen);
      PAN211_ClearIRQFlags(RF_IT_RX_IRQ);
      printf(">> RF_IT_RX_IRQ[%02X] RxLen[%d] ", irqflag, RxLen);
      for (i = 0; i < sizeof(rxbuf); i++)
      {
        printf("%02X", *(rxbuf + i));
      }
      printf("\r\n");
    }
    if (irqflag & RF_IT_TX_IRQ)   /* tx flag */
    {
      PAN211_ClearIRQFlags(RF_IT_TX_IRQ);
      printf(">> RF_IT_TX_IRQ[%02X]\r\n", irqflag);
    }
    if (irqflag & RF_IT_PID_ERR_IRQ)    /* pid error flag */
    {
      PAN211_ClearIRQFlags(RF_IT_PID_ERR_IRQ);
      printf(">> RF_IT_PID_ERR_IRQ[%02X].\r\n", irqflag);
    }
    if (irqflag & RF_IT_CRC_ERR_IRQ)        /* crc error flag */
    {
      PAN211_ClearIRQFlags(RF_IT_CRC_ERR_IRQ);
      printf(">> RF_IT_CRC_ERR_IRQ[%02x]\r\n", irqflag);
    }
    if (irqflag & RF_IT_RX_TIMEOUT_IRQ)    /* rx timeout flag */
    {
      PAN211_ClearIRQFlags(RF_IT_RX_TIMEOUT_IRQ);
      printf(">> RF_IT_RX_TIMEOUT_IRQ[%02X].\r\n", irqflag);
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
