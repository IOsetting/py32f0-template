/***
 * Demo: PAN3029/PAN3060
 * 
 * PY32          PAN3029/PAN3060
 * PA0   ------> MISO 
 * PA1   ------> CLK/SCK
 * PA7   ------> MOSI
 * PA6   ------> CSN
 * PA4   ------> IRQ
 * PA5   ------> RESET
 * PB0   ------> CAD
 *
 *               CH340/CP2102
 * PA2   ------> RX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "pan_rf.h"

/* Mode options: MODE_TX, MODE_RX, MODE_RX_INT */
#define MODE_TX         0
#define MODE_RX         1

#define PAN3029_MODE      MODE_TX

static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);
static void APP_SPIConfig(void);


int main(void)
{
  RF_Err_t ret;
#if PAN3029_MODE == MODE_RX
  uint32_t g_RxCount = 0;
#else
  unsigned int g_TxCount = 0;
  /* g_TxBuf为PAN3029/3060 发送数据缓冲区，大小16字节，内容为0~15 */
  unsigned char g_TxBuf[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
#endif
  /* Set clock = 48MHz */
  APP_SystemClockConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  BSP_USART_Config(115200);
  printf("SPI Demo: PAN3029 Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIOConfig();
  APP_SPIConfig();

  ret = RF_Init();        /* PAN3029/3060初始化 */
  if (ret != RF_OK)
  {
    printf("RF init fail\r\n");
    while (1);
  }
#if PAN3029_MODE == MODE_RX
  printf("RF rx test start.\r\n");
  RF_ConfigUserParams();      /* 配置 Frequency、SF、BW、Preamble、CRC等参数 */
  RF_EnterContinousRxState(); /* 进入连续接收状态 */

  while (1)
  {
    if (CHECK_RF_IRQ()) /* 检测到RF中断，高电平表示有中断 */
    {
      uint8_t IRQFlag;
      IRQFlag = RF_GetIRQFlag();    /* 获取中断标志位 */
      if (IRQFlag & RF_IRQ_RX_DONE) /* 接收完成中断 */
      {
        g_RfRxPkt.Snr = RF_GetPktSnr();   /* 获取接收数据包的SNR值 */
        g_RfRxPkt.Rssi = RF_GetPktRssi(); /* 获取接收数据包的RSSI值 */

        /* 获取接收数据和长度 */
        g_RfRxPkt.RxLen = RF_GetRecvPayload((uint8_t *)g_RfRxPkt.RxBuf);

        printf("+Rx Len=%d, Count=%ld ", g_RfRxPkt.RxLen, ++g_RxCount);
        printf("+RxHexData: ");
        for (int i = 0; i < (int)g_RfRxPkt.RxLen; i++)
        {
          printf("%02X ", ((uint8_t *)g_RfRxPkt.RxBuf)[i]);
        }
        printf("\r\n");
        printf("SNR:%ddB, RSSI:%ddBm \r\n", (int)g_RfRxPkt.Snr, (int)g_RfRxPkt.Rssi);
        RF_ClrIRQFlag(RF_IRQ_RX_DONE); /* 清除接收完成中断标志位 */
        IRQFlag &= ~RF_IRQ_RX_DONE;
      }
      if (IRQFlag & RF_IRQ_CRC_ERR) /* CRC错误中断 */
      {
        RF_ClrIRQFlag(RF_IRQ_CRC_ERR); /* 清除CRC错误中断标志位 */
        IRQFlag &= ~RF_IRQ_CRC_ERR;
        printf(">>RF_IRQ_CRC_ERR\r\n");
      }
      if (IRQFlag & RF_IRQ_RX_TIMEOUT) /* 接收超时中断 */
      {
        RF_ClrIRQFlag(RF_IRQ_RX_TIMEOUT); /* 清除接收超时中断标志位 */
        IRQFlag &= ~RF_IRQ_RX_TIMEOUT;
        printf(">>RF_IRQ_RX_TIMEOUT\r\n");
      }
      if (IRQFlag)
      {
        RF_ClrIRQFlag(IRQFlag); /* 清除未处理的中断标志位 */
      }
    }
  }
#else
  printf("RF tx test start.\r\n");
  RF_ConfigUserParams(); /* 配置 Frequency、SF、BW、Preamble、CRC等参数 */
  while (1)
  {
    //memset(g_TxBuf, g_TxCount, sizeof(g_TxBuf));
    RF_TxSinglePkt(g_TxBuf, sizeof(g_TxBuf)); /* 发送数据包 */
    while (1)
    {
      if (CHECK_RF_IRQ()) /* 检测到RF中断，高电平表示有中断 */
      {
        uint8_t IRQFlag;
        IRQFlag = RF_GetIRQFlag();      /* 获取中断标志位 */
        if (IRQFlag & RF_IRQ_TX_DONE)   /* 发送完成中断 */
        {
          RF_TurnoffPA();                /* 发送完成后须关闭PA */
          RF_ClrIRQFlag(RF_IRQ_TX_DONE); /* 清除发送完成中断标志位 */
          IRQFlag &= ~RF_IRQ_TX_DONE;
          RF_EnterStandbyState();       /* 发送完成后须设置为RF_STATE_STB3状态 */
          printf("Tx done, tx count:%d\r\n", ++g_TxCount); /* 打印发送次数 */
          break;
        }
        if (IRQFlag)
        {
          RF_ClrIRQFlag(IRQFlag); /* 清除未处理的中断标志位 */
        }
      }
    }
    LL_mDelay(2000);
  }
#endif
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

/*
 * PA6   ------> CSN
 * PA4   ------> IRQ
 * PA5   ------> RESET
 * PB0   ------> CAD
*/
static void APP_GPIOConfig(void)
{
  // PA6 CSN
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
  // PA5 RESET
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);
  // PA4 IRQ
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);
  // PB0 CAD
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
}

/**
 * PA0   ------> MISO 
 * PA1   ------> CLK/SCK
 * PA7   ------> MOSI
 * SPI1 Alternative Function Pins
 * SPI1_SCK:  PA1_AF0, PA2_AF10, PA5_AF0, PA9_AF10, PB3_AF0
 * SPI1_MISO: PA0_AF10, PA6_AF0, PA7_AF10, PA11_AF0, PA13_AF10, PB4_AF0
 * SPI1_MOSI: PA1_AF10, PA2_AF0, PA3_AF10, PA7_AF0, PA8_AF10, PA12_AF0, PB5_AF0
 * SPI1_NSS:  PA4_AF0, PA10_AF10, PA15_AF0, PB0_AF0, PF1_AF10, PF3_AF10
*/
static void APP_SPIConfig(void)
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

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
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
