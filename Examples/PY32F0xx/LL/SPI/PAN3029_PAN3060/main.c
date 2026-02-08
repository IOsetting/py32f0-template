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
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"
#include "pan_rf.h"

/* Mode options: MODE_TX, MODE_RX, MODE_RX_INT */
#define MODE_TX         0
#define MODE_RX         1

#define PAN3029_MODE      MODE_RX

static void APP_GPIOConfig(void);
static void APP_SPIConfig(void);


int main(void)
{
  RF_Err_t ret;
#if PAN3029_MODE == MODE_RX
  uint32_t g_RxCount = 0;
#else
  unsigned int g_TxCount = 0;
  unsigned char g_TxBuf[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
#endif
  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  BSP_USART_Config(115200);
  printf("SPI Demo: PAN3029 Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIOConfig();
  APP_SPIConfig();

  ret = RF_Init();
  if (ret != RF_OK)
  {
    printf("RF init fail\r\n");
    while (1);
  }
#if PAN3029_MODE == MODE_RX
  printf("Rx test start\r\n");
  RF_ConfigUserParams();      /* Configure parameters like Frequency, SF, BW, Preamble, CRC, etc. */
  RF_EnterContinousRxState(); /* Enter continuous reception state */

  while (1)
  {
    if (CHECK_RF_IRQ()) /* Detect RF interrupt, high level indicates an interrupt */
    {
      uint8_t IRQFlag;
      IRQFlag = RF_GetIRQFlag();    /* Get interrupt flags */
      if (IRQFlag & RF_IRQ_RX_DONE) /* flag = Receive complete */
      {
        g_RfRxPkt.Snr = RF_GetPktSnr();   /* Get the SNR value of the received packet */
        g_RfRxPkt.Rssi = RF_GetPktRssi(); /* Get the RSSI value of the received packet */
        g_RfRxPkt.RxLen = RF_GetRecvPayload((uint8_t *)g_RfRxPkt.RxBuf); /* Get received data and length */
        printf("SNR:%ddB, RSSI:%ddBm Len=%d, Count=%ld Data:\r\n", 
          (int)g_RfRxPkt.Snr, (int)g_RfRxPkt.Rssi, g_RfRxPkt.RxLen, ++g_RxCount);
        for (int i = 0; i < (int)g_RfRxPkt.RxLen; i++)
        {
          printf("%02X ", ((uint8_t *)g_RfRxPkt.RxBuf)[i]);
        }
        printf("\r\n");
        RF_ClrIRQFlag(RF_IRQ_RX_DONE); /* Clear interrupt flag */
        IRQFlag &= ~RF_IRQ_RX_DONE;
      }
      if (IRQFlag & RF_IRQ_CRC_ERR) /* CRC error */
      {
        RF_ClrIRQFlag(RF_IRQ_CRC_ERR);
        IRQFlag &= ~RF_IRQ_CRC_ERR;
        printf(">>RF_IRQ_CRC_ERR\r\n");
      }
      if (IRQFlag & RF_IRQ_RX_TIMEOUT) /* Receive timeout */
      {
        RF_ClrIRQFlag(RF_IRQ_RX_TIMEOUT);
        IRQFlag &= ~RF_IRQ_RX_TIMEOUT;
        printf(">>RF_IRQ_RX_TIMEOUT\r\n");
      }
      if (IRQFlag)
      {
        RF_ClrIRQFlag(IRQFlag); /* Clear unhandled interrupt flags */
      }
    }
  }
#else
  printf("Tx test start\r\n");
  RF_ConfigUserParams(); /* Configure parameters like Frequency, SF, BW, Preamble, CRC, etc. */
  while (1)
  {
    RF_TxSinglePkt(g_TxBuf, sizeof(g_TxBuf)); /* Send data packet */
    while (1)
    {
      if (CHECK_RF_IRQ()) /* Detect RF interrupt, high level indicates an interrupt */
      {
        uint8_t IRQFlag;
        IRQFlag = RF_GetIRQFlag();      /* Get interrupt flags */
        if (IRQFlag & RF_IRQ_TX_DONE)   /* Transmit complete */
        {
          RF_TurnoffPA();                /* Must turn off PA after transmission is complete */
          RF_ClrIRQFlag(RF_IRQ_TX_DONE); /* Clear transmit complete interrupt flag */
          IRQFlag &= ~RF_IRQ_TX_DONE;
          RF_EnterStandbyState();       /* Must set to RF_STATE_STB3 state after transmission is complete */
          printf("Tx done, tx count:%d\r\n", ++g_TxCount);
          break;
        }
        if (IRQFlag)
        {
          RF_ClrIRQFlag(IRQFlag); /* Clear unhandled interrupt flags */
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
