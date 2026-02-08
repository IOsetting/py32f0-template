/***
 * Demo: nRF24L01
 * 
 * PY32          nRF24L01
 * PA0   ------> MOSI
 * PA1   ------> MISO
 * PB0   ------> CLK/SCK
 * PB3   ------> IRQ
 * PB4   ------> RESET
 * PB5   ------> CSN
 * PA2   ------> CAD
 * 
 *               CH340/CP2102
 * PA3   ------> RX
 */

#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"
#include "pan_rf.h"
#include <string.h>

/* Mode options: MODE_TX, MODE_RX */
#define MODE_TX         0
#define MODE_RX         1

#define PAN3029_MODE    MODE_TX

static void APP_GPIO_Init(void);
static void APP_SPI_Init(void);
uint8_t SPI_TxRxByte(uint8_t data);

int main(void)
{
  RF_Err_t ret;
#if PAN3029_MODE == MODE_RX
  uint32_t g_RxCount = 0;
#else
  unsigned int g_TxCount = 0;
  unsigned char g_TxBuf[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
#endif
  BSP_RCC_HSI_48MConfig();
  LL_mDelay(2000); // avoid being brick
  BSP_USART_Config(115200);
  printf("SPI Demo: PAN3029 Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIO_Init();
  APP_SPI_Init();

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
 * PB3   ------> IRQ
 * PB4   ------> RESET
 * PB5   ------> CSN
 * PA2   ------> CAD
 */
static void APP_GPIO_Init(void)
{
  /* Enable GPIOA,GPIOB,GPIOC clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB | LL_IOP_GRP1_PERIPH_GPIOC);

  // PB4:RESET, PB5:CSN GPIO output
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);

  // PB3:IRQ, PA2:CAD, GPIO input
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
}

/**
 * PA0   ------> MOSI
 * PA1   ------> MISO
 * PB0   ------> CLK/SCK
 */
static void APP_SPI_Init(void)
{
  LL_SPI_InitTypeDef initStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  // PB0 SCK, AF0
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PA0 MOSI, PA1 MISO, AF0
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  initStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  initStruct.Mode = LL_SPI_MODE_MASTER;
  initStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  initStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  initStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  initStruct.NSS = LL_SPI_NSS_SOFT;
  initStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  initStruct.BitOrder = LL_SPI_MSB_FIRST;

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
  LL_SPI_Init(SPI1, &initStruct);
  LL_SPI_Enable(SPI1);
}

void APP_ErrorHandler(void)
{
  while (1);
}
