/***
 * Demo: nRF24L01
 * 
 * PY32          nRF24L01
 * 
 * PA0   ------> MOSI
 * PA1   ------> MISO
 * PB0   ------> CLK/SCK
 * PB3   ------> IRQ
 * PB4   ------> CE
 * PB5   ------> CSN
 */

#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"
#include "nrf24l01.h"
#include <string.h>

/* Mode options: MODE_TX, MODE_RX, MODE_RX_INT */
#define MODE_TX         0
#define MODE_TX_FAST    1
#define MODE_RX         2
#define MODE_RX_INT     3

#define NRF24_MODE      MODE_TX_FAST

/* Payload width options: fixed or dynamic */
#define PAYLOAD_WIDTH_MODE_FIXED    0
#define PAYLOAD_WIDTH_MODE_DYNAMIC  1

#define NRF24_PAYLOAD_WIDTH_MODE    PAYLOAD_WIDTH_MODE_DYNAMIC

#if (NRF24_MODE == MODE_TX || NRF24_MODE == MODE_TX_FAST)
uint8_t payload[] = {
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
    0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
    0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x47, 0x48};
#endif

uint8_t RX_ADDRESS[NRF24L01_ADDR_WIDTH] = {0x32,0x4E,0x6F,0x64,0x65};
uint8_t TX_ADDRESS[NRF24L01_ADDR_WIDTH] = {0x32,0x4E,0x6F,0x64,0x22};

extern uint8_t RX_BUF[];
extern uint8_t TX_BUF[];

static void APP_GPIO_Init(void);
static void APP_SPI_Init(void);
uint8_t SPI_TxRxByte(uint8_t data);

int main(void)
{
  BSP_RCC_HSI_48MConfig();
  LL_mDelay(2000); // avoid being brick
  BSP_USART_Config(115200);
  printf("nRF24L01 demo\r\n");

  APP_GPIO_Init();
  APP_SPI_Init();

  NRF24L01_Init();
  printf("nRF24L01 initialized\r\n");

  while (NRF24L01_Check() != 0)
  {
    printf("nRF24L01 check: error\r\n");
    LL_mDelay(2000);
  }
  printf("nRF24L01 check: succ\r\n");

#if (NRF24_PAYLOAD_WIDTH_MODE == PAYLOAD_WIDTH_MODE_DYNAMIC)
  NRF24L01_SetEnableDynamicPayloads(1);
  NRF24L01_SetEnableAckPayload(1);
#endif

#if (NRF24_MODE == MODE_RX)
  uint8_t pipe, length;
  printf("nRF24L01 in RX polling mode\r\n");
  NRF24L01_RX_Mode(TX_ADDRESS, RX_ADDRESS);

  NRF24L01_DumpConfig();
  while (1)
  {
    if (NRF24L01_RXFIFO_GetStatus() != NRF24L01_RXFIFO_STATUS_EMPTY)
    {
#if (NRF24_PAYLOAD_WIDTH_MODE == PAYLOAD_WIDTH_MODE_DYNAMIC)
      pipe = NRF24L01_ReadPayload(RX_BUF, &length, 1);
#else
      pipe = NRF24L01_ReadPayload(RX_BUF, &length, 0);
#endif
      printf("P:%d, L:%2d:", pipe, length);
      for (int i = 0; i < length; i++)
      {
        printf("%02X", RX_BUF[i]);
      }
      printf("\r\n");
      NRF24L01_ClearIRQFlags();
    }
  }

#elif (NRF24_MODE == MODE_RX_INT)
  printf("nRF24L01 in RX interrupt mode\r\n");
  NRF24L01_RX_Mode(TX_ADDRESS, RX_ADDRESS);
  NRF24L01_ClearIRQFlags();
  NRF24L01_DumpConfig();
  while(1);

#elif (NRF24_MODE == MODE_TX)
  uint8_t length = 0;
  printf("nRF24L01 in TX mode\r\n");
  NRF24L01_TX_Mode(RX_ADDRESS, TX_ADDRESS);
  NRF24L01_DumpConfig();

  while(1)
  {
#if (NRF24_PAYLOAD_WIDTH_MODE == PAYLOAD_WIDTH_MODE_DYNAMIC)
    length++;
#else
    length = NRF24L01_PLOAD_WIDTH;
#endif
    NRF24L01_TxPacket(payload, length);
    if (length == 32)
    {
      length = 0;
    }
    LL_mDelay(200);
  }

#elif (NRF24_MODE == MODE_TX_FAST)
  uint8_t length = 0;
  printf("nRF24L01 in fast TX mode\r\n");
  NRF24L01_TX_Mode(RX_ADDRESS, TX_ADDRESS);
  NRF24L01_DumpConfig();

  uint8_t succ = 0, err = 0;
  while (1)
  {
#if (NRF24_PAYLOAD_WIDTH_MODE == PAYLOAD_WIDTH_MODE_DYNAMIC)
    length++;
#else
    length = NRF24L01_PLOAD_WIDTH;
#endif
    if (NRF24L01_TxFast(payload, length) != 0)
    {
      NRF24L01_ResetTX();
      err++;
    }
    else
    {
      succ++;
    }
    if (err == 255 || succ == 255)
    {
      printf("Fail/Succ: %d/%d\r\n", err, succ);
      err = 0;
      succ = 0;
    }
    if (length == 32)
    {
      length = 0;
    }
    LL_mDelay(200);
  }

#endif
}

#if (NRF24_MODE == MODE_RX_INT)
void EXTI2_3_IRQHandler(void)
{
  uint8_t pipe, length;
  if(LL_EXTI_IsActiveFlag(LL_EXTI_LINE_3))
  {
#if (NRF24_PAYLOAD_WIDTH_MODE == PAYLOAD_WIDTH_MODE_DYNAMIC)
    pipe = NRF24L01_ReadPayload(RX_BUF, &length, 1);
#else
    pipe = NRF24L01_ReadPayload(RX_BUF, &length, 0);
#endif
    printf("P:%d, L:%2d:", pipe, length);
    for (int i = 0; i < length; i++)
    {
      printf("%02X", RX_BUF[i]);
    }
    printf("\r\n");
    //NRF24L01_FlushRX();
    NRF24L01_ClearIRQFlags();
    LL_EXTI_ClearFlag(LL_EXTI_LINE_3);
  }
}
#endif

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

static void APP_SPI_Init(void)
{
  LL_SPI_InitTypeDef initStruct = {0};

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

/*
 * PA0   ------> MOSI
 * PA1   ------> MISO
 * PB0   ------> CLK/SCK
 * PB3   ------> IRQ
 * PB4   ------> CE
 * PB5   ------> CSN
 */
static void APP_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIOA,GPIOB,GPIOC clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB | LL_IOP_GRP1_PERIPH_GPIOC);

  // PB4:CE, PB5:CSN GPIO output
  GPIO_InitStruct.Pin       = LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode      = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull      = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PB3:IRQ, GPIO input
  GPIO_InitStruct.Pin       = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode      = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull      = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  #if (NRF24_MODE == MODE_RX_INT)
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  /* Set EXTI3 to connected to PB3 pin */
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTB, LL_EXTI_CONFIG_LINE3);

  /* Triggerred by falling edge */
  EXTI_InitStruct.Line = LL_EXTI_LINE_3;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  NVIC_SetPriority(EXTI2_3_IRQn, 1);
  NVIC_EnableIRQ(EXTI2_3_IRQn);
  #endif
}

void APP_ErrorHandler(void)
{
  while (1);
}
