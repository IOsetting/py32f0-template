/***
 * Demo: nRF24L01
 * 
 * PY32          nRF24L01
 * PA0   ------> MISO 
 * PA1   ------> CLK/SCK
 * PA4   ------> IRQ
 * PA5   ------> CE
 * PA6   ------> CSN
 * PA7   ------> MOSI
 * 
 * PA2   ------> TX
 * PA3   ------> RX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "nrf24l01.h"

/* Mode options: MODE_TX, MODE_RX, MODE_RX_INT */
#define MODE_TX         0
#define MODE_TX_FAST    1
#define MODE_RX         2
#define MODE_RX_INT     3

#define NRF24_MODE      MODE_RX

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

static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);
static void APP_SPIConfig(void);


int main(void)
{
  /* Set clock = 48MHz */
  APP_SystemClockConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  BSP_USART_Config(115200);
  printf("SPI Demo: nRF24L01 Wireless\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIOConfig();
  APP_SPIConfig();

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
      BSP_UART_TxString("P:");
      BSP_UART_TxHex8(pipe);
      BSP_UART_TxString(",L:");
      BSP_UART_TxHex8(length);
      BSP_UART_TxChar(':');
      for (int i = 0; i < length; i++)
      {
        BSP_UART_TxHex8(RX_BUF[i]);
      }
      BSP_UART_TxString("\r\n");
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
    LL_mDelay(50);
  }

#endif
}

#if (NRF24_MODE == MODE_RX_INT)
void EXTI4_15_IRQHandler(void)
{
  uint8_t pipe, length;
  if(LL_EXTI_IsActiveFlag(LL_EXTI_LINE_4))
  {
#if (NRF24_PAYLOAD_WIDTH_MODE == PAYLOAD_WIDTH_MODE_DYNAMIC)
      pipe = NRF24L01_ReadPayload(RX_BUF, &length, 1);
#else
      pipe = NRF24L01_ReadPayload(RX_BUF, &length, 0);
#endif
    BSP_UART_TxString("P:");
    BSP_UART_TxHex8(pipe);
    BSP_UART_TxString(",L:");
    BSP_UART_TxHex8(length);
    BSP_UART_TxChar(':');
    for (int i = 0; i < length; i++) 
    {
        BSP_UART_TxHex8(RX_BUF[i]);
    }
    BSP_UART_TxString("\r\n");
    NRF24L01_ClearIRQFlags();
    LL_EXTI_ClearFlag(LL_EXTI_LINE_4);
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

static void APP_GPIOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;
#if (NRF24_MODE == MODE_RX_INT)
  LL_EXTI_InitTypeDef EXTI_InitStruct;
#endif

  // PA6 CSN
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  // PA5 CE
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  /* PA4 as input */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#if (NRF24_MODE == MODE_RX_INT)
  /* Triggerred by falling edge */
  EXTI_InitStruct.Line = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  NVIC_SetPriority(EXTI4_15_IRQn, 1);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
#endif
}

/**
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
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
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
