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

uint8_t RX_ADDRESS[NRF24L01_ADDR_WIDTH] = {0x32,0x4E,0x6F,0x64,0x65};
uint8_t TX_ADDRESS[NRF24L01_ADDR_WIDTH] = {0x32,0x4E,0x6F,0x64,0x22};

/*-----------------------------------------------------*/
/* Mode: sending:1 or receiving:0 */
uint8_t Mode = 1;
/*-----------------------------------------------------*/
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
    printf("nRF24L01 check failed\r\n");
    LL_mDelay(2000);
  }
  printf("nRF24L01 check succeeded\r\n");

  if (Mode == 1)
  {
    printf("nRF24L01 in SEND mode\r\n");
    NRF24L01_TX_Mode(RX_ADDRESS, TX_ADDRESS);
  }
  else if (Mode == 0)
  {
    printf("nRF24L01 in RECEIVE mode\r\n");
    NRF24L01_RX_Mode(TX_ADDRESS, RX_ADDRESS);
  }

  while (1)
  {
    NRF24L01_DumpConfig();

    if (Mode == 1)
    {
      uint8_t tmp[] = {0x1f,
                       0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                       0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
                       0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
                       0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x47, 0x48};
      NRF24L01_TxPacket(tmp, 32);
      LL_mDelay(1000);
    }
    else if (Mode == 0)
    {
      NRF24L01_RxPacket(RX_BUF);
    }
  }
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

static void APP_GPIOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  // PA6 CSN
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  // PA5 CE
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  /* PA4 as input */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
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
