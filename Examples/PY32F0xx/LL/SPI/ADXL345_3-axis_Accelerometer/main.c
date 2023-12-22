/***
 * Demo: ADXL345 3-axis accelerometer
 * 
 * PA6   ------> CS      (GPIO OUT)
 * PA4   ------> INT1    (GPIO IN)
 * PA5   ------> INT2    (GPIO IN)
 * PA0   ------> SDO     (MISO)
 * PA7   ------> SDI/SDA (MOSI)
 * PA1   ------> SCL     (SCLK)
 * 
 * PA2   ------> TX
 * PA3   ------> RX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "adxl345.h"

__IO uint8_t count_int1 = 0, count_int2 = 0, count_double_tap = 0;
__IO int16_t xbuf[3];

static void APP_GPIOConfig(void);
static void APP_SPIConfig(void);

int main(void)
{
  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("SPI Demo: ADXL345, 3-axis accelerometer\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIOConfig();
  APP_SPIConfig();

  if (ADXL345_Init(
      ADXL345_DATARATE_100_HZ,
      ADXL345_SPI_WIRE_4,
      ADXL345_INT_ACTIVE_LOW,
      ADXL345_DATA_RESOLVE_FULL,
      ADXL345_DATA_ALIGNMENT_RIGHT,
      ADXL345_G_RANGE_8G
  ) == SUCCESS)
  {
    printf("ADXL345_Init success\r\n");
  }
  else
  {
    printf("ADXL345_Init failed\r\n");
  }
  // Tap threshold: 62.5mg / LSB, value = 2.5g / 0.0625g = 0x28
  ADXL345_WriteByte(ADXL345_REG_THRESH_TAP, 0x28);
  // Tap duration: 625us / LSB, value = 0.02s / 0.000625s = 0x20
  ADXL345_WriteByte(ADXL345_REG_DUR, 0x20);
  // Tap latency: 1.25ms / LSB, value = 0.1s / 0.00125s = 0x50
  ADXL345_WriteByte(ADXL345_REG_LATENT, 0x50);
  // Tap window: 1.25ms / LSB, value = 0.3s / 0.00125s = 0xF0
  ADXL345_WriteByte(ADXL345_REG_WINDOW, 0xF0);
  ADXL345_EnableTapDetectOnAxes(
      ADXL345_TAP_DETECT_AXIS_X|ADXL345_TAP_DETECT_AXIS_Y|ADXL345_TAP_DETECT_AXIS_Z);
  // Remap DATA_READY to INT2
  ADXL345_RemapInterrupts(ADXL345_INT_DATA_READY);
  ADXL345_SetInterrupts(
      ADXL345_INT_DATA_READY|ADXL345_INT_SINGLE_TAP|ADXL345_INT_DOUBLE_TAP);
  /*
   * The interrupt functions are latched and cleared by either reading the data registers 
   * (Address 0x32 to Address 0x37) until the interrupt condition is no longer valid for 
   * the data-related interrupts or by reading the INT_SOURCE register (Address 0x30) for 
   * the remaining interrupts.
   */
  // or ADXL345_BurstRead(ADXL345_REG_DATAX0, (uint8_t *)xbuf, 6);
  xbuf[0] = ADXL345_ReadInt(ADXL345_REG_DATAX0);
  xbuf[0] = ADXL345_ReadInt(ADXL345_REG_DATAY0);
  xbuf[0] = ADXL345_ReadInt(ADXL345_REG_DATAZ0);
  xbuf[0] = ADXL345_ReadByte(ADXL345_REG_INT_SOURCE);

  while (1)
  {
    printf("X:%6d, Y:%6d, Z:%6d, DAT:%3d, TAP:%3d, 2-TAP:%3d\r\n",
           xbuf[0], xbuf[1], xbuf[2], count_int2, count_int1, count_double_tap);
    LL_mDelay(50);
  }
}

static void APP_GPIOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  // PA6 CSN
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  /* PA4 INT1 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // PA5 INT2
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Triggerred by falling edge */
  EXTI_InitStruct.Line = LL_EXTI_LINE_4 | LL_EXTI_LINE_5;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  NVIC_SetPriority(EXTI4_15_IRQn, 1);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
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
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_Enable(SPI1);
}

uint8_t SPI_TxRxByte(uint8_t data)
{
  uint8_t SPITimeout = 0xFF;
  /* Check the status of Transmit buffer Empty flag */
  while (READ_BIT(SPI1->SR, SPI_SR_TXE) == RESET)
  {
    if (SPITimeout-- == 0)
      return 0;
  }
  LL_SPI_TransmitData8(SPI1, data);
  SPITimeout = 0xFF;
  while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == RESET)
  {
    if (SPITimeout-- == 0)
      return 0;
  }
  // Read from RX buffer
  return LL_SPI_ReceiveData8(SPI1);
}

void EXTI4_15_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag(LL_EXTI_LINE_4)) // PA4 -> INT1
  {
    count_int1++;
    if (ADXL345_IsInterrupt(ADXL345_INT_DOUBLE_TAP) > 0)
    {
        count_double_tap++;
    }
    LL_EXTI_ClearFlag(LL_EXTI_LINE_4);
  }
  else if (LL_EXTI_IsActiveFlag(LL_EXTI_LINE_5)) // PA5 -> INT2
  {
    count_int2++;
    ADXL345_BurstRead(ADXL345_REG_DATAX0, (uint8_t *)xbuf, 6);
    LL_EXTI_ClearFlag(LL_EXTI_LINE_5);
  }
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
