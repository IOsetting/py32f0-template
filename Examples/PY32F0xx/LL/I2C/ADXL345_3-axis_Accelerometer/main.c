/***
 * Demo: ADXL345 3-axis accelerometer
 * 
 *               CS       ---> 3.3V
 * PA4   ------> INT1
 * PA5   ------> INT2
 *               SDO      ---> GND or 3.3V
 * PF0   ------> SDI/SDA
 * PF1   ------> SCL
 * 
 * PA2   ------> TX
 * PA3   ------> RX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "adxl345.h"

#define MASTER_ADDRESS      0xA0

#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2

__IO uint32_t   i2cState  = I2C_STATE_READY;


__IO uint8_t count_int1 = 0, count_int2 = 0, count_double_tap = 0;
__IO int16_t xbuf[3];

static void APP_GPIOConfig(void);
static void APP_I2CConfig(void);

int main(void)
{
  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: ADXL345, 3-axis accelerometer\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIOConfig();
  APP_I2CConfig();

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
  ADXL345_BurstRead(ADXL345_REG_DATAX0, (uint8_t *)xbuf, 6);

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

  // PA6 CSN, set it to high to enable I2C mode
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
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

static void APP_I2CConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  // PF1 SCL
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  // PF0 SDA
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

  LL_I2C_InitTypeDef I2C_InitStruct;
  /*
   * Clock speed:
   * - standard = 100khz
   * - fast     = 400khz
  */
  I2C_InitStruct.ClockSpeed      = LL_I2C_MAX_SPEED_FAST;
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1     = MASTER_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
}

void APP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t size)
{
  /* Wait until BUSY flag is reset */
  while (i2cState == I2C_STATE_BUSY_TX);
  /* Disable Pos */
  LL_I2C_DisableBitPOS(I2C1);

  i2cState = I2C_STATE_BUSY_TX;
  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  /* Wait until ADDR flag is set */
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  /* Clear ADDR flag */
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Wait until TXE flag is set */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, memAddress);

  /* Transfer data */
  while (size > 0)
  {
    while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
    LL_I2C_TransmitData8(I2C1, *pData++);
    size--;

    if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (size != 0U))
    {
      LL_I2C_TransmitData8(I2C1, *pData++);
      size--;
    }

    while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
  }

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);

  i2cState = I2C_STATE_READY;
}

void APP_I2C_Receive(uint16_t devAddress, uint16_t memAddress, uint8_t *buf, uint16_t size)
{
  i2cState    = I2C_STATE_BUSY_RX;
  /* Disable Pos */
  LL_I2C_DisableBitPOS(I2C1);

  /***** Send device address + memory address *****/

  /* Enable Acknowledge */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);

  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  /* Wait until ADDR flag is set */
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  /* Clear ADDR flag */
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Wait until TXE flag is set */
  while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, (uint8_t)(memAddress & 0x00FF));
  while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);

  /***** Restart to read *****/

  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);

  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress | 0x1));
  /* Wait until ADDR flag is set */
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);

  if (size == 0U)
  {
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_GenerateStopCondition(I2C1);
  }
  else if(size == 1U)
  {
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

    __disable_irq();
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_GenerateStopCondition(I2C1);
    __enable_irq();
  }
  else if(size == 2U)
  {
    LL_I2C_EnableBitPOS(I2C1);

    __disable_irq();
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
    __enable_irq();
  }
  else
  {
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
    LL_I2C_ClearFlag_ADDR(I2C1);
  }

  while (size > 0U)
  {
    if (size <= 3U)
    {
      if (size == 1U)
      {
        while(LL_I2C_IsActiveFlag_RXNE(I2C1) != 1);
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
      }
      else if (size == 2U)
      {
        while(LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        
        __disable_irq();
        LL_I2C_GenerateStopCondition(I2C1);
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
        __enable_irq();
        
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
      }
      else
      {
        while(LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
        
        __disable_irq();
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
        while(LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        LL_I2C_GenerateStopCondition(I2C1);
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
        __enable_irq();
        
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
      }
    }
    else
    {
      while(LL_I2C_IsActiveFlag_RXNE(I2C1) != 1);
      
      *buf++ = LL_I2C_ReceiveData8(I2C1);
       size--;
      
      if (LL_I2C_IsActiveFlag_BTF(I2C1) == 1)
      {
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
      }
    }
  }

  i2cState = I2C_STATE_READY;
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
