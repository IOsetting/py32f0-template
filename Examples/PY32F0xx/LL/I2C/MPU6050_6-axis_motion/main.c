/***
 * Demo: MPU6050, 3-axis gyroscope + 3-axis accelerometer
 * 
 *              PF1   -> SCL
 *              PF0   -> SDA
 *              GND   -> GND
 *              3.3V  -> VCC
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "mpu6050.h"


#define MASTER_ADDRESS      0xA0

#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2

__IO uint32_t   i2cState  = I2C_STATE_READY;

static void APP_I2CConfig(void);

int main(void)
{
  uint8_t i = 0;
  int16_t buf[7];

  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: MPU6050, 3-axis gyroscope + 3-axis accelerometer\r\nClock: %ld\r\n", SystemCoreClock);

  APP_I2CConfig();

  MPU6050_Init();

  while (1)
  {
    if (i == 0)
    {
      /* In low power mode, gyroscope is off */
      printf("Enable low power mode\r\n");
      MPU6050_EnableLowPowerMode(MPU6050_Wakeup_Freq_1p25Hz);
    }
    else if (i == 0x80)
    {
      printf("Disable low power mode\r\n");
      MPU6050_DisableLowPowerMode();
    }
    MPU6050_ReadAll(buf);
    printf("ax:%6d  ay:%6d  az:%6d  tp:%6d, gx:%6d  gy:%6d  gz:%6d\r\n",
            buf[0], buf[1], buf[2], (int16_t)buf[3] / 34 + 365, buf[4], buf[5], buf[6]);
    i++;
    LL_mDelay(100);
  }
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
  while (i2cState == I2C_STATE_BUSY_TX);
  LL_I2C_DisableBitPOS(I2C1);

  i2cState = I2C_STATE_BUSY_TX;
  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Send memory address */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  LL_I2C_TransmitData8(I2C1, memAddress);
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);

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

uint8_t APP_I2C_Receive(uint16_t devAddress, uint16_t memAddress, uint8_t *buf, uint16_t size)
{
  uint8_t temp = 0;

  i2cState    = I2C_STATE_BUSY_RX;
  /* Turn on ACK */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait the status of Start Bit */
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);

  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  /* Wait the status of Address sent (master mode) */
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  /* Clear Address Matched flag */
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, (uint8_t)(memAddress & 0x00FF));
  while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);

  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);

  /* Send slave address(read) */
  LL_I2C_TransmitData8(I2C1, (devAddress | 0x1));
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  LL_I2C_ClearFlag_ADDR(I2C1);

  while (size--)
  {
    while(LL_I2C_IsActiveFlag_RXNE(I2C1) != 1);
    *buf++ = LL_I2C_ReceiveData8(I2C1);
  }
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
  LL_I2C_GenerateStopCondition(I2C1);

  i2cState = I2C_STATE_READY;
  return temp;
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
