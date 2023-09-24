/***
 * Demo: I2C - DS3231 Real Time Clock
 * 
 * PY32          
 *  PF1          SCL
 *  PF0          SDA
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"


#define MASTER_ADDRESS      0xA0
#define DS3231_ADDRESS      0xD0

#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2

#define DS3231_REG_SECOND               0x00        /**< second register */
#define DS3231_REG_MINUTE               0x01        /**< minute register */
#define DS3231_REG_HOUR                 0x02        /**< hour register */
#define DS3231_REG_WEEK                 0x03        /**< week register */
#define DS3231_REG_DATE                 0x04        /**< date register */
#define DS3231_REG_MONTH                0x05        /**< month register */
#define DS3231_REG_YEAR                 0x06        /**< year register */
#define DS3231_REG_ALARM1_SECOND        0x07        /**< alarm1 second register */
#define DS3231_REG_ALARM1_MINUTE        0x08        /**< alarm1 minute register */
#define DS3231_REG_ALARM1_HOUR          0x09        /**< alarm1 hour register */
#define DS3231_REG_ALARM1_WEEK          0x0A        /**< alarm1 week register */
#define DS3231_REG_ALARM2_MINUTE        0x0B        /**< alarm2 minute register */
#define DS3231_REG_ALARM2_HOUR          0x0C        /**< alarm2 hour register */
#define DS3231_REG_ALARM2_WEEK          0x0D        /**< alarm2 week register */
#define DS3231_REG_CONTROL              0x0E        /**< control register */
#define DS3231_REG_STATUS               0x0F        /**< status register */
#define DS3231_REG_XTAL                 0x10        /**< xtal register */
#define DS3231_REG_TEMPERATUREH         0x11        /**< temperature high register */
#define DS3231_REG_TEMPERATUREL         0x12        /**< temperature low register */

typedef enum
{
    DS3231_FORMAT_12H = 0x01,        /**< 12h format */
    DS3231_FORMAT_24H = 0x00,        /**< 24h format */
} DS3231_HourFormat_t;

uint8_t buff[7];

__IO uint32_t   i2cState  = I2C_STATE_READY;

static void APP_SystemClockConfig(void);
static void APP_I2CConfig(void);
void APP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t size);
uint8_t APP_I2C_Receive(uint16_t devAddress, uint16_t memAddress, uint8_t *buf, uint16_t size);

uint8_t DS3231_Hex2Bcd(uint8_t hex)
{
  return (hex % 10) + ((hex / 10) << 4);
}

uint8_t DS3231_Bcd2Hex(uint8_t bcd)
{
  return (bcd >> 4) * 10 + (bcd & 0x0F);
}

uint8_t DS3231_GetStatus(void)
{
  APP_I2C_Receive(DS3231_ADDRESS, DS3231_REG_STATUS, buff, 1);
  return buff[0];
}

void DS3231_GetTime(uint8_t *t)
{
  APP_I2C_Receive(DS3231_ADDRESS, DS3231_REG_SECOND, buff, 7);
  t[0] = 19 + ((buff[5] >> 7) & 0x01);    // century
  t[1] = DS3231_Bcd2Hex(buff[6]);         // year
  t[2] = DS3231_Bcd2Hex(buff[5] & 0x1F);  // month
  t[3] = DS3231_Bcd2Hex(buff[3]);         // week
  t[4] = DS3231_Bcd2Hex(buff[4]);         // date
  t[8] = (buff[2] >> 6) & 0x01;           // 12h/24h
  t[9] = (buff[2] >> 5) & 0x01;           // am/pm
  if (t[8] == DS3231_FORMAT_12H)
  {
      t[5] = DS3231_Bcd2Hex(buff[2] & 0x1F); // hour
  }
  else
  {
      t[5] = DS3231_Bcd2Hex(buff[2] & 0x3F); // hour
  }
  t[6] = DS3231_Bcd2Hex(buff[1]); // minute
  t[7] = DS3231_Bcd2Hex(buff[0]); // second
}

int main(void)
{
  uint8_t time[10];

  APP_SystemClockConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: DS3231 Real Time Clock\r\nClock: %ld\r\n", SystemCoreClock);

  APP_I2CConfig();

  time[0] = DS3231_GetStatus();
  printf("Status: %d\r\n", time[0]);

  while(1)
  {
    DS3231_GetTime(time);
    printf("%02d%02d-%02d-%02d %02d:%02d:%02d %d-%d\r\n", 
        time[0], time[1], time[2], time[4], time[5], time[6], time[7], time[8], time[9]);
    LL_mDelay(1000);
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

static void APP_SystemClockConfig(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSI_Enable();
  /* Change this value to adjust frequency */
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
