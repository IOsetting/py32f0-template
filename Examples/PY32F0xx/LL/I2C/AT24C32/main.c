/***
 * Demo: I2C - AT24C32 Read/Write
 * 
 * PY32          
 *  PF1          SCL
 *  PF0          SDA
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"


#define MASTER_ADDRESS      0xA0
#define AT24C_ADDRESS       0xAE
#define AT24C_MEM_ADDRESS   0x80

#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2

/*
 * EERPOM data address length
 * 
 * 1-byte: AT24C01 - AT24C16
 * 2-byte: AT24C32+
 */
#define AT24C_DATA_ADDR_8B    0
#define AT24C_DATA_ADDR_16B   1
#define AT24C_DATA_ADDR_LEN   AT24C_DATA_ADDR_16B

#define SIZE sizeof(test_text)
const uint8_t test_text[]={"I2C read/write test"};

__IO uint32_t   i2cState  = I2C_STATE_READY;

static void APP_SystemClockConfig(void);
static void APP_I2CConfig(void);
void APP_I2C_Transmit(uint8_t devAddress, uint16_t memAddress, uint8_t *pData, uint16_t len);
uint8_t APP_I2C_ReceiveByte(uint16_t devAddress, uint16_t memAddress);

void AT24CXX_Read(uint16_t dataAddr, uint8_t *pBuffer, uint16_t len)
{
  while (len--)
  {
    *pBuffer++ = APP_I2C_ReceiveByte(AT24C_ADDRESS, dataAddr++);
    LL_mDelay(5);
  }
}

void AT24CXX_Write(uint16_t dataAddr, uint8_t *pBuffer, uint16_t len)
{
  while (len--)
  {
    APP_I2C_Transmit(AT24C_ADDRESS, dataAddr++, pBuffer++, 1);
    // Writing might halt if delay is too short
    LL_mDelay(5);
  }
}

int main(void)
{
  uint8_t buf[SIZE];

  APP_SystemClockConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: AT24C32\r\nClock: %ld\r\n", SystemCoreClock);

  APP_I2CConfig();

  printf("Start writing...");
  AT24CXX_Write(AT24C_MEM_ADDRESS, (uint8_t *)test_text, SIZE);
  printf("succ\r\n");

  printf("Start reading...");
  AT24CXX_Read(AT24C_MEM_ADDRESS, buf, SIZE);
  printf("read: %s\r\n", buf);

  while(1);
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

void APP_I2C_Transmit(uint8_t devAddress, uint16_t memAddress, uint8_t *pData, uint16_t len)
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
#if (AT24C_DATA_ADDR_LEN == AT24C_DATA_ADDR_8B)
  LL_I2C_TransmitData8(I2C1, (uint8_t)(memAddress & 0x00FF));
#elif (AT24C_DATA_ADDR_LEN == AT24C_DATA_ADDR_16B)
  LL_I2C_TransmitData8(I2C1, (uint8_t)((memAddress & 0xFF00) >> 8));
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);

  LL_I2C_TransmitData8(I2C1, (uint8_t)(memAddress & 0x00FF));
#endif
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);

  /* Transfer data */
  while (len > 0)
  {
    while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
    LL_I2C_TransmitData8(I2C1, *pData++);
    len--;

    if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (len != 0U))
    {
      LL_I2C_TransmitData8(I2C1, *pData++);
      len--;
    }

    while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
  }

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);
  i2cState = I2C_STATE_READY;
}

uint8_t APP_I2C_ReceiveByte(uint16_t devAddress, uint16_t memAddress)
{
  uint8_t temp = 0;

  LL_I2C_DisableBitPOS(I2C1);
  i2cState    = I2C_STATE_BUSY_RX;

  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Send memory address */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
#if (AT24C_DATA_ADDR_LEN == AT24C_DATA_ADDR_8B)
  LL_I2C_TransmitData8(I2C1, (uint8_t)(memAddress & 0x00FF));

#elif (AT24C_DATA_ADDR_LEN == AT24C_DATA_ADDR_16B)
  LL_I2C_TransmitData8(I2C1, (uint8_t)((memAddress & 0xFF00) >> 8));
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);

  LL_I2C_TransmitData8(I2C1, (uint8_t)(memAddress & 0x00FF));
#endif
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);

  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);

  /* Send slave address(read) */
  LL_I2C_TransmitData8(I2C1, (devAddress | 0x1));
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  LL_I2C_ClearFlag_ADDR(I2C1);

  while(LL_I2C_IsActiveFlag_RXNE(I2C1) != 1);
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
  temp = LL_I2C_ReceiveData8(I2C1);
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
// static void APP_SystemClockConfig(void)
// {
//   LL_RCC_HSI_Enable();
//   LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_8MHz);
//   while(LL_RCC_HSI_IsReady() != 1);

//   LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

//   LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
//   while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

//   LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

//   LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
//   /* Update global SystemCoreClock(or through SystemCoreClockUpdate function) */
//   LL_SetSystemCoreClock(8000000);
//   /* Re-init frequency of SysTick source, reload = freq/ticks = 48000000/1000 = 48000 */
//   LL_InitTick(8000000, 1000U);
// }

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
