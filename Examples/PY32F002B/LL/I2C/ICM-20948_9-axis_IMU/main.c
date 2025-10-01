#include "main.h"
#include "bsp_uart.h"
#include "icm20948.h"

static void APP_Rcc48MConfig(void);
static void APP_GpioConfig(void);

int main(void)
{
  int16_t buff[7], magbuf[3];

  APP_Rcc48MConfig();
  APP_GpioConfig();
  BSP_UartReconfig(115200);
  BSP_UartTxString("I2C Demo: ICM-20948, 9-axis IMU\r\nClock:");
  BSP_UartTxInt(SystemCoreClock, 10);
  BSP_UartTxString("\r\n");

  APP_I2cReconfig();

  if (ICM20948_Detect(ICM20948_SLAVE_ADDR) == SUCCESS)
  {
    BSP_UartTxString("ICM20948 detected, ");
    BSP_UartTxLine("ID: ", ICM20948_GetWhoami());
  }
  else
  {
    BSP_UartTxString("ICM20948 not detected\r\n");
  }
  ICM20948_Init();
  ICM20948_SetSleep(0);

  ICM20948_SetEnBypass(1);
  if (AK09916_Detect(AK09916_SLAVE_ADDR) == SUCCESS)
  {
    BSP_UartTxString("AK09916 detected, ");
    BSP_UartTxLine("ID: ", AK09916_GetWhoami());
  }
  else
  {
    BSP_UartTxString("AK09916 not detected\r\n");
  }
  AK09916_SetMode(AK09916_MAG_MODE_CONT_100HZ);
  ICM20948_SetEnBypass(0);

  ICM20948_SetGyroConfig(0, 0x01, 0x02);
  ICM20948_SetAccelConfig(0, 0x01, 0x02);

  while (1)
  {
    ICM20948_ReadAll(buff);
    ICM20948_SetEnBypass(1);
    AK09916_ReadAll(magbuf);
    ICM20948_SetEnBypass(0);

    BSP_UartTxString("A: ");
    BSP_UartTxInt(buff[0], 7);
    BSP_UartTxInt(buff[1], 7);
    BSP_UartTxInt(buff[2], 7);
    BSP_UartTxString(",  G: ");
    BSP_UartTxInt(buff[3], 7);
    BSP_UartTxInt(buff[4], 7);
    BSP_UartTxInt(buff[5], 7);
    BSP_UartTxString(",  T: ");
    BSP_UartTxInt(buff[6], 7);

    BSP_UartTxString(",  M: ");
    BSP_UartTxInt(magbuf[0], 7);
    BSP_UartTxInt(magbuf[1], 7);
    BSP_UartTxInt(magbuf[2], 7);
    BSP_UartTxString("\r\n");
    LL_mDelay(100);
  }

}

void APP_Rcc48MConfig(void)
{
  /*  Set FLASH Latency = 1 for 48MHz */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  /* SET HSI to 48MHz */
  LL_RCC_HSI_SetCalibTrimming(LL_RCC_HSICALIBRATION_48MHz);
  /* Enable HSI */
  LL_RCC_HSI_Enable();

  while(LL_RCC_HSI_IsReady() != 1);
  /* Set AHB divider: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  /* HSISYS used as SYSCLK clock source  */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(48000000);
  LL_SetSystemCoreClock(48000000);
}

static void APP_GpioConfig(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
}

void APP_I2cReconfig(void)
{
  // PB3 SCL
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF_6);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
  // PB4 SDA
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_6);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_UP);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
  // Initialize I2C
  LL_I2C_Disable(I2C1);
  // PCLK1_Frequency = 48M = 0x02DC6C00
  LL_I2C_ConfigSpeed(I2C1, 0x02DC6C00, LL_I2C_MAX_SPEED_FAST, LL_I2C_DUTYCYCLE_16_9);
  LL_I2C_SetOwnAddress1(I2C1, 0xA0, 0);
  LL_I2C_Enable(I2C1);
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
}

void BSP_RccNop(uint16_t nop)
{
  nop *= 12;
  while(nop--) __NOP();
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
