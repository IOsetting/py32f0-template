#include "main.h"
#include "uart.h"
#include "mpu6050.h"

static void APP_Rcc48MConfig(void);
static void APP_GpioConfig(void);

int main(void)
{
  uint8_t i = 0, j;
  int16_t buf[7];

  APP_Rcc48MConfig();
  APP_GpioConfig();
  MOD_UART_Config(115200);

  APP_I2cReconfig();
  MPU6050_Init();

  while (1)
  {
    if (i == 0)
    {
      /* In low power mode, gyroscope is off */
      MOD_UART_TxString("Enable low power mode\r\n");
      MPU6050_EnableLowPowerMode(MPU6050_Wakeup_Freq_1p25Hz);
    }
    else if (i == 0x20)
    {
      MOD_UART_TxString("Disable low power mode\r\n");
      MPU6050_DisableLowPowerMode();
    }
    MPU6050_ReadAll(buf);
    for (j = 0; j < 7; j++)
    {
      MOD_UART_TxHex((uint8_t *)&buf[j], 2);
      MOD_UART_TxChar(' ');
    }
    MOD_UART_TxString("\r\n");
    i++;
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_2);
    LL_mDelay(50);
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
