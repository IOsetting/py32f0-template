// This example was tested with 'PY32F003W16S6TU SOP16' chip
//
// It uses a modded version of the https://github.com/bob-01/STM32-SI5351/tree/master
// library.

#include "py32f0xx_bsp_printf.h"

#include "si5351.h"

#define I2C_ADDRESS 0xC0 // 0x60

I2C_HandleTypeDef hi2c1;

void APP_ErrorHandler(void);
static void APP_I2C_Config(void);

void I2C_Scan() {
  printf("Scanning I2C bus...\r\n");

  HAL_StatusTypeDef res;
  int found = 0;

  for (uint16_t i = 0; i < 128; i++) {
    res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
    if (res == HAL_OK) {
      char msg[64];
      found = 1;
      snprintf(msg, sizeof(msg), "Found at 0x%02X\n", i);
      printf(msg);
    }
  }
  if (!found)
    printf("An I2C device was not found - check connections!\n");
}

int main(void)
{
  HAL_Init();

  // PA2 ------> USART2_TX
  BSP_USART_Config();
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  APP_I2C_Config();

  // HAL_Delay(5000);

  I2C_Scan();

  int32_t si5351_FREQ_CORR = 0;
  uint8_t si5351_XTAL = 26; // I am using a 26 MHz TCXO, change according to your Si5351 setup

  si5351_init(&hi2c1, SI5351_BUS_BASE_ADDR, SI5351_CRYSTAL_LOAD_0PF, si5351_XTAL*1000000, si5351_FREQ_CORR);
  si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351_drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);

  si5351_set_freq(28074000*100ULL, SI5351_CLK0); // FT8 frequency on the 10m band

  while(1) {
    HAL_Delay(10);
  }
}

static void APP_I2C_Config(void)
{
  hi2c1.Instance             = I2C;
  hi2c1.Init.ClockSpeed      = 100000;        // 100KHz ~ 400KHz
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Export assert error source and line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1);
}
#endif /* USE_FULL_ASSERT */
