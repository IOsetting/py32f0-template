// This example was tested with 'PY32F003W16S6TU SOP16' chip

#include "py32f0xx_hal_dma.h"
#include "py32f0xx_hal_i2c.h"
#include "py32f0xx_bsp_printf.h"

#include "LCD1602.h"

#define I2C_ADDRESS 0xA0 // host address

I2C_HandleTypeDef I2cHandle;

void APP_ErrorHandler(void);
static void APP_I2C_Config(void);

int SLAVE_ADDRESS_LCD = 0; // PCF8574

int find_lcd_i2c_address() {
  int ret = -1;

  for (int i = 1; i < 128; i++) {
    ret = HAL_I2C_IsDeviceReady(&I2cHandle, (uint16_t)(i << 1), 3, 1000);
    if (ret != HAL_OK) {
    }
    else if (ret == HAL_OK) {
      return i;
    }
  }

  return -1;
}

int main(void)
{
  HAL_Init();

  // BSP_USART_Config();
  // printf("SystemClk:%ld\r\n", SystemCoreClock);

  APP_I2C_Config();

  SLAVE_ADDRESS_LCD = find_lcd_i2c_address();
  SLAVE_ADDRESS_LCD = (SLAVE_ADDRESS_LCD << 1);

  LCD_Init();
  LCD_SetCursor(0, 0);
  LCD_PrintString("HELLO WORLD!");
  HAL_Delay(1000);
  LCD_SetCursor(0, 1);
  LCD_PrintString("HOWDY!");
  HAL_Delay(2000);

  while(1);
}

static void APP_I2C_Config(void)
{
  I2cHandle.Instance             = I2C;
  I2cHandle.Init.ClockSpeed      = 100000;        // 100KHz ~ 400KHz
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

void APP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t len)
{
  HAL_I2C_Mem_Write(&I2cHandle, devAddress, memAddress, I2C_MEMADD_SIZE_8BIT, pData, len, 5000);
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
