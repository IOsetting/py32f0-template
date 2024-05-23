/**
  ******************************************************************************
  * PY32F002A/003/030 Deep Sleep Mode
  * 
  * Connections
  *  VCC      ---> Microammeter --> 3.3V
  *  GND      ---> GND 
  *  PA0      ---> Key          --> GND
  *  PA1      ---> +LED-        --> GND
  *  
  * - Don't connect any other pins to avoid possible current leakage
  * - MCU enters stop mode after 2 seconds, current drops from 1.3 mA to around 6 uA
  * - Press key (short PA0 to GND) will bring MCU back to run mode, current then rises to 1.5 mA
  * - LED blink three times, then MCU enters stop mode again
  */

#include "py32f0xx_bsp_clock.h"

static void APP_ExtiConfig(void);
static void APP_GPIO_ConfigOutput(void);
static void APP_GPIO_ConfigAnalog(void);

int main(void)
{
  uint16_t i = 0;
  HAL_Init();

  BSP_HSI_24MHzClockConfig();

  HAL_Delay(2000);

  APP_ExtiConfig();

  __HAL_RCC_PWR_CLK_ENABLE();

  APP_GPIO_ConfigOutput();
  while (1)
  {
    i++;
    if (i > 6)
    {
      // Set IO mode to analog to avoid current leakage
      APP_GPIO_ConfigAnalog();
      HAL_SuspendTick();
      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);  
      HAL_ResumeTick();
      APP_GPIO_ConfigOutput();
      i = 0;
    }
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
  }
}

static void APP_ExtiConfig(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  // PA0
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
}

static void APP_GPIO_ConfigOutput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  // PA1
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

static void APP_GPIO_ConfigAnalog(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  // PA1
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
