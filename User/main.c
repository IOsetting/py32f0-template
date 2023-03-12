/***
 * Demo: LED Toggle
 * 
 * PA0   ------> LED+
 * GND   ------> LED-
 */
#include "py32f0xx_bsp_printf.h"


static void APP_GPIO_Config(void);

int main(void)
{
  HAL_Init();                                 
  APP_GPIO_Config();
  BSP_USART_Config();
  printf("PY32F0xx LED Toggle Demo\r\nSystem Clock: %ld\r\n", SystemCoreClock);
  
  while (1)
  {
    HAL_Delay(1000);                            
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    printf("echo\r\n");
  }
}

static void APP_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  // PA0
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
