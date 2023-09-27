#include "py32f07x_hal.h"

static void APP_GPIO_Config(void);

int main(void)
{
  HAL_Init();                                  
  
  /* GPIO Initialization */
  APP_GPIO_Config();

  while (1)
  {
    /* Delay 500ms */
    HAL_Delay(500);   

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}

static void APP_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOC_CLK_ENABLE();                          /* Enable GPIOC Clock */

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
