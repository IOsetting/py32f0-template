/**
 * There is a good chance the PY32F002A you got is a relabeled PY32F003X6
 * or PY32F030X6, this code will test the hidden resources of PY32F002A
 * 
 * 1. Fill flash to 32KB
 * 2. Fill RAM to 4KB
 * 3. Enable PLL to make system clock run at 48MHz
 * 
 * Notes:
 * 
 * 1. Before compiling, edit Makefile, change all options to PY32F030X6 
 *    instead of PY32F002A
 * 2. In case the code size exceed 32KB, edit dummy_data.h and dummy_data.c
 *    to shrink the array size.
*/
#include "py32f0xx_hal.h"
#include "py32f0xx_bsp_printf.h"
#include "dummy_data.h"

#define BUF_SIZE 1900

uint8_t buf[BUF_SIZE];

static void APP_LedConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int main(void)
{
  uint16_t pos = DUMMY_DATA_SIZE - 1, pos2 = BUF_SIZE - 1;

  HAL_Init();                                 
  APP_LedConfig();
  BSP_USART_Config();

  buf[0] = 0xAA;
  buf[BUF_SIZE - 1] = 0xBB;

  while (1)
  {
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    buf[pos2 - 2] = dummy_data[pos];
    printf("flash:%02x ram:%02x %02x %02x\r\n", dummy_data[pos], buf[pos2], buf[pos2 - 1], buf[pos2 - 2]);
    if (--pos == 0)
    {
      pos = DUMMY_DATA_SIZE - 1;
    }
    if (--pos2 == 1)
    {
      pos2 = BUF_SIZE - 1;
    }
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
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
