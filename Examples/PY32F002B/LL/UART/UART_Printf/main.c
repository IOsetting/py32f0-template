/**
  ******************************************************************************
  * PY32F002B UART Printf Demo
  * 
  * PA3     -->              USB2TTL-RX
  * PA4     -->              USB2TTL-TX
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"

int main(void)
{
  uint8_t count = 0;
  BSP_RCC_HSI_48MConfig();

  BSP_USART_Config(115200);
  printf("PY32F002B UART Printf Demo\r\nClock: %ld\r\n", SystemCoreClock);

  while (1)
  {
    printf("print count: %d\r\n", count++);
    LL_mDelay(1000);
  }
}

void APP_ErrorHandler(void)
{
  /* Infinite loop */
  while (1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file：Pointer to the source file name
  * @param  line：assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add His own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1);
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
