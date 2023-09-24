/**
 * FreeRTOS example
*/
#include "py32f0xx_hal.h"
#include "py32f0xx_bsp_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void APP_ErrorHandler(void);

void APP_LedConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void task1(void *pvParameters)
{
  (void)(pvParameters); // Suppress "unused parameter" warning

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    vTaskDelay(500);
  }
}

void task2(void *pvParameters)
{
  (void)(pvParameters);

  while (1)
  {
    printf("task2 echo\r\n");
    vTaskDelay(1000);
  }
}

int main(void)
{
  BaseType_t xReturned;

  APP_LedConfig();
  BSP_USART_Config();
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  /* Create task 1 */
  xReturned = xTaskCreate(
      task1,                    // Task function point
      "Task1",                  // Task name
      configMINIMAL_STACK_SIZE, // Use the minimum stack size, each take 4 bytes(32bit)
      NULL,                     // Parameters
      2,                        // Priority
      NULL);                    // Task handler
  if (xReturned != pdPASS)
  {
    APP_ErrorHandler();
  }

  /* Create task 2 */
  xReturned = xTaskCreate(task2, "Task2", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  if (xReturned != pdPASS)
  {
    APP_ErrorHandler();
  }

  printf("FreeRTOS Scheduler starting...\r\n");
  /* Start the scheduler. */
  vTaskStartScheduler();

  /* Will only get here if there was not enough heap space to create the idle task. */
  return 0;
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
