#include "py32f07x_hal.h"
#include "py32f07x_bsp_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void APP_ErrorHandler(void);
static void APP_GPIO_Config(void);


void task1(void *pvParameters)
{
  (void)(pvParameters); // Suppress "unused parameter" warning

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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

  HAL_Init();                                  
  
  /* GPIO Initialization */
  APP_GPIO_Config();

  BSP_USART_Config();

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
