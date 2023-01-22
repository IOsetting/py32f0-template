#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);

void task1(void *pvParameters)
{
  (void)(pvParameters); // Suppress "unused parameter" warning

  while (1)
  {
    LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_5);
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

  APP_SystemClockConfig();
  APP_GPIOConfig();

  BSP_USART_Config(115200);
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

static void APP_SystemClockConfig(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSI_Enable();
  /* Change this value to adjust frequency */
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while(LL_RCC_HSI_IsReady() != 1);

  UTILS_ClkInitStruct.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
  UTILS_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_1;
  LL_PLL_ConfigSystemClock_HSI(&UTILS_ClkInitStruct);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Don't invoke this in FreeRTOS */
  // LL_InitTick(48000000, 1000U);
  /* Update global SystemCoreClock(or through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
}

static void APP_GPIOConfig(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

void APP_ErrorHandler(void)
{
  while (1);
}
