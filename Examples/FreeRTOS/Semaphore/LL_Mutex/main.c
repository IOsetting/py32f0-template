/**
 * Example of Mutex
 * 
 * Note: Require RAM > 4.5K Byte
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

const uint8_t ids[] = {1,2,3};
SemaphoreHandle_t mutex;
volatile uint16_t counter;

static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);

void taskCount(void *pvParameters)
{
  uint8_t id = *(uint8_t *)pvParameters;

  while (1)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    counter++;
    BSP_UART_TxHex8(id);
    BSP_UART_TxChar(':');
    BSP_UART_TxHex16(counter);
    BSP_UART_TxString("\r\n");
    xSemaphoreGive(mutex);
    vTaskDelay(0);
  }
}

int main(void)
{
  uint8_t i;

  APP_SystemClockConfig();
  APP_GPIOConfig();

  BSP_USART_Config(115200);
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  // Create mutex
  mutex = xSemaphoreCreateMutex();
  // Create tasks
  for (i = 0; i < 3; i++)
  {
    printf("Task %d\n", ids[i]);
    if (xTaskCreate(taskCount, "taskProducer", configMINIMAL_STACK_SIZE, (void *)&ids[i], 2, NULL) != pdPASS)
    {
      APP_ErrorHandler();
    }
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
