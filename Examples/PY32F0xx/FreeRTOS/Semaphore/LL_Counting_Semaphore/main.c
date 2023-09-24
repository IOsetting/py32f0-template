/**
 * Example of Counting Semaphore
 * - 3 producers, each generate 255 numbers
 * - 2 consumers, print the produced numbers
 * 
 * Note: Require RAM > 6.5K Byte
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define QUEUE_SIZE      5
#define PRODUCERS       3
#define CONSUMERS       2
#define LIMIT           0x100

SemaphoreHandle_t mutex;
SemaphoreHandle_t binary_sema;
SemaphoreHandle_t empty_sema;
SemaphoreHandle_t filled_sema;

const uint16_t ids[] = {1,2,3,4,5};
uint16_t buf[QUEUE_SIZE], from = 0, to = 0;

static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);

void taskProducer(void *pvParameters)
{
  uint16_t count, id = *(uint16_t *)pvParameters;

  for (count = 0; count < LIMIT; count++)
  {
    xSemaphoreTake(empty_sema, portMAX_DELAY);
    xSemaphoreTake(mutex, portMAX_DELAY);
    buf[to] = ((id & 0xFF) << 8) + count;
    to = (to + 1) % QUEUE_SIZE;
    xSemaphoreGive(mutex);
    xSemaphoreGive(filled_sema);
  }
  vTaskDelete(NULL);
}

void taskConsumer(void *pvParameters)
{
  uint16_t id = *(uint16_t *)pvParameters;

  while (1)
  {
    xSemaphoreTake(filled_sema, portMAX_DELAY);
    xSemaphoreTake(mutex, portMAX_DELAY);
    BSP_UART_TxHex16(id);
    BSP_UART_TxChar(':');
    BSP_UART_TxHex16(buf[from]);
    BSP_UART_TxString("\r\n");
    from = (from + 1) % QUEUE_SIZE;
    xSemaphoreGive(mutex);
    xSemaphoreGive(empty_sema);
  }
}

int main(void)
{
  uint8_t i;

  APP_SystemClockConfig();
  APP_GPIOConfig();

  BSP_USART_Config(115200);
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  mutex = xSemaphoreCreateMutex();
  binary_sema = xSemaphoreCreateBinary();
  empty_sema = xSemaphoreCreateCounting(QUEUE_SIZE, QUEUE_SIZE);
  filled_sema = xSemaphoreCreateCounting(QUEUE_SIZE, 0);

  memset(buf, 0, QUEUE_SIZE * sizeof(uint16_t));

  for (i = 0; i < PRODUCERS; i++)
  {
    printf("Producer %d\n", i);
    if (xTaskCreate(taskProducer, "taskProducer", 128, (void *)&ids[i], 2, NULL) != pdPASS)
    {
      APP_ErrorHandler();
    }
  }

  for (i = 0; i < CONSUMERS; i++)
  {
    printf("Consumer %d\n", i);
    if (xTaskCreate(taskConsumer, "taskConsumer", 128, (void *)&ids[i], 2, NULL) != pdPASS)
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

/**
 * Set system clock to 48MHz
*/
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