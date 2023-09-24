/**
 * Example of Binary Semaphore
 * 
 * Note: Require RAM > 6K Byte
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

TaskStatus_t xTaskDetails;
char buff[512];

SemaphoreHandle_t xBinarySemaphore;
volatile uint32_t val;

static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);
static void APP_TIM1Config(void);

void taskUART(void *pvParameters)
{
  (void)(pvParameters);

  while (1)
  {
    // Block till being notified by TIM1 Interrupt
    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
    /**
     * Use printf() will take more stack size
     * printf("%lu Name  State  Priority Stack Num\r\n%s", val, buff);
    */
    BSP_UART_TxHex32(val);
    BSP_UART_TxString(" Name  State  Priority Stack Num\r\n");
    BSP_UART_TxString(buff);
  }
}

void taskInfo(void *pvParameters)
{
  (void)(pvParameters);

  while (1)
  {
    vTaskList(buff);
    vTaskDelay(300);
  }
}

int main(void)
{
  BaseType_t xReturned;

  APP_SystemClockConfig();
  APP_GPIOConfig();

  BSP_USART_Config(115200);
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  xBinarySemaphore = xSemaphoreCreateBinary();
  if (xBinarySemaphore == NULL)
  {
    APP_ErrorHandler();
  }

  xReturned = xTaskCreate(taskUART, "taskUART", 160, NULL, 2, NULL); 
  if (xReturned != pdPASS)
  {
      APP_ErrorHandler();
  }

  xReturned = xTaskCreate(taskInfo, "taskInfo", 160, NULL, 2, NULL); 
  if (xReturned != pdPASS)
  {
      APP_ErrorHandler();
  }

  APP_TIM1Config();

  printf("FreeRTOS Scheduler starting...\r\n");
  /* Start the scheduler. */
  vTaskStartScheduler();

  /* Will only get here if there was not enough heap space to create the idle task. */
  return 0;
}

static void APP_TIM1Config(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(RCC_APBENR2_TIM1EN);
  
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler           = 8000-1;
  TIM1CountInit.Autoreload          = 6000-1;
  TIM1CountInit.RepetitionCounter   = 0;
  LL_TIM_Init(TIM1,&TIM1CountInit);

  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);
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

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) && LL_TIM_IsEnabledIT_UPDATE(TIM1))
  {
      // Clear interrupt flag
      LL_TIM_ClearFlag_UPDATE(TIM1);
      // Increase the value
      val++;
      // Give the semaphore so that taskUART can proceed
      xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void APP_ErrorHandler(void)
{
  while (1);
}