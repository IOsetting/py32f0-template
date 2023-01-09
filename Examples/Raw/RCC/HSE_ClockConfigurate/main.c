/**
  ******************************************************************************
  * Switch System Clock Source To HSE(High Speed External)
  * 
  * -- Edit py32f0xx_hal_conf.h, change HSE_VALUE according to the frequency
  *    of your crystal oscillator.
  * -- If the frequency is incorrect, the printf output will be garbled.
  * -- If the crystal oscillator is not working or not connected, the LED stops
  *    blink.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_hal.h"
#include "py32f0xx_bsp_printf.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void APP_ErrorHandler(void);

static void APP_SystemClockConfig(void);

static void APP_LedConfig(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int main(void)
{
  HAL_Init();

  /* Switch system clock to HSE */
  APP_SystemClockConfig();

  APP_LedConfig();

  BSP_USART_Config();

  while (1)
  {
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    printf("Clock: %ld \r\n", SystemCoreClock);
  }
}

static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                                                     /* Turn on HSE */
  RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;                                                /* HSE frequency range */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;                                        /* SYSCLK source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  /* 
   * Re-initialize RCC clock
   * -- clock <= 24MHz: FLASH_LATENCY_0
   * -- clock > 24MHz:  FLASH_LATENCY_1
   */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
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
