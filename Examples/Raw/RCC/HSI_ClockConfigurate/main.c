/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_hal.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);

void APP_ErrorHandler(void);

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

  /* Reconfig system clock */
  APP_SystemClockConfig();

  APP_LedConfig();

  while (1)
  {
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
  }
}

static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
                                    | RCC_OSCILLATORTYPE_HSI
                                    | RCC_OSCILLATORTYPE_LSE
                                    | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                            /* HSI ON */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;   /* Set HSI clock 24MHz */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                            /* No division */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                           /* OFF */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                           /* OFF */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                           /* OFF */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                       /* OFF */

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Reinitialize AHB,APB bus clock */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                                | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;              /* Select HSI as SYSCLK source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                  /* APH clock, no division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                   /* APB clock, no division */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

void APP_ErrorHandler(void)
{
  while (1)
  {
  }
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
