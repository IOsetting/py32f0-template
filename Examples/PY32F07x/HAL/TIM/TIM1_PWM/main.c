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
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define PERIOD    100
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef   TimHandle;
TIM_OC_InitTypeDef  sConfig;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);


int main(void)
{
  uint32_t p1 = 0, p2 = 25, p3 = 50, p4 = 75;

  HAL_Init();
  APP_SystemClockConfig(); 
  
  TimHandle.Instance = TIM1;
  TimHandle.Init.Period            = PERIOD;
  TimHandle.Init.Prescaler         = 800 - 1;
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 1 - 1;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

  sConfig.Pulse = p1;                                               /* CC1=10, duty cycle=10/50=20% */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  sConfig.Pulse = p2;                                               /* CC2=20, duty cycle=20/50=40% */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  sConfig.Pulse = p3;                                               /* CC3=30, duty cycle=30/50=60% */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    APP_ErrorHandler();
  }
  sConfig.Pulse = p4;                                               /* CC4=40, duty cycle=40/50=80% */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  while (1)
  {
    p1 = (p1 + 1) % PERIOD;
    p2 = (p2 + 1) % PERIOD;
    p3 = (p3 + 1) % PERIOD;
    p4 = (p4 + 1) % PERIOD;
    __HAL_TIM_SET_COMPARE(&TimHandle, TIM_CHANNEL_1, p1);
    __HAL_TIM_SET_COMPARE(&TimHandle, TIM_CHANNEL_2, p2);
    __HAL_TIM_SET_COMPARE(&TimHandle, TIM_CHANNEL_3, p3);
    __HAL_TIM_SET_COMPARE(&TimHandle, TIM_CHANNEL_4, p4);
    HAL_Delay(20);
  }
}

static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI \
                                   | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;   /* Set HSI = 8MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  /* RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz; */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  /* RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM; */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  /*RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_NONE;*/
  /*RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;*/
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS;          /* Set HSI as clock source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  /* 0 wait when SYSCLK<=24MHz */
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

void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
