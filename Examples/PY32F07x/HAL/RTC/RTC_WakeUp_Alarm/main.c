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
/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef RTCinit;
RTC_TimeTypeDef RTCtime;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_RtcInit(void);
static void APP_RtcSetAlarm_IT(uint8_t Sec, uint8_t Min, uint8_t Hour);

int main(void)
{
  HAL_Init();

  APP_SystemClockConfig();

  BSP_USART_Config();

  APP_RtcInit();
  /* SWD port still works within this second */
  HAL_Delay(1000);
  printf("Stop system tick\r\n");
  HAL_SuspendTick();

  while (1)
  {
    /* Waits until the RTC_CNT, RTC_ALR and RTC_PRL are synchronized with RTC APB clock.*/
    HAL_RTC_WaitForSynchro(&RTCinit);
    HAL_RTC_GetTime(&RTCinit, &RTCtime, RTC_FORMAT_BIN);
    /* Set Alarm (will be triggered 1 second later) */
    APP_RtcSetAlarm_IT(RTCtime.Seconds, RTCtime.Minutes, RTCtime.Hours);
    /* Enter stop mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  }
}

static void APP_RtcInit(void)
{
  RTC_TimeTypeDef Timeinit;
  
  RTCinit.Instance = RTC;
  RTCinit.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  /*2022-1-1-00:00:00*/
  RTCinit.DateToUpdate.Year = 22;
  RTCinit.DateToUpdate.Month = RTC_MONTH_JANUARY;
  RTCinit.DateToUpdate.Date = 1;
  RTCinit.DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
  Timeinit.Hours = 0x00;
  Timeinit.Minutes = 0x00;
  Timeinit.Seconds = 0x00;

  HAL_RTC_DeInit(&RTCinit);
  HAL_RTC_Init(&RTCinit);
  
  HAL_RTC_SetTime(&RTCinit, &Timeinit, RTC_FORMAT_BIN);
}

static void APP_RtcSetAlarm_IT(uint8_t Sec, uint8_t Min, uint8_t Hour)
{
  RTC_AlarmTypeDef Alarminit;
  RTCinit.Instance = RTC;
  Alarminit.AlarmTime.Hours = Hour;
  Alarminit.AlarmTime.Minutes = Min;
  Alarminit.AlarmTime.Seconds = Sec;
  HAL_RTC_SetAlarm_IT(&RTCinit, &Alarminit, RTC_FORMAT_BIN);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  printf("%02d:%02d:%02d\r\n", RTCtime.Hours, RTCtime.Minutes, RTCtime.Seconds);
}

static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  /*RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  /*RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_NONE;*/
  /*RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;*/
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
