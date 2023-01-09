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
#include "py32f0xx_bsp_printf.h"

/* Private define ------------------------------------------------------------*/
#define BUF_SIZE    1024
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void APP_ErrorHandler(void);

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef             AdcHandle;
ADC_ChannelConfTypeDef        sConfig;
TIM_HandleTypeDef             TimHandle;
TIM_OC_InitTypeDef            OCConfig;
TIM_MasterConfigTypeDef       sMasterConfig;
uint32_t   gADCxConvertedData[BUF_SIZE];

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_AdcConfig(void);

int main(void)
{
  HAL_Init();

  BSP_USART_Config();

  APP_AdcConfig();

  while (1)
  {
    if (__HAL_DMA_GET_FLAG(DMA1, DMA_ISR_TCIF1))
    {
      printf("ADC: %ld %ld %ld\r\n", *gADCxConvertedData, *(gADCxConvertedData + 1), *(gADCxConvertedData + 2));
      __HAL_DMA_CLEAR_FLAG(DMA1, DMA_IFCR_CTCIF1);
    }
  }
}

static void APP_AdcConfig(void)
{
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
  __HAL_RCC_ADC_CLK_ENABLE();

  AdcHandle.Instance = ADC1;
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  AdcHandle.Instance = ADC1;
  AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;            /* ADC clock */
  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;                      /* Valid resolution is around 8 bit */
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* Right alignment */
  AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;
  AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  AdcHandle.Init.LowPowerAutoWait = ENABLE;
  AdcHandle.Init.ContinuousConvMode = ENABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* Software trigger */
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                       /* DMA Continuous Mode */
  AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  AdcHandle.Init.SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.Channel = ADC_CHANNEL_0;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  if (HAL_ADC_Start_DMA(&AdcHandle, gADCxConvertedData, BUF_SIZE) != HAL_OK)
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
