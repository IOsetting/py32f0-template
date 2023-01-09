/**
  ******************************************************************************
  * @file    py32f0xx_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static DMA_HandleTypeDef HdmaCh1;
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
  * @brief  Configure the Flash prefetch and the Instruction cache,
  *         the time base source, NVIC and any required global low level hardware
  *         by calling the HAL_MspInit() callback function from HAL_Init()
  *         
  */
void HAL_MspInit(void)
{
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_DMA_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
 
  /* ADC Channel: PA0   */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_SYSCFG_DMA_Req(DMA_CHANNEL_MAP_ADC);            /* DMA1_MAP Set to ADC */

  HdmaCh1.Instance                 = DMA1_Channel1;
  HdmaCh1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  HdmaCh1.Init.PeriphInc           = DMA_PINC_DISABLE;
  HdmaCh1.Init.MemInc              = DMA_MINC_ENABLE;
  HdmaCh1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  HdmaCh1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  HdmaCh1.Init.Mode                = DMA_CIRCULAR;
  HdmaCh1.Init.Priority            = DMA_PRIORITY_HIGH;

  HAL_DMA_DeInit(&HdmaCh1);
  HAL_DMA_Init(&HdmaCh1);
  __HAL_LINKDMA(hadc, DMA_Handle, HdmaCh1);
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
  __HAL_RCC_DMA_FORCE_RESET();
  __HAL_RCC_DMA_RELEASE_RESET();

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
