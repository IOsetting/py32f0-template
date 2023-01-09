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
extern uint32_t   aADCxConvertedData;
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

  /*
   * DMA1_MAP:
   *  00000:ADC       00001:SPI1_TX     00010:SPI1_RX     00011:Reserved
   *  00100:Reserved  00101:USART1_TX   00110:USART1_RX   00111:USART2_TX
   *  01000:USART2_RX 01001:I2C_TX      01010:I2C_RX      01011:TIM1_CH1
   *  01100:TIM1_CH2  01101:TIM1_CH3    01110:TIM1_CH4    01111:TIM1_COM
   *  10000:TIM1_UP   10001:TIM1_TRIG   10010:TIM3_CH1    10011:TIM3_CH3
   *  10100:TIM3_CH4  10101:TIM3_TRG    10110:TIM3_UP     10111:Reserved
   *  11000:TIM16_CH1 11001:TIM16_UP    11010:TIM17_CH1   11011:TIM17_UP
   *  Others:Reserved
  */
  HAL_SYSCFG_DMA_Req(0);                                      /* DMA1_MAP Set to ADC */

  HdmaCh1.Instance                 = DMA1_Channel1;
  HdmaCh1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  HdmaCh1.Init.PeriphInc           = DMA_PINC_DISABLE;
  HdmaCh1.Init.MemInc              = DMA_MINC_DISABLE;
  HdmaCh1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  HdmaCh1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  HdmaCh1.Init.Mode                = DMA_CIRCULAR;
  HdmaCh1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

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
