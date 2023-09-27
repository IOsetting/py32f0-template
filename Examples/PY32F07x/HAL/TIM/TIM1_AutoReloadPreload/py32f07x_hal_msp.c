/**
  ******************************************************************************
  * @file    py32f07x_hal_msp.c
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
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
  * @brief 初始化全局MSP
  */
void HAL_MspInit(void)
{
}

/**
  * @brief 初始TIM相关MSP
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /* 使能TIM1时钟 */
  __HAL_RCC_TIM1_CLK_ENABLE();
  
  /* 设置中断优先级 */
  HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0, 0);
  /* 使能TIM1中断 */
  HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}
