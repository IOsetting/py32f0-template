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

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_I2C_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /**
  PF1     ------> I2C1_SCL
  PF0     ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_I2C;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  __HAL_RCC_I2C_FORCE_RESET();
  __HAL_RCC_I2C_RELEASE_RESET();
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
