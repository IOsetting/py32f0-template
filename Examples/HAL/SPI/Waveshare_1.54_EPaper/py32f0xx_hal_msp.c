/**
  ******************************************************************************
  * @file    py32f0xx_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
  ******************************************************************************
  */

#include "main.h"

extern SPI_HandleTypeDef spi1Handle;

/**
  * @brief  Configure the Flash prefetch and the Instruction cache,
  *         the time base source, NVIC and any required global low level hardware
  *         by calling the HAL_MspInit() callback function from HAL_Init()
  *         
  */
void HAL_MspInit(void)
{
  
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  /*
 * PA0   ------> Reset 
 * PA1   ------> SCL/SCK
 * PA4   ------> Busy
 * PA5   ------> DC/A0
 * PA6   ------> CSN
 * PA7   ------> SDA/MOSI
  */
  /* PA1 SCL/SCK */
  GPIO_InitStruct.Pin       = GPIO_PIN_1;
  if (hspi->Init.CLKPolarity == SPI_POLARITY_LOW)
  {
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  }
  else
  {
    GPIO_InitStruct.Pull = GPIO_PULLUP;
  }
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA7 SDA/MOSI */
  GPIO_InitStruct.Pin       = GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
