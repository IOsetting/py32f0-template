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
#include "py32f0xx_bsp_button.h"
#include "py32f0xx_bsp_led.h"
#include "py32f0xx_bsp_printf.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_LedConfig(void);


int main(void)
{
  /*
  * Configure the Flash prefetch and the Instruction cache,
  * the time base source, NVIC and any required global low level hardware
  * by calling the HAL_MspInit() callback function to be optionally defined in user file
  * PY32F0xx_hal_msp.c.
  */
  HAL_Init();                                 
  
  /* LED GPIO init */
  APP_LedConfig();

  BSP_USART_Config();

  while (1)
  {
    HAL_Delay(1000);                            
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    printf("echo\r\n");
  }
}

static void APP_LedConfig(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();                          /* GPIOB时钟使能 */

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;                    /* 使能上拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;          /* GPIO速度 */

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                /* GPIO初始化 */
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
