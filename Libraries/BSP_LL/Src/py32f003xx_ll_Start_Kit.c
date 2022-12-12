/**
  ******************************************************************************
  * @file    py32f003xx_ll_Start_Kit.c
  * @author  MCU Application Team
  * @brief   This file provides set of firmware functions to manage Leds, 
  *          push-button available on Start Kit.
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
#include "py32f003xx_ll_Start_Kit.h"

/**
  * @brief PY32F003xx STK BSP Driver version number
  */
#define __PY32F003xx_STK_BSP_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __PY32F003xx_STK_BSP_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __PY32F003xx_STK_BSP_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define __PY32F003xx_STK_BSP_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __PY32F003xx_STK_BSP_VERSION        ((__PY32F003xx_STK_BSP_VERSION_MAIN << 24) \
                                            |(__PY32F003xx_STK_BSP_VERSION_SUB1 << 16) \
                                            |(__PY32F003xx_STK_BSP_VERSION_SUB2 << 8 ) \
                                            |(__PY32F003xx_STK_BSP_VERSION_RC))

GPIO_TypeDef* LED_PORT[LEDn] = {LED3_GPIO_PORT};
const uint16_t LED_PIN[LEDn] = {LED3_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT };
const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN };
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn };
const uint32_t BUTTON_EXTI_LINE[BUTTONn] = {USER_BUTTON_EXTI_LINE };

/** @addtogroup PY32F003xx_STK_Exported_Functions
  * @{
  */

/**
  * @brief  This method returns the PY32F003 STK BSP Driver revision.
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __PY32F003xx_STK_BSP_VERSION;
}

/** @addtogroup LED_Functions
  * @{
  */

/**
  * @brief  Configures LED GPIO.
  * @param  Led Specifies the Led to be configured.
  *         This parameter can be one of following parameters:
  *         @arg  LED3
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  LL_GPIO_SetPinMode(LED_PORT[Led], LED_PIN[Led], LL_GPIO_MODE_OUTPUT);
  /* LL_GPIO_SetPinOutputType(LED_PORT[Led], LED_PIN[Led], LL_GPIO_OUTPUT_PUSHPULL); */
  /* LL_GPIO_SetPinSpeed(LED_PORT[Led], LED_PIN[Led], LL_GPIO_SPEED_FREQ_LOW);       */
  /* LL_GPIO_SetPinPull(LED_PORT[Led], LED_PIN[Led], LL_GPIO_PULL_NO);               */

  LL_GPIO_SetOutputPin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  DeInitialize LED GPIO.
  * @param  Led Specifies the Led to be deconfigured.
  *         This parameter can be one of the following values:
  *         @arg  LED3
  * @note BSP_LED_DeInit() does not disable the GPIO clock
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  /* Turn off LED */
  LL_GPIO_ResetOutputPin(LED_PORT[Led], LED_PIN[Led]);
  /* DeInit the GPIO_LED pin */
  LL_GPIO_SetPinMode(LED_PORT[Led], LED_PIN[Led], LL_GPIO_MODE_ANALOG);
  /* LL_GPIO_SetPinOutputType(LED_PORT[Led], LED_PIN[Led], LL_GPIO_OUTPUT_PUSHPULL); */
  /* LL_GPIO_SetPinSpeed(LED_PORT[Led], LED_PIN[Led], LL_GPIO_SPEED_FREQ_LOW);       */
  /* LL_GPIO_SetPinPull(LED_PORT[Led], LED_PIN[Led], LL_GPIO_PULL_NO);               */
}

/**
  * @brief  Turns selected LED On.
  * @param  Led Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *   @arg  LED3
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  LL_GPIO_ResetOutputPin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  LL_GPIO_SetOutputPin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  LL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_PULL_NO);
  /* LL_GPIO_SetPinSpeed(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_SPEED_FREQ_HIGH); */

  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    LL_EXTI_EnableIT(BUTTON_EXTI_LINE[Button]);
    LL_EXTI_EnableFallingTrig(BUTTON_EXTI_LINE[Button]);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F);
    NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  Push Button DeInit.
  * @param  Button: Button to be configured
  *   This parameter should be: BUTTON_USER
  * @note PB DeInit does not disable the GPIO clock
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  LL_GPIO_SetPinMode(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_MODE_ANALOG);
  /* LL_GPIO_SetPinSpeed(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_SPEED_FREQ_LOW); */
  /* LL_GPIO_SetPinPull(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_PULL_NO);         */
  /* LL_GPIO_SetAFPin_8_15(BUTTON_PORT[Button], BUTTON_PIN[Button], LL_GPIO_AF_0);         */
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval Button state.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return LL_GPIO_IsInputPinSet(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}


//#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
