#include "py32f0xx_bsp_led.h"


GPIO_TypeDef* LED_PORT[LEDn] = {LED3_GPIO_PORT};
const uint16_t LED_PIN[LEDn] = {LED3_PIN};


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
