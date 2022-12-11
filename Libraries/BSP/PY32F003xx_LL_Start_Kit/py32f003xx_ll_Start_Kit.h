/**
  ******************************************************************************
  * @file    py32f003xx_ll_Start_Kit.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F0XX_LL_START_KIT_H
#define PY32F0XX_LL_START_KIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "py32f0xx_ll_rcc.h"
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_system.h"
#include "py32f0xx_ll_exti.h"
#include "py32f0xx_ll_cortex.h"
#include "py32f0xx_ll_utils.h"
#include "py32f0xx_ll_pwr.h"
#include "py32f0xx_ll_dma.h"
#include "py32f0xx_ll_gpio.h"
#include "py32f0xx_ll_usart.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup py32f0xx_Start_Kit
  * @brief This section contains the exported types, contants and functions
  *        required to use the Nucleo 32 board.
  * @{
  */

/** @defgroup py32f0xx_Start_Kit_Exported_Types Exported Types
  * @{
  */
typedef enum
{
  LED3 = 0,
  LED_GREEN = LED3
} Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY  = BUTTON_USER
} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

#define LEDn                               1

#define LED3_PIN                           LL_GPIO_PIN_5
#define LED3_GPIO_PORT                     GPIOB
#define LED3_GPIO_CLK_ENABLE()             LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB)
#define LED3_GPIO_CLK_DISABLE()            LL_IOP_GRP1_DisableClock(LL_IOP_GRP1_PERIPH_GPIOB)

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)    do {LED3_GPIO_CLK_ENABLE(); } while(0U)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)   LED3_GPIO_CLK_DISABLE())

#define BUTTONn                            1

/**
  * @brief User push-button
  */
#define USER_BUTTON_PIN                         LL_GPIO_PIN_12
#define USER_BUTTON_GPIO_PORT                   GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()           LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA)
#define USER_BUTTON_GPIO_CLK_DISABLE()          LL_IOP_GRP1_DisableClock(LL_IOP_GRP1_PERIPH_GPIOA)
#define USER_BUTTON_EXTI_IRQn                   EXTI4_15_IRQn
#define USER_BUTTON_EXTI_LINE                   LL_EXTI_LINE_12
#define USER_BUTTON_EXTI_LINE_ENABLE()          LL_EXTI_EnableIT(USER_BUTTON_EXTI_LINE)
#define USER_BUTTON_EXTI_FALLING_TRIG_ENABLE()  LL_EXTI_EnableFallingTrig(USER_BUTTON_EXTI_LINE)
#define USER_BUTTON_IRQHANDLER                  EXTI4_15_IRQHandler

/* Aliases */
#define KEY_BUTTON_PIN                          USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT                    USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()            USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()           USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn                    USER_BUTTON_EXTI_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == 0) USER_BUTTON_GPIO_CLK_ENABLE();} while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)     (((__INDEX__) == 0) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0)


//debug printf redirect config
#define DEBUG_USART_BAUDRATE                    115200

#define DEBUG_USART                             USART2
#define DEBUG_USART_CLK_ENABLE()                LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2)

#define __GPIOA_CLK_ENABLE()                    do { \
                                                     __IO uint32_t tmpreg = 0x00U; \
                                                     SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);\
                                                     /* Delay after an RCC peripheral clock enabling */ \
                                                     tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);\
                                                     UNUSED(tmpreg); \
                                                   } while(0U)

#define DEBUG_USART_RX_GPIO_PORT                GPIOA
#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA)
#define DEBUG_USART_RX_PIN                      LL_GPIO_PIN_3
#define DEBUG_USART_RX_AF                       LL_GPIO_AF_4

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA)
#define DEBUG_USART_TX_PIN                      LL_GPIO_PIN_2
#define DEBUG_USART_TX_AF                       LL_GPIO_AF_4

#define DEBUG_USART_IRQHandler                  USART2_IRQHandler
#define DEBUG_USART_IRQ                         USART2_IRQn
/************************************************************/

/** @defgroup Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);

void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);

void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);

void             BSP_USART_Config(void);
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* PY32F003XX_LL_START_KIT_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
