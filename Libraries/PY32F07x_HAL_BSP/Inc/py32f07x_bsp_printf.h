/**
  ******************************************************************************
  * @file    py32f07x_bsp_printf.h
  * @author  MCU Application Team
  * @brief   
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F07X_BSP_PRINTF_H
#define PY32F07X_BSP_PRINTF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "py32f07x_hal.h"


#ifdef HAL_UART_MODULE_ENABLED
//debug printf redirect config
#define DEBUG_USART_BAUDRATE                    115200

#define DEBUG_USART                             USART2
#define DEBUG_USART_CLK_ENABLE()                __HAL_RCC_USART2_CLK_ENABLE()

#define DEBUG_USART_RX_GPIO_PORT                GPIOA
#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_RX_PIN                      GPIO_PIN_3
#define DEBUG_USART_RX_AF                       GPIO_AF1_USART2

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_TX_PIN                      GPIO_PIN_2
#define DEBUG_USART_TX_AF                       GPIO_AF1_USART2

#define DEBUG_USART_IRQHandler                  USART2_IRQHandler
#define DEBUG_USART_IRQ                         USART2_IRQn

extern UART_HandleTypeDef DebugUartHandle;
#endif

void             BSP_USART_Config(void);


#ifdef __cplusplus
}
#endif

#endif /* PY32F0XX_BSP_PRINTF_H */
