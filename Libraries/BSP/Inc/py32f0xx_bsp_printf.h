/**
  ******************************************************************************
  * @file    py32f0xx_bsp_printf.h
  * @author  MCU Application Team
  * @brief   
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F0XX_BSP_PRINTF_H
#define PY32F0XX_BSP_PRINTF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "py32f0xx_hal.h"



#ifdef HAL_UART_MODULE_ENABLED
//debug printf redirect config
#define DEBUG_USART_BAUDRATE                    115200

#define DEBUG_USART                             USART1
#define DEBUG_USART_CLK_ENABLE()                do { \
                                                     __IO uint32_t tmpreg = 0x00U; \
                                                     SET_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);\
                                                     /* Delay after an RCC peripheral clock enabling */ \
                                                     tmpreg = READ_BIT(RCC->APBENR2, RCC_APBENR2_USART1EN);\
                                                     UNUSED(tmpreg); \
                                                   } while(0U)

#define __GPIOA_CLK_ENABLE()                    do { \
                                                     __IO uint32_t tmpreg = 0x00U; \
                                                     SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);\
                                                     /* Delay after an RCC peripheral clock enabling */ \
                                                     tmpreg = READ_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);\
                                                     UNUSED(tmpreg); \
                                                   } while(0U)

#define DEBUG_USART_RX_GPIO_PORT                GPIOA
#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_RX_PIN                      GPIO_PIN_3
#define DEBUG_USART_RX_AF                       GPIO_AF1_USART1

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_TX_PIN                      GPIO_PIN_2
#define DEBUG_USART_TX_AF                       GPIO_AF1_USART1

#define DEBUG_USART_IRQHandler                  USART1_IRQHandler
#define DEBUG_USART_IRQ                         USART1_IRQn

extern UART_HandleTypeDef DebugUartHandle;
#endif

void             BSP_USART_Config(void);


#ifdef __cplusplus
}
#endif

#endif /* PY32F0XX_BSP_PRINTF_H */
