/**
  ******************************************************************************
  * @file    py32f0xx_bsp_printf.h
  * @author  MCU Application Team
  * @brief   
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F003_BSP_PRINTF_H
#define PY32F003_BSP_PRINTF_H

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


//debug printf redirect config
#define DEBUG_USART                             USART1
#define DEBUG_USART_CLK_ENABLE()                LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1)

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
#define DEBUG_USART_RX_AF                       LL_GPIO_AF_1

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA)
#define DEBUG_USART_TX_PIN                      LL_GPIO_PIN_2
#define DEBUG_USART_TX_AF                       LL_GPIO_AF_1

#define DEBUG_USART_IRQHandler                  USART1_IRQHandler
#define DEBUG_USART_IRQ                         USART1_IRQn

/************************************************************/

void            BSP_USART_Config(uint32_t baudRate);

void            BSP_UART_TxChar(char ch);
void            BSP_UART_TxHex8(uint8_t hex);
void            BSP_UART_TxHex16(uint16_t hex);
void            BSP_UART_TxHex32(uint32_t hex);
void            BSP_UART_TxString(char *str);

#ifdef __cplusplus
}
#endif

#endif /* PY32F003_BSP_PRINTF_H */
