#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "py32f002b_ll_rcc.h"
#include "py32f002b_ll_bus.h"
#include "py32f002b_ll_system.h"
#include "py32f002b_ll_exti.h"
#include "py32f002b_ll_cortex.h"
#include "py32f002b_ll_utils.h"
#include "py32f002b_ll_pwr.h"
#include "py32f002b_ll_gpio.h"
#include "py32f002b_ll_usart.h"


void BSP_UartConfig(uint32_t baudRate);
void BSP_UartTxChar(char ch);
void BSP_UartTxHex(uint8_t *hex, uint8_t size);
void BSP_UartTxString(char *str);
void BSP_UartTxInt(int32_t num, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
