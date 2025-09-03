#ifndef MODULE_UART_H
#define MODULE_UART_H

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


void MOD_UART_Config(uint32_t baudRate);
void MOD_UART_TxChar(char ch);
void MOD_UART_TxHex(uint8_t *hex, uint8_t size);
void MOD_UART_TxString(char *str);


#ifdef __cplusplus
}
#endif

#endif /* MODULE_UART_H */
