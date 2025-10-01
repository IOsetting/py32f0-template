#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void BSP_UartReconfig(uint32_t baudRate);

void BSP_UartTxChar(char ch);
void BSP_UartTxHex(uint8_t hex);
void BSP_UartTxHexs(uint8_t *hex, uint8_t size);
void BSP_UartTxString(char *str);
void BSP_UartTxLine(char *str, uint8_t hex);
void BSP_UartTxInt(int32_t num, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
