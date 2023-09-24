#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "py32f0xx_hal.h"

void APP_ErrorHandler(void);
void SPI_TxRxByte(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
