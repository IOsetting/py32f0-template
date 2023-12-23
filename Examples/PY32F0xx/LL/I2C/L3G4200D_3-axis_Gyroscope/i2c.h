#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void APP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t size);
void APP_I2C_Receive(uint16_t devAddress, uint16_t memAddress, uint8_t *buf, uint16_t size);


#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
