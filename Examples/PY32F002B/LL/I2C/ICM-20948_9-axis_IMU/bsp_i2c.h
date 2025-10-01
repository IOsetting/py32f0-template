#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void BSP_I2cTransmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t size);
void BSP_I2cReceive(uint16_t devAddress, uint16_t memAddress, uint8_t *buf, uint16_t size);


#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_H */
