#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define I2C_MEMADD_SIZE_8BIT            0x00000001U
#define I2C_MEMADD_SIZE_16BIT           0x00000010U

ErrorStatus I2C_MemoryRead(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
ErrorStatus I2C_MemoryWrite(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);


#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
