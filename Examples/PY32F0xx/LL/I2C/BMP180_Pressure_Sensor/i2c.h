#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

ErrorStatus APP_I2C_TestAddress(uint8_t dev_addr);
int8_t APP_I2C_Transmit(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint8_t len);
int8_t APP_I2C_Receive(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
