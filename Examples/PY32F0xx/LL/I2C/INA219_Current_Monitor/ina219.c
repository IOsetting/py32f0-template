#include "ina219.h"
#include "i2c.h"

uint16_t INA219_Read(uint8_t addr)
{
    uint16_t ret;
    APP_I2C_Receive(INA219_ADDRESS, addr, (uint8_t *)&ret, 2);
    return ((ret & 0xFF00) >> 8)|((ret & 0xFF) << 8);
}

void INA219_Write(uint8_t addr, uint16_t dat)
{
    dat = ((dat & 0xFF00) >> 8)|((dat & 0xFF) << 8);
    APP_I2C_Transmit(INA219_ADDRESS, addr, (uint8_t *)&dat, 2);
}