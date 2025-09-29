#include "qmc6309.h"

static QMC6309_t qmc6309;


uint8_t QMC6309_Read(uint8_t addr)
{
  uint8_t ret;
  BSP_I2cReceive(qmc6309.addr, addr, &ret, 1);
  BSP_RccNop(1);
  return ret;
}

void QMC6309_ReadInBatch(uint8_t addr, uint8_t *buf, uint8_t size)
{
  BSP_I2cReceive(qmc6309.addr, addr, buf, size);
  BSP_RccNop(1);
}

void QMC6309_Write(uint8_t addr, uint8_t dat)
{
  BSP_I2cTransmit(qmc6309.addr, addr, &dat, 1);
  BSP_RccNop(1);
}

ErrorStatus QMC6309_Detect(uint8_t address)
{
  qmc6309.addr = address;
  if (QMC6309_Read(QMC6309_REG_CHIP_ID) == QMC6309_CHIP_ID)
  {
      return SUCCESS;
  }
  return ERROR;
}

void QMC6309_Reset(void) 
{
  QMC6309_Write(QMC6309_REG_CONTROL_2, 0x80);
  LL_mDelay(20);
  QMC6309_Write(QMC6309_REG_CONTROL_2, 0x00);
  LL_mDelay(20);
}

void QMC6309_SetMode(uint8_t mode, uint8_t odr)
{
  uint8_t osr = 0b11;
  uint8_t rng = 0b01;
  uint8_t ctrl1 = (osr << 6) | (rng << 4) | ((odr & 0x03) << 2) | (mode & 0x03);
  QMC6309_Write(QMC6309_REG_CONTROL_1, ctrl1);
  QMC6309_Write(QMC6309_REG_CONTROL_2, 0x03);
  LL_mDelay(100);
}

void QMC6309_Init(void)
{
  QMC6309_Reset();
  QMC6309_SetMode(QMC6309_MODE_SUSPEND, QMC6309_ODR_200HZ);
  QMC6309_SetMode(QMC6309_MODE_CONTINUOUS, QMC6309_ODR_200HZ);
}

void QMC6309_ReadAll(int16_t *buf)
{
  uint8_t tmp, *buf8b = (uint8_t *)buf;
  BSP_I2cReceive(qmc6309.addr, QMC6309_REG_X_LSB, buf8b, 6);
  BSP_RccNop(1);
  tmp = buf8b[0]; buf8b[0] = buf8b[1]; buf8b[1] = tmp;
  tmp = buf8b[2]; buf8b[2] = buf8b[3]; buf8b[3] = tmp;
  tmp = buf8b[4]; buf8b[4] = buf8b[5]; buf8b[5] = tmp;
}
