#include "qmc6309.h"
#include "cordic-math.h"

int16_t Xlow,Xhigh,Ylow,Yhigh;

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

void QMC6309_SetMode(uint8_t mode, uint8_t osr, uint8_t lpf, uint8_t rng, uint8_t odr)
{
    uint8_t ctrl1 = ((lpf & 0x07) << 5) | ((osr & 0x03) << 3) | (mode & 0x03);
    uint8_t ctrl2 = ((odr & 0x07) << 4) | ((rng & 0x03) << 2) | (0x00 & 0x03);
    QMC6309_Write(QMC6309_REG_CONTROL_1, ctrl1);
    QMC6309_Write(QMC6309_REG_CONTROL_2, ctrl2);
    LL_mDelay(100);
}

void QMC6309_Init(void)
{
  QMC6309_Reset();
  QMC6309_SetMode(QMC6309_MODE_CONTINUOUS, QMC6309_OSR_2, QMC6309_LPF_2, QMC6309_RANGE_8G, QMC6309_ODR_100HZ);
}

void QMC6309_ReadAll(int16_t *buf)
{
  uint8_t *buf8b = (uint8_t *)buf;
  BSP_I2cReceive(qmc6309.addr, QMC6309_REG_X_LSB, buf8b, 6);
  BSP_RccNop(1);
}

int32_t QMC6309_Heading(int16_t Xraw, int16_t Yraw, int16_t Zraw)
{
    int16_t X,Y, comp = 0;

    int32_t Heading;
    X = Xraw;
    Y = Yraw;

    // continuously tracks the minimum and maximum values for X and Y axes for automatic calibration and normalization
    if (X < Xlow)
    {
        Xlow = X;
    }
    else if (X > Xhigh)
    {
        Xhigh = X;
    }

    if (Y < Ylow)
    {
        Ylow = Y;
    }
    else if (Y > Yhigh)
    {
        Yhigh = Y;
    }
    if (Xlow == Xhigh || Ylow == Yhigh)
    {
        return 0;
    }
    // subtracts the zero-point offset to compensate for hard iron interference
    // a crucial calibration step that removes the effects of fixed magnetic fields around the sensor
    X -= (Xlow + Xhigh) / 2;
    Y -= (Ylow + Yhigh) / 2;
    // scales the data to a uniform range, eliminates sensitivity differences between axes
    X = (X << 10) / (Xhigh - Xlow);
    Y = (Y << 10) / (Yhigh - Ylow);
    // alculate the heading angle
    if (X < 0)
    {
        if (Y > 0) comp = 180;
        else comp = -180;
    }
    Heading = cordic_atan((1 << CORDIC_MATH_FRACTION_BITS) * Y, (1 << CORDIC_MATH_FRACTION_BITS) * X) / (1 << CORDIC_MATH_FRACTION_BITS);
    // EAST, adds the magnetic declination angle (difference between magnetic north and true north), set according to the geographic location
    Heading += QMC6309_DECLINATION_ANGLE + comp;
    // WEST
    // Heading -= QMC5883L_DECLINATION_ANGLE + comp;

    // ensures the angle stays within a reasonable range (-2π to 2π)
    if (Heading < - 180)
    {
        Heading += 360;
    }
    else if (Heading > 180)
    {
        Heading -= 360;
    }
    return Heading;
}
