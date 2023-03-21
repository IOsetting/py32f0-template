#include <stdio.h>
#include "bsp_i2c.h"


#define I2C_SELF_ADDRESS    0xA0     /* host address */
#define I2C_MAX_TIMEOUT     0x2000
#define I2C_STATE_READY     0
#define I2C_STATE_BUSY_TX   1
#define I2C_STATE_BUSY_RX   2

__IO uint32_t   i2cState  = I2C_STATE_READY;


void BSP_I2C_Config(void)
{
  LL_I2C_InitTypeDef I2C_InitStruct;

  LL_I2C_DeInit(I2C1);
  /* 
   * Clock speed:
   * - standard = 100khz, if PLL is on, set system clock <= 16MHz, or I2C might not work
   * - fast     = 400khz
  */
  I2C_InitStruct.ClockSpeed      = LL_I2C_MAX_SPEED_FAST;
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1     = I2C_SELF_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  LL_I2C_Init(I2C1, &I2C_InitStruct);

  /* Enale clock stretch (reset default: on) */
  // LL_I2C_EnableClockStretching(I2C1);

  /* Enable general call (reset default: off) */
  // LL_I2C_EnableGeneralCall(I2C1);
}


void BSP_I2C_Scan(void)
{
  for(uint16_t i = 0; i < 128; i++) 
  {
    if(BSP_I2C_IsDeviceReady(i << 1, I2C_MAX_TIMEOUT) == SUCCESS)
    {
      //printf("Found address: 0x%02X\r\n", i << 1);
    }
  }
}

ErrorStatus BSP_I2C_IsDeviceReady(uint8_t devAddress, uint16_t timeout)
{
  uint16_t t = timeout;
  while (i2cState == I2C_STATE_BUSY_TX && t--);
  if (t == 0) return ERROR;
  t = timeout;

  LL_I2C_DisableBitPOS(I2C1);

  i2cState = I2C_STATE_BUSY_TX;

  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1 && t--);
  if (t == 0) 
  {
    i2cState  = I2C_STATE_READY;
    return ERROR;
  }
  t = timeout;

  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  while (LL_I2C_IsActiveFlag_ADDR(I2C1) != 1 && t--)
  if (t == 0) 
  {
    i2cState  = I2C_STATE_READY;
    return ERROR;
  }
  t = timeout;

  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);

  i2cState = I2C_STATE_READY;
  return SUCCESS;
}

ErrorStatus BSP_I2C_MasterTransmit(uint16_t devAddress, uint8_t *pData, uint16_t len, uint16_t timeout)
{
  uint16_t t = timeout;

  while (i2cState == I2C_STATE_BUSY_TX && t--)
  if (t == 0) return ERROR;
  t = timeout;

  LL_I2C_DisableBitPOS(I2C1);
  i2cState = I2C_STATE_BUSY_TX;

  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1 && t--)
  if (t == 0) 
  {
    i2cState  = I2C_STATE_READY;
    return ERROR;
  }
  t = timeout;

  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  while (LL_I2C_IsActiveFlag_ADDR(I2C1) != 1 && t--)
  if (t == 0) 
  {
    i2cState  = I2C_STATE_READY;
    return ERROR;
  }
  t = timeout;

  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Transfer data */
  while (len > 0)
  {
    while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1 && t--)
    if (t == 0) 
    {
      i2cState  = I2C_STATE_READY;
      return ERROR;
    }
    t = timeout;

    LL_I2C_TransmitData8(I2C1, *pData++);
    len--;

    while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1 && t--)
    if (t == 0) 
    {
      i2cState  = I2C_STATE_READY;
      return ERROR;
    }
    t = timeout;
  }

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);

  i2cState = I2C_STATE_READY;
  return SUCCESS;
}

ErrorStatus BSP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t len, uint16_t timeout)
{
  uint16_t t = timeout;

  while (i2cState == I2C_STATE_BUSY_TX && t--)
  if (t == 0) return ERROR;
  t = timeout;

  LL_I2C_DisableBitPOS(I2C1);

  i2cState = I2C_STATE_BUSY_TX;

  /* Start */
  LL_I2C_GenerateStartCondition(I2C1);
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1 && t--)
  if (t == 0) 
  {
    i2cState  = I2C_STATE_READY;
    return ERROR;
  }
  t = timeout;
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  while (LL_I2C_IsActiveFlag_ADDR(I2C1) != 1 && t--)
  if (t == 0) 
  {
    i2cState  = I2C_STATE_READY;
    return ERROR;
  }
  t = timeout;
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, memAddress);
  while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1 && t--)
  if (t == 0) 
  {
    i2cState  = I2C_STATE_READY;
    return ERROR;
  }
  t = timeout;

  /* Transfer data */
  while (len > 0)
  {
    while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1 && t--)
    if (t == 0) 
    {
      i2cState  = I2C_STATE_READY;
      return ERROR;
    }
    t = timeout;

    LL_I2C_TransmitData8(I2C1, *pData++);
    len--;

    if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (len != 0U))
    {
      LL_I2C_TransmitData8(I2C1, *pData++);
      len--;
    }

    while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1 && t--)
    if (t == 0) 
    {
      i2cState  = I2C_STATE_READY;
      return ERROR;
    }
    t = timeout;
  }

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);
  i2cState = I2C_STATE_READY;
  return SUCCESS;
}
