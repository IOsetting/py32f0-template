#include "bsp_i2c.h"

#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2

#ifdef __cplusplus
extern "C" {
#endif

__IO static uint32_t i2cState  = I2C_STATE_READY;

void BSP_I2cTransmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t size)
{
  uint16_t timeout;

  while (i2cState != I2C_STATE_READY);
  i2cState = I2C_STATE_BUSY_TX;
  /* Disable Pos */
  LL_I2C_DisableBitPOS(I2C1);

  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  /* Wait until ADDR flag is set */
  timeout = 0xFFFF;
  while(--timeout && LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  /* Clear ADDR flag */
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Wait until TXE flag is set */
  timeout = 0xFFFF;
  while(--timeout && LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, memAddress);

  /* Transfer data */
  while (size > 0)
  {
    timeout = 0xFFFF;
    while (--timeout && LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
    LL_I2C_TransmitData8(I2C1, *pData++);
    size--;

    if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (size != 0U))
    {
      LL_I2C_TransmitData8(I2C1, *pData++);
      size--;
    }
    timeout = 0xFFFF;
    while (--timeout && LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
  }

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);

  i2cState = I2C_STATE_READY;
  if (!timeout) 
  {
    APP_I2cReconfig();
  }
}

void BSP_I2cReceive(uint16_t devAddress, uint16_t memAddress, uint8_t *buf, uint16_t size)
{
  uint16_t timeout;

  while (i2cState != I2C_STATE_READY);
  i2cState    = I2C_STATE_BUSY_RX;
  /* Disable Pos */
  LL_I2C_DisableBitPOS(I2C1);

  /***** Send device address + memory address *****/

  /* Enable Acknowledge */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);

  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress & (uint8_t)(~0x01)));
  /* Wait until ADDR flag is set */
  timeout = 0xFFFF;
  while(--timeout && LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);

  /* Clear ADDR flag */
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Wait until TXE flag is set */
  timeout = 0xFFFF;
  while (--timeout && LL_I2C_IsActiveFlag_TXE(I2C1) != 1);

  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, (uint8_t)(memAddress & 0x00FF));
  timeout = 0xFFFF;
  while (--timeout && LL_I2C_IsActiveFlag_BTF(I2C1) != 1);

  /***** Restart to read *****/

  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  timeout = 0xFFFF;
  while(--timeout && LL_I2C_IsActiveFlag_SB(I2C1) != 1);

  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (devAddress | 0x1));
  /* Wait until ADDR flag is set */
  timeout = 0xFFFF;
  while(--timeout && LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);

  if (size == 0U)
  {
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_GenerateStopCondition(I2C1);
  }
  else if(size == 1U)
  {
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

    __disable_irq();
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_GenerateStopCondition(I2C1);
    __enable_irq();
  }
  else if(size == 2U)
  {
    LL_I2C_EnableBitPOS(I2C1);

    __disable_irq();
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
    __enable_irq();
  }
  else
  {
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
    LL_I2C_ClearFlag_ADDR(I2C1);
  }

  while (size > 0U)
  {
    if (size <= 3U)
    {
      if (size == 1U)
      {
        timeout = 0xFFFF;
        while(--timeout && LL_I2C_IsActiveFlag_RXNE(I2C1) != 1);
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
      }
      else if (size == 2U)
      {
        timeout = 0xFFFF;
        while(--timeout && LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        __disable_irq();
        LL_I2C_GenerateStopCondition(I2C1);
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
        __enable_irq();
        
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
      }
      else
      {
        timeout = 0xFFFF;
        while(--timeout && LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
        
        __disable_irq();
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
        timeout = 0xFFFF;
        while(--timeout && LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        LL_I2C_GenerateStopCondition(I2C1);
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
        __enable_irq();
        
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
      }
    }
    else
    {
      timeout = 0xFFFF;
      while(--timeout && LL_I2C_IsActiveFlag_RXNE(I2C1) != 1);
      *buf++ = LL_I2C_ReceiveData8(I2C1);
       size--;
      
      if (LL_I2C_IsActiveFlag_BTF(I2C1) == 1)
      {
        *buf++ = LL_I2C_ReceiveData8(I2C1);
        size--;
      }
    }
  }
  i2cState = I2C_STATE_READY;
  if (!timeout) 
  {
    APP_I2cReconfig();
  }
}


#ifdef __cplusplus
}
#endif
