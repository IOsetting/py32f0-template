#include "i2c.h"

#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2

#ifdef __cplusplus
extern "C" {
#endif

__IO static uint32_t i2cState  = I2C_STATE_READY;


ErrorStatus APP_I2C_TestAddress(uint8_t dev_addr)
{
  uint16_t timeout = 0xFFF;
  while (i2cState != I2C_STATE_READY);
  i2cState = I2C_STATE_BUSY_TX;
  /* Disable Pos */
  LL_I2C_DisableBitPOS(I2C1);
  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (dev_addr & (uint8_t)(~0x01)));
  /* Wait until ADDR flag is set */
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1)
  {
    timeout--;
    if (timeout == 0) break;
  }
  if (timeout == 0)
  {
    i2cState = I2C_STATE_READY;
    return ERROR;
  }
  else
  {
    /* Clear ADDR flag */
    LL_I2C_ClearFlag_ADDR(I2C1);
    /* Stop */
    LL_I2C_GenerateStopCondition(I2C1);

    i2cState = I2C_STATE_READY;
    return SUCCESS;
  }
}

int8_t APP_I2C_Transmit(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint8_t len)
{
  while (i2cState != I2C_STATE_READY);
  i2cState = I2C_STATE_BUSY_TX;
  /* Disable Pos */
  LL_I2C_DisableBitPOS(I2C1);

  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  while (LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (dev_addr & (uint8_t)(~0x01)));
  /* Wait until ADDR flag is set */
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  /* Clear ADDR flag */
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Wait until TXE flag is set */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, reg_addr);

  /* Transfer data */
  while (len > 0)
  {
    while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
    LL_I2C_TransmitData8(I2C1, *read_data++);
    len--;

    if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (len != 0U))
    {
      LL_I2C_TransmitData8(I2C1, *read_data++);
      len--;
    }

    while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
  }

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);

  i2cState = I2C_STATE_READY;
  return 0;
}

int8_t APP_I2C_Receive(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
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
  LL_I2C_TransmitData8(I2C1, (dev_addr & (uint8_t)(~0x01)));
  /* Wait until ADDR flag is set */
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  /* Clear ADDR flag */
  LL_I2C_ClearFlag_ADDR(I2C1);

  /* Wait until TXE flag is set */
  while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  /* Send memory address */
  LL_I2C_TransmitData8(I2C1, (uint8_t)(reg_addr & 0x00FF));
  while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);

  /***** Restart to read *****/

  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2C1);
  /* Wait until SB flag is set */
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);

  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (dev_addr | 0x1));
  /* Wait until ADDR flag is set */
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);

  if (len == 0U)
  {
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_GenerateStopCondition(I2C1);
  }
  else if(len == 1U)
  {
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

    __disable_irq();
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_GenerateStopCondition(I2C1);
    __enable_irq();
  }
  else if(len == 2U)
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

  while (len > 0U)
  {
    if (len <= 3U)
    {
      if (len == 1U)
      {
        while(LL_I2C_IsActiveFlag_RXNE(I2C1) != 1);
        *data++ = LL_I2C_ReceiveData8(I2C1);
        len--;
      }
      else if (len == 2U)
      {
        while(LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        
        __disable_irq();
        LL_I2C_GenerateStopCondition(I2C1);
        *data++ = LL_I2C_ReceiveData8(I2C1);
        len--;
        __enable_irq();
        
        *data++ = LL_I2C_ReceiveData8(I2C1);
        len--;
      }
      else
      {
        while(LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
        
        __disable_irq();
        *data++ = LL_I2C_ReceiveData8(I2C1);
        len--;
        while(LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
        LL_I2C_GenerateStopCondition(I2C1);
        *data++ = LL_I2C_ReceiveData8(I2C1);
        len--;
        __enable_irq();
        
        *data++ = LL_I2C_ReceiveData8(I2C1);
        len--;
      }
    }
    else
    {
      while(LL_I2C_IsActiveFlag_RXNE(I2C1) != 1);
      
      *data++ = LL_I2C_ReceiveData8(I2C1);
       len--;
      
      if (LL_I2C_IsActiveFlag_BTF(I2C1) == 1)
      {
        *data++ = LL_I2C_ReceiveData8(I2C1);
        len--;
      }
    }
  }

  i2cState = I2C_STATE_READY;
  return 0;
}


#ifdef __cplusplus
}
#endif
