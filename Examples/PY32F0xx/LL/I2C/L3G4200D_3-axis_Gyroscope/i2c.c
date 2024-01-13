#include "i2c.h"

#define I2C_STATE_READY    0
#define I2C_STATE_BUSY_TX  1
#define I2C_STATE_BUSY_RX  2

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MAX_DELAY                   0xFFFFU


static uint32_t i2cState  = I2C_STATE_READY;


static ErrorStatus I2C_WaitOnBTFFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout);
static ErrorStatus I2C_WaitOnTXEFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout);
static ErrorStatus I2C_WaitOnRXNEFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout);
static ErrorStatus I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout);
static ErrorStatus I2C_WaitOnFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t SR1Flag, FlagStatus Status, uint32_t Timeout);
static ErrorStatus I2C_WaitSR2FlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t SR2Flag, FlagStatus Status, uint32_t Timeout);
static ErrorStatus I2C_RequestMemoryWrite(I2C_TypeDef *I2Cx, uint8_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout);


ErrorStatus I2C_MemoryRead(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint8_t stopped = 0;

  if (i2cState != I2C_STATE_READY) return ERROR;

  i2cState = I2C_STATE_BUSY_RX;

  /* Wait until BUSY flag is reset */
  if (I2C_WaitSR2FlagUntilTimeout(I2Cx, I2C_SR2_BUSY, SET, I2C_MAX_DELAY) == ERROR)
  {
    i2cState = I2C_STATE_READY;
    return ERROR;
  }
  /* Disable Pos */
  LL_I2C_DisableBitPOS(I2Cx);

  /* Enable Acknowledge */
  LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
  /* Write Memory Address */
  if (I2C_RequestMemoryWrite(I2Cx, DevAddress, MemAddress, MemAddSize, I2C_MAX_DELAY) == ERROR)
  {
    return ERROR;
  }
  /* Generate Restart */
  LL_I2C_GenerateStartCondition(I2Cx);
  /* Wait until SB flag is set */
  if (I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_SR1_SB, RESET, I2C_MAX_DELAY) == ERROR)
  {
    return ERROR;
  }
  /* Send slave address */
  LL_I2C_TransmitData8(I2Cx, (DevAddress | 0x1));
  /* Wait until ADDR flag is set */
  if (I2C_WaitOnMasterAddressFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
  {
    return ERROR;
  }

  /* Clear ADDR flag */
  LL_I2C_ClearFlag_ADDR(I2Cx);

  if (Size > 2U)
  {
    /* Enable Acknowledge */
    LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
  }
  do
  {
    if (Size < 2 && stopped == 0)
    {
      /* Disable Acknowledge */
      LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
      /* Generate Stop */
      LL_I2C_GenerateStopCondition(I2Cx);
      stopped = 1;
    }
    /* Wait until RXNE flag is set */
    if (I2C_WaitOnRXNEFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
    {
      i2cState = I2C_STATE_READY;
      return ERROR;
    }
    /* Read data from DR */
    *pData++ = LL_I2C_ReceiveData8(I2Cx);
    Size--;

    if (Size < 2 && stopped == 0)
    {
      /* Disable Acknowledge */
      LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
      /* Generate Stop */
      LL_I2C_GenerateStopCondition(I2Cx);
      stopped = 1;
    }

    if ((LL_I2C_IsActiveFlag_BTF(I2Cx)) && (Size != 0U))
    {
      /* Read data from DR */
      *pData++ = LL_I2C_ReceiveData8(I2Cx);
      Size--;
    }

  } while (Size > 0U);

  // if (Size == 1U)
  // {
  //   /* Disable Acknowledge */
  //   LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
  //   /* Clear ADDR flag */
  //   LL_I2C_ClearFlag_ADDR(I2Cx);
  //   /* Generate Stop */
  //   LL_I2C_GenerateStopCondition(I2Cx);
  // }
  // else if (Size == 2U)
  // {
  //   /* Enable Pos */
  //   LL_I2C_EnableBitPOS(I2Cx);
  //   /* Clear ADDR flag */
  //   LL_I2C_ClearFlag_ADDR(I2Cx);
  //   /* Disable Acknowledge */
  //   LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
  // }
  // else
  // {
  //   /* Enable Acknowledge */
  //   LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
  //   /* Clear ADDR flag */
  //   LL_I2C_ClearFlag_ADDR(I2Cx);
  // }

  // while (Size > 0U)
  // {
  //   if (Size <= 3U)
  //   {
  //     if (Size == 1U)
  //     {
  //       /* Wait until RXNE flag is set */
  //       if (I2C_WaitOnRXNEFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
  //       {
  //         i2cState = I2C_STATE_READY;
  //         return ERROR;
  //       }
  //       *pData++ = LL_I2C_ReceiveData8(I2Cx);
  //       Size--;
  //     }
  //     else if (Size == 2U)
  //     {
  //       /* Wait until BTF flag is set */
  //       if (I2C_WaitOnBTFFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
  //       {
  //         i2cState = I2C_STATE_READY;
  //         return ERROR;
  //       }
  //       /* Generate Stop */
  //       LL_I2C_GenerateStopCondition(I2Cx);
  //       /* Read data from DR */
  //       *pData++ = LL_I2C_ReceiveData8(I2Cx);
  //       Size--;
  //       *pData++ = LL_I2C_ReceiveData8(I2Cx);
  //       Size--;
  //     }
  //     else
  //     {
  //       /* Wait until BTF flag is set */
  //       if (I2C_WaitOnBTFFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
  //       {
  //         i2cState = I2C_STATE_READY;
  //         return ERROR;
  //       }
  //       /* Disable Acknowledge */
  //       LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
  //       /* Read data from DR */
  //       *pData++ = LL_I2C_ReceiveData8(I2Cx);
  //       Size--;

  //       /* Wait until BTF flag is set */
  //       if (I2C_WaitOnBTFFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
  //       {
  //         i2cState = I2C_STATE_READY;
  //         return ERROR;
  //       }
  //       /* Generate Stop */
  //       LL_I2C_GenerateStopCondition(I2Cx);
  //       /* Read data from DR */
  //       *pData++ = LL_I2C_ReceiveData8(I2Cx);
  //       Size--;
  //       *pData++ = LL_I2C_ReceiveData8(I2Cx);
  //       Size--;
  //     }
  //   }
  //   else
  //   {
  //     /* Wait until RXNE flag is set */
  //     if (I2C_WaitOnRXNEFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
  //     {
  //       i2cState = I2C_STATE_READY;
  //       return ERROR;
  //     }
  //     /* Read data from DR */
  //     *pData++ = LL_I2C_ReceiveData8(I2Cx);
  //     Size--;

  //     if ((LL_I2C_IsActiveFlag_BTF(I2Cx)) && (Size != 0U))
  //     {
  //       /* Read data from DR */
  //       *pData++ = LL_I2C_ReceiveData8(I2Cx);
  //       Size--;
  //     }
  //   }
  // }

  i2cState = I2C_STATE_READY;
  return SUCCESS;
}

ErrorStatus I2C_MemoryWrite(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  if (i2cState != I2C_STATE_READY) return ERROR;

  i2cState = I2C_STATE_BUSY_TX;

  /* Wait until BUSY flag is reset */
  if (I2C_WaitSR2FlagUntilTimeout(I2Cx, I2C_SR2_BUSY, SET, I2C_MAX_DELAY) == ERROR)
  {
    i2cState = I2C_STATE_READY;
    return ERROR;
  }
  /* Disable Pos */
  LL_I2C_DisableBitPOS(I2Cx);

  /* Send Slave Address and Memory Address */
  if (I2C_RequestMemoryWrite(I2Cx, DevAddress, MemAddress, MemAddSize, I2C_MAX_DELAY) == ERROR)
  {
    i2cState = I2C_STATE_READY;
    return ERROR;
  }

  while (Size > 0U)
  {
    /* Wait until TXE flag is set */
    if (I2C_WaitOnTXEFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
    {
      i2cState = I2C_STATE_READY;
      return ERROR;
    }
    /* Write data to DR */
    LL_I2C_TransmitData8(I2Cx, *pData++);
    Size--;

    if ((LL_I2C_IsActiveFlag_BTF(I2Cx)) && (Size != 0U))
    {
      LL_I2C_TransmitData8(I2Cx, *pData++);
      Size--;
    }
  }
  /* Wait until BTF flag is set */
  if (I2C_WaitOnBTFFlagUntilTimeout(I2Cx, I2C_MAX_DELAY) == ERROR)
  {
    i2cState = I2C_STATE_READY;
    return ERROR;
  }
  /* Generate Stop */
  LL_I2C_GenerateStopCondition(I2Cx);

  i2cState = I2C_STATE_READY;
  return SUCCESS;
}

static ErrorStatus I2C_RequestMemoryWrite(I2C_TypeDef *I2Cx, uint8_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout)
{
  /* Generate Start */
  LL_I2C_GenerateStartCondition(I2Cx);
  /* Wait until SB flag is set */
  if (I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_SR1_SB, RESET, Timeout) == ERROR)
  {
    return ERROR;
  }
  /* Send slave address */
  LL_I2C_TransmitData8(I2Cx, (DevAddress & (uint8_t)(~0x01)));
  /* Wait until ADDR flag is set */
  if (I2C_WaitOnMasterAddressFlagUntilTimeout(I2Cx, Timeout) == ERROR)
  {
    return ERROR;
  }
  /* Clear ADDR flag */
  LL_I2C_ClearFlag_ADDR(I2Cx);
  /* Wait until TXE flag is set */
  if (I2C_WaitOnTXEFlagUntilTimeout(I2Cx, Timeout) == ERROR)
  {
    return ERROR;
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send LSB of memory address */
    LL_I2C_TransmitData8(I2Cx, (uint8_t)(MemAddress & 0xFF));
  }
  else /* If Memory address size is 16Bit */
  {
    /* Send MSB of memory address */
    LL_I2C_TransmitData8(I2Cx, (uint8_t)((MemAddress >> 8) & 0xFF));
    /* Wait until TXE flag is set */
    if (I2C_WaitOnTXEFlagUntilTimeout(I2Cx, Timeout) == ERROR)
    {
      return ERROR;
    }
    /* Send LSB of memory address */
    LL_I2C_TransmitData8(I2Cx, (uint8_t)(MemAddress & 0xFF));
  }
  /* Wait until TXE flag is set */
  if (I2C_WaitOnTXEFlagUntilTimeout(I2Cx, Timeout) == ERROR)
  {
    return ERROR;
  }
  return SUCCESS;
}

static ErrorStatus I2C_WaitOnFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t SR1Flag, FlagStatus Status, uint32_t Timeout)
{
  /* Wait until flag equals to expected value */
  while ((READ_BIT(I2Cx->SR1, SR1Flag) == (SR1Flag)) == Status)
  {
    if (--Timeout == 0)
    {
      return ERROR;
    }
  }
  return SUCCESS;
}

static ErrorStatus I2C_WaitSR2FlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t SR2Flag, FlagStatus Status, uint32_t Timeout)
{
  /* Wait until flag equals to expected value */
  while ((READ_BIT(I2Cx->SR2, SR2Flag) == (SR2Flag)) == Status)
  {
    if (--Timeout == 0)
    {
      return ERROR;
    }
  }
  return SUCCESS;
}

static ErrorStatus I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout)
{
  while(!LL_I2C_IsActiveFlag_ADDR(I2Cx))
  {
    if (LL_I2C_IsActiveFlag_AF(I2Cx))
    {
      /* Generate Stop */
      LL_I2C_GenerateStopCondition(I2Cx);
      /* Clear NACKF Flag */
      LL_I2C_ClearFlag_AF(I2Cx);
      return ERROR;
    }
    if (--Timeout == 0)
    {
      return ERROR;
    }
  }
  return SUCCESS;
}

static ErrorStatus I2C_WaitOnTXEFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout)
{
  while (!LL_I2C_IsActiveFlag_TXE(I2Cx))
  {
    /* Check if a NACK is detected */
    if (LL_I2C_IsActiveFlag_AF(I2Cx))
    {
      /* Generate Stop */
      LL_I2C_GenerateStopCondition(I2Cx);
      /* Clear NACKF Flag */
      LL_I2C_ClearFlag_AF(I2Cx);
      return ERROR;
    }
    if (--Timeout == 0)
    {
      return ERROR;
    }
  }
  return SUCCESS;
}

static ErrorStatus I2C_WaitOnBTFFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout)
{
  while (!LL_I2C_IsActiveFlag_BTF(I2Cx))
  {
    /* Check if a NACK is detected */
    if (LL_I2C_IsActiveFlag_AF(I2Cx))
    {
      /* Generate Stop */
      LL_I2C_GenerateStopCondition(I2Cx);
      /* Clear NACKF Flag */
      LL_I2C_ClearFlag_AF(I2Cx);
      return ERROR;
    }
    if (--Timeout == 0)
    {
      return ERROR;
    }
  }
  return SUCCESS;
}

static ErrorStatus I2C_WaitOnRXNEFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout)
{
  while (!LL_I2C_IsActiveFlag_RXNE(I2Cx))
  {
    /* Check if a STOPF is detected */
    if (LL_I2C_IsActiveFlag_STOP(I2Cx))
    {
      /* Clear STOP Flag */
      CLEAR_BIT(I2Cx->SR1, I2C_SR1_STOPF);
      return ERROR;
    }
    if (--Timeout == 0)
    {
      return ERROR;
    }
  }
  return SUCCESS;
}



#ifdef __cplusplus
}
#endif
