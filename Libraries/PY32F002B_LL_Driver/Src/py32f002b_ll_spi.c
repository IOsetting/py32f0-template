/**
  ******************************************************************************
  * @file    py32f002b_ll_spi.c
  * @author  MCU Application Team
  * @brief   SPI LL module driver.
  ******************************************************************************
  Additional table :

       DataSize = SPI_DATASIZE_8BIT:
       +----------------------------------------------------------------------------------------------+
       |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
       | Process | Tranfert mode  |---------------------|----------------------|----------------------|
       |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
       |==============================================================================================|
       |    TX   |     Polling    | Fpclk/2  | Fpclk/32 |    NA     |    NA    |    NA     |   NA     |
       |    /    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    RX   |     Interrupt  | Fpclk/2  | Fpclk/32 |    NA     |    NA    |    NA     |   NA     |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    |    NA    |    NA    | Fpclk/4   | Fpclk/8  | Fpclk/4   | Fpclk/8  |
       |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    X    |     Interrupt  |    NA    |    NA    | Fpclk/16  | Fpclk/16 | Fpclk/16  | Fpclk/16 |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    |    NA    |     NA   |     NA    |    NA    | Fpclk/2   | Fpclk/8  |
       |    T    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    X    |     Interrupt  |    NA    |     NA   |     NA    |    NA    | Fpclk/2   | Fpclk/16 |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       +----------------------------------------------------------------------------------------------+

       DataSize = SPI_DATASIZE_16BIT:
       +----------------------------------------------------------------------------------------------+
       |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
       | Process | Tranfert mode  |---------------------|----------------------|----------------------|
       |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
       |==============================================================================================|
       |    TX   |     Polling    | Fpclk/2  | Fpclk/16 |    NA     |    NA    |    NA     |   NA     |
       |    /    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    RX   |     Interrupt  | Fpclk/2  | Fpclk/16 |    NA     |    NA    |    NA     |   NA     |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    |    NA    |    NA    | Fpclk/2   | Fpclk/4  | Fpclk/2   | Fpclk/4  |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    R    |     Interrupt  |    NA    |    NA    | Fpclk/2   | Fpclk/8  | Fpclk/2   | Fpclk/8  |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    |    NA    |     NA   |     NA    |    NA    | Fpclk/2   | Fpclk/4  |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    T    |     Interrupt  |    NA    |     NA   |     NA    |    NA    | Fpclk/2   | Fpclk/8  |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       +----------------------------------------------------------------------------------------------+
       @note The max SPI frequency depend on SPI data size (8bits, 16bits),
             SPI mode(2 Lines fullduplex, 2 lines RxOnly, 1 line TX/RX) and Process mode (Polling, IT).

  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#if defined(USE_FULL_LL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "py32f002b_ll_spi.h"
#include "py32f002b_ll_bus.h"
#include "py32f002b_ll_rcc.h"

#ifdef  USE_FULL_ASSERT
  #include "py32_assert.h"
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

/** @addtogroup PY32F002B_LL_Driver
  * @{
  */

#if defined (SPI1)

/** @addtogroup SPI_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup SPI_LL_Private_Constants SPI Private Constants
  * @{
  */
/* SPI registers Masks */
#define SPI_CR1_CLEAR_MASK                 (SPI_CR1_CPHA    | SPI_CR1_CPOL     | SPI_CR1_MSTR   | \
                                            SPI_CR1_BR      | SPI_CR1_LSBFIRST | SPI_CR1_SSI    | \
                                            SPI_CR1_SSM     | SPI_CR1_RXONLY   | SPI_CR1_DFF    | \
                                            SPI_CR1_BIDIOE | SPI_CR1_BIDIMODE)
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_LL_Private_Macros SPI Private Macros
  * @{
  */
#define IS_LL_SPI_TRANSFER_DIRECTION(__VALUE__) (((__VALUE__) == LL_SPI_FULL_DUPLEX)       \
                                                 || ((__VALUE__) == LL_SPI_SIMPLEX_RX)     \
                                                 || ((__VALUE__) == LL_SPI_HALF_DUPLEX_RX) \
                                                 || ((__VALUE__) == LL_SPI_HALF_DUPLEX_TX))

#define IS_LL_SPI_MODE(__VALUE__) (((__VALUE__) == LL_SPI_MODE_MASTER) \
                                   || ((__VALUE__) == LL_SPI_MODE_SLAVE))

#define IS_LL_SPI_DATAWIDTH(__VALUE__) (((__VALUE__) == LL_SPI_DATAWIDTH_8BIT)  \
                                        || ((__VALUE__) == LL_SPI_DATAWIDTH_16BIT))

#define IS_LL_SPI_POLARITY(__VALUE__) (((__VALUE__) == LL_SPI_POLARITY_LOW) \
                                       || ((__VALUE__) == LL_SPI_POLARITY_HIGH))

#define IS_LL_SPI_PHASE(__VALUE__) (((__VALUE__) == LL_SPI_PHASE_1EDGE) \
                                    || ((__VALUE__) == LL_SPI_PHASE_2EDGE))

#define IS_LL_SPI_NSS(__VALUE__) (((__VALUE__) == LL_SPI_NSS_SOFT)          \
                                  || ((__VALUE__) == LL_SPI_NSS_HARD_INPUT) \
                                  || ((__VALUE__) == LL_SPI_NSS_HARD_OUTPUT))

#define IS_LL_SPI_BAUDRATE(__VALUE__) (((__VALUE__) == LL_SPI_BAUDRATEPRESCALER_DIV2)      \
                                       || ((__VALUE__) == LL_SPI_BAUDRATEPRESCALER_DIV4)   \
                                       || ((__VALUE__) == LL_SPI_BAUDRATEPRESCALER_DIV8)   \
                                       || ((__VALUE__) == LL_SPI_BAUDRATEPRESCALER_DIV16)  \
                                       || ((__VALUE__) == LL_SPI_BAUDRATEPRESCALER_DIV32)  \
                                       || ((__VALUE__) == LL_SPI_BAUDRATEPRESCALER_DIV64)  \
                                       || ((__VALUE__) == LL_SPI_BAUDRATEPRESCALER_DIV128) \
                                       || ((__VALUE__) == LL_SPI_BAUDRATEPRESCALER_DIV256))

#define IS_LL_SPI_BITORDER(__VALUE__) (((__VALUE__) == LL_SPI_LSB_FIRST) \
                                       || ((__VALUE__) == LL_SPI_MSB_FIRST))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_LL_Exported_Functions
  * @{
  */

/** @addtogroup SPI_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the SPI registers to their default reset values.
  * @param  SPIx SPI Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: SPI registers are de-initialized
  *          - ERROR: SPI registers are not de-initialized
  */
ErrorStatus LL_SPI_DeInit(SPI_TypeDef *SPIx)
{
  ErrorStatus status = ERROR;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_INSTANCE(SPIx));

  if (SPIx == SPI1)
  {
    /* Force reset of SPI clock */
    LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_SPI1);

    /* Release reset of SPI clock */
    LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_SPI1);

    status = SUCCESS;
  }
  return status;
}

/**
  * @brief  Initialize the SPI registers according to the specified parameters in SPI_InitStruct.
  * @note   As some bits in SPI configuration registers can only be written when the SPI is disabled (SPI_CR1_SPE bit =0),
  *         SPI peripheral should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @param  SPIx SPI Instance
  * @param  SPI_InitStruct pointer to a @ref LL_SPI_InitTypeDef structure
  * @retval An ErrorStatus enumeration value. (Return always SUCCESS)
  */
ErrorStatus LL_SPI_Init(SPI_TypeDef *SPIx, LL_SPI_InitTypeDef *SPI_InitStruct)
{
  ErrorStatus status = ERROR;

  /* Check the SPI Instance SPIx*/
  assert_param(IS_SPI_ALL_INSTANCE(SPIx));

  /* Check the SPI parameters from SPI_InitStruct*/
  assert_param(IS_LL_SPI_TRANSFER_DIRECTION(SPI_InitStruct->TransferDirection));
  assert_param(IS_LL_SPI_MODE(SPI_InitStruct->Mode));
  assert_param(IS_LL_SPI_DATAWIDTH(SPI_InitStruct->DataWidth));
  assert_param(IS_LL_SPI_POLARITY(SPI_InitStruct->ClockPolarity));
  assert_param(IS_LL_SPI_PHASE(SPI_InitStruct->ClockPhase));
  assert_param(IS_LL_SPI_NSS(SPI_InitStruct->NSS));
  assert_param(IS_LL_SPI_BAUDRATE(SPI_InitStruct->BaudRate));
  assert_param(IS_LL_SPI_BITORDER(SPI_InitStruct->BitOrder));

  if (LL_SPI_IsEnabled(SPIx) == 0x00000000U)
  {
    /*---------------------------- SPIx CR1 Configuration ------------------------
     * Configure SPIx CR1 with parameters:
     * - TransferDirection:  SPI_CR1_BIDIMODE, SPI_CR1_BIDIOE and SPI_CR1_RXONLY bits
     * - Master/Slave Mode:  SPI_CR1_MSTR bit
     * - ClockPolarity:      SPI_CR1_CPOL bit
     * - ClockPhase:         SPI_CR1_CPHA bit
     * - NSS management:     SPI_CR1_SSM bit
     * - BaudRate prescaler: SPI_CR1_BR[2:0] bits
     * - BitOrder:           SPI_CR1_LSBFIRST bit
     * - DataWidth:          SPI_CR1_DFF bit
     */
    MODIFY_REG(SPIx->CR1,
               SPI_CR1_CLEAR_MASK,
               SPI_InitStruct->TransferDirection | SPI_InitStruct->Mode |
               SPI_InitStruct->ClockPolarity | SPI_InitStruct->ClockPhase |
               (SPI_InitStruct->NSS & SPI_CR1_SSM) | SPI_InitStruct->BaudRate |
               SPI_InitStruct->BitOrder | SPI_InitStruct->DataWidth);

    /*---------------------------- SPIx CR2 Configuration ------------------------
     * Configure SPIx CR2 with parameters:
     * - NSS management:     SSOE bit
     */
    MODIFY_REG(SPIx->CR2, SPI_CR2_SSOE, ((SPI_InitStruct->NSS >> 16U) & SPI_CR2_SSOE));

    status = SUCCESS;
  }
  return status;
}

/**
  * @brief  Set each @ref LL_SPI_InitTypeDef field to default value.
  * @param  SPI_InitStruct pointer to a @ref LL_SPI_InitTypeDef structure
  * whose fields will be set to default values.
  * @retval None
  */
void LL_SPI_StructInit(LL_SPI_InitTypeDef *SPI_InitStruct)
{
  /* Set SPI_InitStruct fields to default values */
  SPI_InitStruct->TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct->Mode              = LL_SPI_MODE_SLAVE;
  SPI_InitStruct->DataWidth         = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct->ClockPolarity     = LL_SPI_POLARITY_LOW;
  SPI_InitStruct->ClockPhase        = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct->NSS               = LL_SPI_NSS_HARD_INPUT;
  SPI_InitStruct->BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct->BitOrder          = LL_SPI_MSB_FIRST;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (SPI1) */

/**
  * @}
  */

#endif /* USE_FULL_LL_DRIVER */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
