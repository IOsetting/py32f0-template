/**
  ******************************************************************************
  * @file    py32f002b_hal_rcc_ex.c
  * @author  MCU Application Team
  * @brief   Extended RCC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities RCC extended peripheral:
  *           + Extended Peripheral Control functions
  *           + Extended Clock management functions
  *
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

/* Includes ------------------------------------------------------------------*/
#include "py32f002b_hal.h"

/** @addtogroup PY32F002B_HAL_Driver
  * @{
  */

/** @defgroup RCCEx RCCEx
  * @brief RCC Extended HAL module driver
  * @{
  */

#ifdef HAL_RCC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup RCCEx_Exported_Functions RCCEx Exported Functions
  * @{
  */

/** @defgroup RCCEx_Exported_Functions_Group1 Extended Peripheral Control functions
 *  @brief  Extended Peripheral Control functions
 *
@verbatim
 ===============================================================================
                ##### Extended Peripheral Control functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the RCC Clocks
    frequencies.
    [..]
    (@) Important note: Care must be taken when @ref HAL_RCCEx_PeriphCLKConfig() is used to
        select the RTC clock source; in this case the Backup domain will be reset in
        order to modify the RTC Clock source, as consequence RTC registers (including
        the backup registers) and RCC_BDCR register are set to their reset values.

@endverbatim
  * @{
  */
/**
  * @brief  Initialize the RCC extended peripherals clocks according to the specified
  *         parameters in the @ref RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit  pointer to a @ref RCC_PeriphCLKInitTypeDef structure that
  *         contains a field PeriphClockSelection which can be a combination of the following values:
  *            @arg @ref RCC_PERIPHCLK_COMP1   COMP1 peripheral clock  (1)
  *            @arg @ref RCC_PERIPHCLK_COMP2   COMP2 peripheral clock  (1)
  *            @arg @ref RCC_PERIPHCLK_LPTIM  LPTIM peripheral clock  (1)
  *
  * @note   (1) Peripherals maybe not available on some devices
  * @note   Care must be taken when @ref HAL_RCCEx_PeriphCLKConfig() is used to select
  *         the RTC clock source: in this case the access to Backup domain is enabled.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

#if defined(RCC_CCIPR_COMP1SEL)
  /*-------------------------- COMP1 clock source configuration -------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_COMP1) == RCC_PERIPHCLK_COMP1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_COMP1CLKSOURCE(PeriphClkInit->Comp1ClockSelection));

    /* Configure the COMP1 clock source */
    __HAL_RCC_COMP1_CONFIG(PeriphClkInit->Comp1ClockSelection);
  }
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
  /*-------------------------- COMP2 clock source configuration -------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_COMP2) == RCC_PERIPHCLK_COMP2)
  {
    /* Check the parameters */
    assert_param(IS_RCC_COMP2CLKSOURCE(PeriphClkInit->Comp2ClockSelection));

    /* Configure the COMP2 clock source */
    __HAL_RCC_COMP2_CONFIG(PeriphClkInit->Comp2ClockSelection);
  }
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_LPTIMSEL)
  /*-------------------------- LPTIM clock source configuration -------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM) == (RCC_PERIPHCLK_LPTIM))
  {
    assert_param(IS_RCC_LPTIM1CLKSOURCE(PeriphClkInit->LptimClockSelection));
    __HAL_RCC_LPTIM_CONFIG(PeriphClkInit->LptimClockSelection);
  }
#endif /* RCC_CCIPR_LPTIM1SEL */

  return HAL_OK;
}

/**
  * @brief  Get the RCC_ClkInitStruct according to the internal RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         returns the configuration information for the Extended Peripherals
  *         clocks: COMP1, COMP2, LPTIM
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripheral availability.
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  /* Set all possible values for the extended clock type parameter------------*/
PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_COMP1 | RCC_PERIPHCLK_COMP2 | \
                                      RCC_PERIPHCLK_LPTIM;

#if defined(RCC_CCIPR_COMP1SEL)
  /* Get the COMP1 clock source --------------------------------------------*/
  PeriphClkInit->Comp1ClockSelection = __HAL_RCC_GET_COMP1_SOURCE();
#endif /* RCC_CCIPR_COMP1SEL */
#if defined(RCC_CCIPR_COMP2SEL)
  /* Get the COMP2 clock source ---------------------------------------------*/
  PeriphClkInit->Comp2ClockSelection  = __HAL_RCC_GET_COMP2_SOURCE();
#endif /* RCC_CCIPR_COMP2SEL */
#if defined(RCC_CCIPR_LPTIMSEL)
  /* Get the LPTIM clock source ---------------------------------------------*/
  PeriphClkInit->LptimClockSelection  = __HAL_RCC_GET_LPTIM_SOURCE();
#endif /* RCC_CCIPR_LPTIM2SEL */
}

/**
  * @brief  Return the peripheral clock frequency.
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk  Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_PERIPHCLK_COMP1   COMP1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_COMP2   COMP2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPTIM   LPTIM peripheral clock
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripheral availability.
  * @retval Frequency in Hz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  uint32_t frequency = 0U;
  uint32_t srcclk;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClk));

  switch (PeriphClk)
  {

#if defined(RCC_CCIPR_COMP1SEL)
  case RCC_PERIPHCLK_COMP1:
    /* Get the current COMP1 source */
    srcclk = __HAL_RCC_GET_COMP1_SOURCE();

    if (srcclk == RCC_COMP1CLKSOURCE_PCLK)            /* PCLK1 */
    {
      frequency = HAL_RCC_GetPCLK1Freq();
    }
    else if ((HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY)) && (HAL_IS_BIT_CLR(RCC->BDCR, RCC_BDCR_LSCOSEL)) \
             && (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSCOEN)) && (srcclk == RCC_COMP1CLKSOURCE_LSC))
    {
      if ((READ_BIT(RCC->ICSCR, RCC_ICSCR_LSI_TRIM) >> RCC_ICSCR_LSI_TRIM_Pos) == RCC_LSICALIBRATION_32768Hz)
      {
        frequency = 32768U;
      }
      else if ((READ_BIT(RCC->ICSCR, RCC_ICSCR_LSI_TRIM) >> RCC_ICSCR_LSI_TRIM_Pos) == RCC_LSICALIBRATION_38400Hz)
      {
        frequency = 38400U;
      }
      else
      {
        frequency = 0U;
      }
    }
    else if ((HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY)) && (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSCOSEL)) \
             && (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSCOEN)) && (srcclk == RCC_COMP1CLKSOURCE_LSC))
    {
      frequency = LSE_VALUE;
    }
    /* Clock not enabled for COMP1 */
    else
    {
      /* Nothing to do as frequency already initialized to 0U */
    }
    break;
#endif

#if defined(RCC_CCIPR_COMP2SEL)
  case RCC_PERIPHCLK_COMP2:
    /* Get the current COMP2 source */
    srcclk = __HAL_RCC_GET_COMP2_SOURCE();

    if (srcclk == RCC_COMP2CLKSOURCE_PCLK)            /* PCLK1 */
    {
      frequency = HAL_RCC_GetPCLK1Freq();
    }
    else if ((HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY)) && (HAL_IS_BIT_CLR(RCC->BDCR, RCC_BDCR_LSCOSEL)) \
             && (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSCOEN)) && (srcclk == RCC_COMP2CLKSOURCE_LSC))
    {
      if ((READ_BIT(RCC->ICSCR, RCC_ICSCR_LSI_TRIM) >> RCC_ICSCR_LSI_TRIM_Pos) == RCC_LSICALIBRATION_32768Hz)
      {
        frequency = 32768U;
      }
      else if ((READ_BIT(RCC->ICSCR, RCC_ICSCR_LSI_TRIM) >> RCC_ICSCR_LSI_TRIM_Pos) == RCC_LSICALIBRATION_38400Hz)
      {
        frequency = 38400U;
      }
      else
      {
        frequency = 0U;
      }
    }
    else if ((HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY)) && (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSCOSEL)) \
             && (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSCOEN)) && (srcclk == RCC_COMP2CLKSOURCE_LSC))
    {
      frequency = LSE_VALUE;
    }
    /* Clock not enabled for COMP2 */
    else
    {
      /* Nothing to do as frequency already initialized to 0U */
    }
    break;
#endif

#if defined(RCC_CCIPR_LPTIMSEL)
  case RCC_PERIPHCLK_LPTIM:
    /* Get the current LPTIM1 source */
    srcclk = __HAL_RCC_GET_LPTIM_SOURCE();

    if (srcclk == RCC_LPTIMCLKSOURCE_PCLK)
    {
      frequency = HAL_RCC_GetPCLK1Freq();
    }
    else if ((HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY)) && (srcclk == RCC_LPTIMCLKSOURCE_LSI))
    {
      if ((READ_BIT(RCC->ICSCR, RCC_ICSCR_LSI_TRIM) >> RCC_ICSCR_LSI_TRIM_Pos) == RCC_LSICALIBRATION_32768Hz)
      {
        frequency = 32768U;
      }
      else if ((READ_BIT(RCC->ICSCR, RCC_ICSCR_LSI_TRIM) >> RCC_ICSCR_LSI_TRIM_Pos) == RCC_LSICALIBRATION_38400Hz)
      {
        frequency = 38400U;
      }
      else
      {
        frequency = 0U;
      }
    }
    else if ((HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY)) && (srcclk == RCC_LPTIMCLKSOURCE_LSE))
    {
      frequency = LSE_VALUE;
    }
    /* Clock not enabled for LPTIM1 */
    else
    {
      /* Nothing to do as frequency already initialized to 0U */
    }
    break;
#endif /* RCC_CCIPR_LPTIM1SEL */

  default:
    break;
  }

  return (frequency);
}

/**
  * @}
  */

/** @defgroup RCCEx_Exported_Functions_Group2 Extended Clock management functions
 *  @brief  Extended Clock management functions
 *
@verbatim
 ===============================================================================
                ##### Extended clock management functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the
    activation or deactivation of LSE CSS, Low speed clock output and
    clock after wake-up from STOP mode.
@endverbatim
  * @{
  */
/**
  * @brief  Select the Low Speed clock source to output on LSCO pin (PA2).
  * @param  LSCOSource  specifies the Low Speed clock source to output.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_LSCOSOURCE_LSI  LSI clock selected as LSCO source
  *            @arg @ref RCC_LSCOSOURCE_LSE  LSE clock selected as LSCO source
  * @retval None
  */
void HAL_RCCEx_EnableLSCO(uint32_t LSCOSource)
{
  /* Check the parameters */
  assert_param(IS_RCC_LSCOSOURCE(LSCOSource));

  MODIFY_REG(RCC->BDCR, RCC_BDCR_LSCOSEL | RCC_BDCR_LSCOEN, LSCOSource | RCC_BDCR_LSCOEN);
}

/**
  * @brief  Disable the Low Speed clock output.
  * @retval None
  */
void HAL_RCCEx_DisableLSCO(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSCOEN);
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

#endif /* HAL_RCC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
