/**
  ******************************************************************************
  * @file    py32f002b_hal_rcc_ex.h
  * @author  MCU Application Team
  * @brief   Header file of RCC HAL Extended module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PY32F002B_HAL_RCC_EX_H
#define __PY32F002B_HAL_RCC_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f002b_hal_def.h"

/** @addtogroup PY32F002B_HAL_Driver
  * @{
  */

/** @addtogroup RCCEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup RCCEx_Exported_Types RCCEx Exported Types
  * @{
  */

/**
  * @brief  RCC extended clocks structure definition
  */
typedef struct
{
  uint32_t PeriphClockSelection;   /*!< The Extended Clock to be configured.
                                        This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */

#if defined(RCC_CCIPR_COMP1SEL)
  uint32_t Comp1ClockSelection;    /*!< Specifies COMP1 clock source.
                                        This parameter can be a value of @ref RCCEx_COMP1_Clock_Source */
#endif

#if defined(RCC_CCIPR_COMP2SEL)
  uint32_t Comp2ClockSelection;    /*!< Specifies COMP2 clock source.
                                        This parameter can be a value of @ref RCCEx_COMP2_Clock_Source */
#endif

#if defined(RCC_CCIPR_LPTIMSEL)
  uint32_t LptimClockSelection;   /*!< Specifies LPTIM1 clock source
                                        This parameter can be a value of @ref RCCEx_LPTIM1_Clock_Source */
#endif
} RCC_PeriphCLKInitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCCEx_Exported_Constants RCCEx Exported Constants
  * @{
  */
#if defined(RCC_BDCR_LSCOSEL)
/** @defgroup RCCEx_LSCO_Clock_Source Low Speed Clock Source
  * @{
  */
#define RCC_LSCOSOURCE_LSI             0x00000000U           /*!< LSI selection for low speed clock output */
#define RCC_LSCOSOURCE_LSE             RCC_BDCR_LSCOSEL      /*!< LSE selection for low speed clock output */
/**
  * @}
  */
#endif
/** @defgroup RCCEx_Periph_Clock_Selection Periph Clock Selection
  * @{
  */
#if defined(RCC_CCIPR_COMP1SEL)
#define RCC_PERIPHCLK_COMP1            0x00000002U
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
#define RCC_PERIPHCLK_COMP2            0x00000020U
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_LPTIMSEL)
#define RCC_PERIPHCLK_LPTIM            0x00000200U
#endif /* RCC_CCIPR_LPTIM1SEL */
/**
  * @}
  */

#if defined(RCC_CCIPR_COMP1SEL)
/** @defgroup RCCEx_COMP1_Clock_Source RCC COMP1 Clock Source
  * @{
  */
#define RCC_COMP1CLKSOURCE_PCLK        0x00000000U                                      /*!< APB clock selected as COMP1 clock */
#define RCC_COMP1CLKSOURCE_LSC         RCC_CCIPR_COMP1SEL                               /*!< LSC clock selected as COMP1 clock */
/**
  * @}
  */
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
/** @defgroup RCCEx_COMP2_Clock_Source RCC COMP2 Clock Source
  * @{
  */
#define RCC_COMP2CLKSOURCE_PCLK        0x00000000U                                      /*!< APB clock selected as COMP2 clock */
#define RCC_COMP2CLKSOURCE_LSC         RCC_CCIPR_COMP2SEL                               /*!< LSC clock selected as COMP2 clock */
/**
  * @}
  */
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_LPTIMSEL)
/** @defgroup RCCEx_LPTIM1_Clock_Source RCC LPTIM1 Clock Source
  * @{
  */
#define RCC_LPTIMCLKSOURCE_PCLK       0x00000000U               /*!< APB clock selected as LPTimer 1 clock */
#define RCC_LPTIMCLKSOURCE_LSI        RCC_CCIPR_LPTIMSEL_0     /*!< LSI clock selected as LPTimer 1 clock */
#define RCC_LPTIMCLKSOURCE_LSE        RCC_CCIPR_LPTIMSEL       /*!< LSE clock selected as LPTimer 1 clock */
/**
  * @}
  */
#endif /* RCC_CCIPR_LPTIM1SEL */



/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup RCCEx_Exported_Macros RCCEx Exported Macros
 * @{
 */

#if defined(RCC_CCIPR_COMP1SEL)
/** @brief  Macro to configure the COMP1 clock (COMP1CLK).
  *
  * @param  __COMP1_CLKSOURCE__ specifies the COMP1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_COMP1CLKSOURCE_PCLK   PCLK selected as COMP1 clock
  *            @arg @ref RCC_COMP1CLKSOURCE_HSI  LSC selected as COMP1 clock
  */
#define __HAL_RCC_COMP1_CONFIG(__COMP1_CLKSOURCE__) \
                  do {                                                                                                            \
                       register uint32_t regTmp1 = (RCC->CCIPR & RCC_CCIPR_COMP2SEL);                                             \
                       regTmp1 = regTmp1 | (regTmp1 >> 2);                                                                        \
                       register uint32_t regTmp2 = ((uint32_t)(__COMP1_CLKSOURCE__)) | (((uint32_t)(__COMP1_CLKSOURCE__)) >> 2);  \
                       MODIFY_REG(RCC->CCIPR, RCC_CCIPR_COMP1SEL, (regTmp1 | regTmp2));                                           \
                    } while(0U)
                  
                  

/** @brief  Macro to get the COMP1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_COMP1CLKSOURCE_PCLK1   PCLK selected as COMP1 clock
  *            @arg @ref RCC_COMP1CLKSOURCE_HSI  LSC selected as COMP1 clock
  */
#define __HAL_RCC_GET_COMP1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_COMP1SEL)))
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
/** @brief  Macro to configure the COMP2 clock (COMP2CLK).
  *
  * @param  __COMP2_CLKSOURCE__ specifies the COMP2 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_COMP2CLKSOURCE_PCLK   PCLK selected as COMP2 clock
  *            @arg @ref RCC_COMP2CLKSOURCE_HSI  LSC selected as COMP2 clock
  */
#define __HAL_RCC_COMP2_CONFIG(__COMP2_CLKSOURCE__) \
                  do {                                                                                                            \
                       register uint32_t regTmp1 = (RCC->CCIPR & RCC_CCIPR_COMP1SEL);                                             \
                       regTmp1 = regTmp1 | (regTmp1 >> 2);                                                                        \
                       register uint32_t regTmp2 = ((uint32_t)(__COMP2_CLKSOURCE__)) | (((uint32_t)(__COMP2_CLKSOURCE__)) >> 2);  \
                       MODIFY_REG(RCC->CCIPR, RCC_CCIPR_COMP2SEL, (regTmp1 | regTmp2));                                           \
                    } while(0U)
                  

/** @brief  Macro to get the COMP2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_COMP2CLKSOURCE_PCLK1   PCLK selected as COMP2 clock
  *            @arg @ref RCC_COMP2CLKSOURCE_HSI  LSC selected as COMP2 clock
  */
#define __HAL_RCC_GET_COMP2_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_COMP2SEL)))
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_LPTIMSEL)
/** @brief  Macro to configure the LPTIM1 clock (LPTIM1CLK).
  *
  * @param  __LPTIM1_CLKSOURCE__ specifies the LPTIM1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_LPTIM1CLKSOURCE_PCLK1  PCLK1 selected as LPTIM1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_LSI  HSI  selected as LPTIM1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_HSI  LSI  selected as LPTIM1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_LSE  LSE  selected as LPTIM1 clock
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  */
#define __HAL_RCC_LPTIM_CONFIG(__LPTIM1_CLKSOURCE__) \
                  do {                                                                       \
                       register uint32_t regTmp1 = (RCC->CCIPR & 0x0000FF00U);               \
                       register uint32_t regTmp2 = ((RCC->CCIPR & 0x0000FF00U) >> 2);        \
                       register uint32_t regTmp = regTmp1 | regTmp2;                         \
                       MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIMSEL, (((uint32_t)(__LPTIM1_CLKSOURCE__)) | regTmp));  \
                    } while(0U)

/** @brief  Macro to get the LPTIM clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_LPTIMCLKSOURCE_PCLK1  PCLK1 selected as LPUART1 clock
  *            @arg @ref RCC_LPTIMCLKSOURCE_LSI  HSI selected as LPUART1 clock
  *            @arg @ref RCC_LPTIMCLKSOURCE_LSE  LSE selected as LPUART1 clock
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  */
#define __HAL_RCC_GET_LPTIM_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_LPTIMSEL)))
#endif /* RCC_CCIPR_LPTIM1SEL */

/** @defgroup RCCEx_Flags_Interrupts_Management Flags Interrupts Management
  * @brief macros to manage the specified RCC Flags and interrupts.
  * @{
  */



/**
  * @}
  */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCCEx_Exported_Functions
  * @{
  */

/** @addtogroup RCCEx_Exported_Functions_Group1
  * @{
  */

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void              HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t          HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

/**
  * @}
  */

/** @addtogroup RCCEx_Exported_Functions_Group2
  * @{
  */
#if defined(RCC_BDCR_LSCOEN)
void              HAL_RCCEx_EnableLSCO(uint32_t LSCOSource);
void              HAL_RCCEx_DisableLSCO(void);
#endif
/**
  * @}
  */


/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup RCCEx_Private_Macros RCCEx Private Macros
  * @{
  */
#if defined(RCC_BDCR_LSCOSEL)
#define IS_RCC_LSCOSOURCE(__SOURCE__) (((__SOURCE__) == RCC_LSCOSOURCE_LSI) || \
                                       ((__SOURCE__) == RCC_LSCOSOURCE_LSE))
#endif

#if defined(RCC_CCIPR_COMP1SEL)
#define IS_RCC_COMP1CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_COMP1CLKSOURCE_PCLK)  || \
                ((__SOURCE__) == RCC_COMP1CLKSOURCE_LSC))
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
#define IS_RCC_COMP2CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_COMP2CLKSOURCE_PCLK)  || \
                ((__SOURCE__) == RCC_COMP2CLKSOURCE_LSC))
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_LPTIMSEL)
#define IS_RCC_LPTIM1CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_LPTIMCLKSOURCE_PCLK)|| \
                ((__SOURCE__) == RCC_LPTIMCLKSOURCE_LSI)  || \
                ((__SOURCE__) == RCC_LPTIMCLKSOURCE_LSE))
#endif /* RCC_CCIPR_LPTIM1SEL */

#define IS_RCC_PERIPHCLOCK(__SELECTION__)  \
  ((((__SELECTION__) & RCC_PERIPHCLK_COMP1)   == RCC_PERIPHCLK_COMP1)   || \
   (((__SELECTION__) & RCC_PERIPHCLK_COMP2)   == RCC_PERIPHCLK_COMP2)   || \
   (((__SELECTION__) & RCC_PERIPHCLK_LPTIM)   == RCC_PERIPHCLK_LPTIM))
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __PY32F002B_HAL_RCC_EX_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
