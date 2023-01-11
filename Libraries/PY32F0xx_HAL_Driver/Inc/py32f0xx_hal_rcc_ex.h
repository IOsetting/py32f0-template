/**
  ******************************************************************************
  * @file    py32f0xx_hal_rcc_ex.h
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
#ifndef __PY32F0xx_HAL_RCC_EX_H
#define __PY32F0xx_HAL_RCC_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_hal_def.h"

/** @addtogroup PY32F0xx_HAL_Driver
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

#if defined(RCC_CCIPR_PVDSEL)
  uint32_t PvdClockSelection;      /*!< Specifies PVD clock source.
                                       This parameter can be a value of @ref RCCEx_PVD_Clock_Source */
#endif

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

#if defined(RCC_BDCR_RTCSEL)
  uint32_t RTCClockSelection;      /*!< Specifies RTC clock source.
                                        This parameter can be a value of @ref RCC_RTC_Clock_Source */
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
#if defined(RCC_LSE_SUPPORT)
#define RCC_LSCOSOURCE_LSE             RCC_BDCR_LSCOSEL      /*!< LSE selection for low speed clock output */
#endif
/**
  * @}
  */
#endif
/** @defgroup RCCEx_Periph_Clock_Selection Periph Clock Selection
  * @{
  */
#if defined(RCC_CCIPR_PVDSEL)
#define RCC_PERIPHCLK_PVD              0x00000001U
#endif /* RCC_CCIPR_PVDSEL */

#if defined(RCC_CCIPR_COMP1SEL)
#define RCC_PERIPHCLK_COMP1            0x00000002U
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
#define RCC_PERIPHCLK_COMP2            0x00000020U
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_LPTIMSEL)
#define RCC_PERIPHCLK_LPTIM            0x00000200U
#endif /* RCC_CCIPR_LPTIM1SEL */

#if defined(RCC_CCIPR_ADCSEL)
#define RCC_PERIPHCLK_ADC              0x00004000U
#endif /* RCC_CCIPR_ADCSEL */

#if defined(RCC_BDCR_RTCSEL)
#define RCC_PERIPHCLK_RTC              0x00020000U
#endif /* RCC_BDCR_RTCSEL */

/**
  * @}
  */

#if defined(RCC_CCIPR_PVDSEL)
/** @defgroup RCCEx_PVD_Clock_Source RCC PVD Clock Source
  * @{
  */
#define RCC_PVDCLKSOURCE_PCLK          0x00000000U                                      /*!< APB clock selected as PVD clock */
#define RCC_PVDCLKSOURCE_LSC           RCC_CCIPR_PVDSEL                                 /*!< LSC clock selected as PVD clock */
/**
  * @}
  */
#endif

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
#if defined(RCC_LSE_SUPPORT)
#define RCC_LPTIMCLKSOURCE_LSE        RCC_CCIPR_LPTIMSEL       /*!< LSE clock selected as LPTimer 1 clock */
#endif
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

#if defined(RCC_CCIPR_PVDSEL)
/** @brief  Macro to configure the PVD clock (PVDCLK).
  *
  * @param  __PVD_CLKSOURCE__ specifies the PVD clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_PVDCLKSOURCE_PCLK   PCLK selected as PVD clock
  *            @arg @ref RCC_PVDCLKSOURCE_LSC    LSC selected as PVD clock
  */
#define __HAL_RCC_PVD_CONFIG(__PVD_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_PVDSEL, (uint32_t)(__PVD_CLKSOURCE__))

/** @brief  Macro to get the PVD clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_PVDCLKSOURCE_PCLK   PCLK selected as PVD clock
  *            @arg @ref RCC_PVDCLKSOURCE_LSC    LSC selected as PVD clock
  */
#define __HAL_RCC_GET_PVD_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_PVDSEL)))
#endif /* RCC_CCIPR_PVDSEL */

#if defined(RCC_CCIPR_COMP1SEL)
/** @brief  Macro to configure the COMP1 clock (COMP1CLK).
  *
  * @param  __COMP1_CLKSOURCE__ specifies the COMP1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_COMP1CLKSOURCE_PCLK   PCLK selected as COMP1 clock
  *            @arg @ref RCC_COMP1CLKSOURCE_HSI  LSC selected as COMP1 clock
  */
#define __HAL_RCC_COMP1_CONFIG(__COMP1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_COMP1SEL, (uint32_t)(__COMP1_CLKSOURCE__))

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
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_COMP2SEL, (uint32_t)(__COMP2_CLKSOURCE__))

/** @brief  Macro to get the COMP2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_COMP2CLKSOURCE_PCLK1   PCLK selected as COMP2 clock
  *            @arg @ref RCC_COMP2CLKSOURCE_HSI  LSC selected as COMP2 clock
  */
#define __HAL_RCC_GET_COMP2_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_COMP2SEL)))
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_LPUART1SEL)
/** @brief  Macro to configure the LPUART1 clock (LPUART1CLK).
  *
  * @param  __LPUART1_CLKSOURCE__ specifies the LPUART1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1   PCLK1 selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_HSI  HSI selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK  System Clock selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_LSE  LSE selected as LPUART1 clock
  */
#define __HAL_RCC_LPUART1_CONFIG(__LPUART1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPUART1SEL, (uint32_t)(__LPUART1_CLKSOURCE__))

/** @brief  Macro to get the LPUART1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1  PCLK1 selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_HSI HSI selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK System Clock selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_LSE LSE selected as LPUART1 clock
  */
#define __HAL_RCC_GET_LPUART1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_LPUART1SEL)))
#endif /* RCC_CCIPR_LPUART1SEL */

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
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIMSEL, (uint32_t)(__LPTIM1_CLKSOURCE__))

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

#if defined(RCC_CCIPR_TIM1SEL)
/** @brief  Macro to configure the TIM1 interface clock
  * @param  __TIM1_CLKSOURCE__ specifies the TIM1 digital interface clock source.
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_TIM1CLKSOURCE_PLL PLLQ Output Clock selected as TIM1 clock
  *            @arg @ref RCC_TIM1CLKSOURCE_PCLK1 System Clock selected as TIM1 clock
  */
#define __HAL_RCC_TIM1_CONFIG(__TIM1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_TIM1SEL, (uint32_t)(__TIM1_CLKSOURCE__))

/** @brief  Macro to get the TIM1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_TIM1CLKSOURCE_PLL PLLQ Output Clock selected as TIM1 clock
  *            @arg @ref RCC_TIM1CLKSOURCE_PCLK1  System Clock selected as TIM1 clock
  */
#define __HAL_RCC_GET_TIM1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_TIM1SEL)))
#endif /* RCC_CCIPR_TIM1SEL */

#if defined(RCC_CCIPR_TIM15SEL)
/** @brief  Macro to configure the TIM15 interface clock
  * @param  __TIM15_CLKSOURCE__ specifies the TIM15 digital interface clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_TIM15CLKSOURCE_PLL PLLQ Output Clock selected as TIM15 clock
  *            @arg RCC_TIM15CLKSOURCE_PCLK1  System Clock selected as TIM15 clock
  */
#define __HAL_RCC_TIM15_CONFIG(__TIM15_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_TIM15SEL, (uint32_t)(__TIM15_CLKSOURCE__))

/** @brief  Macro to get the TIM15 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_TIM15CLKSOURCE_PLL PLLQ Output Clock selected as TIM15 clock
  *            @arg @ref RCC_TIM15CLKSOURCE_PCLK1  System Clock selected as TIM15 clock
  */
#define __HAL_RCC_GET_TIM15_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_TIM15SEL)))
#endif /* RCC_CCIPR_TIM15SEL */

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
#if defined(RCC_LSE_SUPPORT)
#define IS_RCC_LSCOSOURCE(__SOURCE__) (((__SOURCE__) == RCC_LSCOSOURCE_LSI) || \
                                       ((__SOURCE__) == RCC_LSCOSOURCE_LSE))
#else
#define IS_RCC_LSCOSOURCE(__SOURCE__) (((__SOURCE__) == RCC_LSCOSOURCE_LSI))
#endif
#endif

#if defined(RCC_LSE_SUPPORT)
#define IS_RCC_USART1CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_USART1CLKSOURCE_PCLK1)  || \
                ((__SOURCE__) == RCC_USART1CLKSOURCE_SYSCLK) || \
                ((__SOURCE__) == RCC_USART1CLKSOURCE_LSE)    || \
                ((__SOURCE__) == RCC_USART1CLKSOURCE_HSI))
#else
#define IS_RCC_USART1CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_USART1CLKSOURCE_PCLK1)  || \
                ((__SOURCE__) == RCC_USART1CLKSOURCE_SYSCLK) || \
                ((__SOURCE__) == RCC_USART1CLKSOURCE_HSI))
#endif

#if defined(RCC_CCIPR_PVDSEL)
#define IS_RCC_PVDCLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_PVDCLKSOURCE_PCLK)  || \
                ((__SOURCE__) == RCC_PVDCLKSOURCE_LSC))
#endif /* RCC_CCIPR_PVDSEL */

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
#if defined(RCC_LSE_SUPPORT)
#define IS_RCC_LPTIM1CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_LPTIMCLKSOURCE_PCLK)|| \
                ((__SOURCE__) == RCC_LPTIMCLKSOURCE_LSI)  || \
                ((__SOURCE__) == RCC_LPTIMCLKSOURCE_LSE))
#else
#define IS_RCC_LPTIM1CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_LPTIMCLKSOURCE_PCLK)|| \
                ((__SOURCE__) == RCC_LPTIMCLKSOURCE_LSI))
#endif
#endif /* RCC_CCIPR_LPTIM1SEL */

#if defined(PY32F002APRE)
#define IS_RCC_PERIPHCLOCK(__SELECTION__)  \
  ((((__SELECTION__) & RCC_PERIPHCLK_LPTIM)   == RCC_PERIPHCLK_LPTIM))
#else
#define IS_RCC_PERIPHCLOCK(__SELECTION__)  \
  ((((__SELECTION__) & RCC_PERIPHCLK_COMP1)   == RCC_PERIPHCLK_COMP1)   || \
   (((__SELECTION__) & RCC_PERIPHCLK_COMP2)   == RCC_PERIPHCLK_COMP2)   || \
   (((__SELECTION__) & RCC_PERIPHCLK_PVD)     == RCC_PERIPHCLK_PVD)     || \
   (((__SELECTION__) & RCC_PERIPHCLK_LPTIM)   == RCC_PERIPHCLK_LPTIM)   || \
   (((__SELECTION__) & RCC_PERIPHCLK_RTC)     == RCC_PERIPHCLK_RTC))
#endif
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

#endif /* __PY32F0xx_HAL_RCC_EX_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
