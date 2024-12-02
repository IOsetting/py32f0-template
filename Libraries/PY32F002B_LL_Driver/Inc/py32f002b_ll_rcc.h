/**
  ******************************************************************************
  * @file    py32f002b_ll_rcc.h
  * @author  MCU Application Team
  * @brief   Header file of RCC LL module.
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
#ifndef __PY32F002B_LL_RCC_H
#define __PY32F002B_LL_RCC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx.h"

/** @addtogroup PY32F002B_LL_Driver
  * @{
  */

#if defined(RCC)

/** @defgroup RCC_LL RCC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup RCC_LL_Private_Variables RCC Private Variables
  * @{
  */
/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RCC_LL_Private_Macros RCC Private Macros
  * @{
  */
/**
  * @}
  */
#endif /*USE_FULL_LL_DRIVER*/

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RCC_LL_Exported_Types RCC Exported Types
  * @{
  */

/** @defgroup LL_ES_CLOCK_FREQ Clocks Frequency Structure
  * @{
  */

/**
  * @brief  RCC Clocks Frequency Structure
  */
typedef struct
{
  uint32_t SYSCLK_Frequency;        /*!< SYSCLK clock frequency */
  uint32_t HCLK_Frequency;          /*!< HCLK clock frequency */
  uint32_t PCLK1_Frequency;         /*!< PCLK1 clock frequency */
} LL_RCC_ClocksTypeDef;

/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCC_LL_Exported_Constants RCC Exported Constants
  * @{
  */

/** @defgroup RCC_LL_EC_OSC_VALUES Oscillator Values adaptation
  * @brief    Defines used to adapt values of different oscillators
  * @note     These values could be modified in the user environment according to
  *           HW set-up.
  * @{
  */
#if !defined  (HSE_VALUE)
#define HSE_VALUE    24000000U   /*!< Value of the HSE oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    24000000U  /*!< Value of the HSI oscillator in Hz */
#endif /* HSI_VALUE */

#if defined(RCC_LSE_SUPPORT)
#if !defined  (LSE_VALUE)
#define LSE_VALUE    32768U     /*!< Value of the LSE oscillator in Hz */
#endif /* LSE_VALUE */
#endif

#if !defined  (LSI_VALUE)
#define LSI_VALUE    32768U     /*!< Value of the LSI oscillator in Hz */
#endif /* LSI_VALUE */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with LL_RCC_WriteReg function
  * @{
  */
#define LL_RCC_CICR_LSIRDYC                RCC_CICR_LSIRDYC     /*!< LSI Ready Interrupt Clear */
#define LL_RCC_CICR_HSIRDYC                RCC_CICR_HSIRDYC     /*!< HSI Ready Interrupt Clear */
#if defined(RCC_LSE_SUPPORT)
#define LL_RCC_CICR_LSERDYC                RCC_CICR_LSERDYC     /*!< LSE Ready Interrupt Clear */
#define LL_RCC_CICR_LSECSSC                RCC_CICR_LSECSSC     /*!< LSE Clock Security System Interrupt Clear */
#endif

/**
  * @}
  */

/** @defgroup RCC_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_RCC_ReadReg function
  * @{
  */
#define LL_RCC_CIFR_LSIRDYF                RCC_CIFR_LSIRDYF     /*!< LSI Ready Interrupt flag */
#define LL_RCC_CIFR_HSIRDYF                RCC_CIFR_HSIRDYF     /*!< HSI Ready Interrupt flag */
#if defined(RCC_LSE_SUPPORT)
#define LL_RCC_CIFR_LSERDYF                RCC_CIFR_LSERDYF     /*!< LSE Ready Interrupt flag */
#define LL_RCC_CIFR_LSECSSF                RCC_CIFR_LSECSSF     /*!< LSE Clock Security System Interrupt flag */
#endif

#define LL_RCC_CSR_OBLRSTF                 RCC_CSR_OBLRSTF    /*!< OBL reset flag */
#define LL_RCC_CSR_PINRSTF                 RCC_CSR_PINRSTF    /*!< PIN reset flag */
#define LL_RCC_CSR_SFTRSTF                 RCC_CSR_SFTRSTF    /*!< Software Reset flag */
#define LL_RCC_CSR_IWDGRSTF                RCC_CSR_IWDGRSTF   /*!< Independent Watchdog reset flag */
#define LL_RCC_CSR_PWRRSTF                 RCC_CSR_PWRRSTF    /*!< BOR or POR/PDR reset flag */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_RCC_ReadReg and  LL_RCC_WriteReg functions
  * @{
  */
#define LL_RCC_CIER_LSIRDYIE               RCC_CIER_LSIRDYIE      /*!< LSI Ready Interrupt Enable */
#if defined(RCC_LSE_SUPPORT)
#define LL_RCC_CIER_LSERDYIE               RCC_CIER_LSERDYIE      /*!< LSE Ready Interrupt Enable */
#endif
#define LL_RCC_CIER_HSIRDYIE               RCC_CIER_HSIRDYIE      /*!< HSI Ready Interrupt Enable */
/**
  * @}
  */
  
#if defined(RCC_LSE_SUPPORT)
/** @defgroup RCC_LL_EC_LSEDRIVE  LSE oscillator drive capability
  * @{
  */
#define LL_RCC_LSEDRIVE_CLOSE              0x00000000                         /*!< LSE driving capability closed */
#define LL_RCC_LSEDRIVE_LOW                RCC_ECSCR_LSE_DRIVER_0             /*!< LSE lower driving capability */
#define LL_RCC_LSEDRIVE_MEDIUM             RCC_ECSCR_LSE_DRIVER_1             /*!< LSE medium driving capability */
#define LL_RCC_LSEDRIVE_HIGH               RCC_ECSCR_LSE_DRIVER               /*!< LSE higher driving capability */
/**
  * @}
  */
	
/** @defgroup RCC_LL_EC_LSESTARTUP  LSE oscillator startup time
  * @{
  */
#define LL_RCC_LSESTARTUP_DELAY_LOW              RCC_ECSCR_LSE_STARTUP         /*!< LSE Startup none delay */
#define LL_RCC_LSESTARTUP_DELAY_MEDIUM           RCC_ECSCR_LSE_STARTUP_0       /*!< LSE Startup short delay */
#define LL_RCC_LSESTARTUP_DELAY_HIGH             0x00000000                    /*!< LSE Startup long delay */
#define LL_RCC_LSESTARTUP_DELAY_VERY_HIGH        RCC_ECSCR_LSE_STARTUP_1       /*!< LSE Startup very long delay */
/**
  * @}
  */
#endif


#if defined(RCC_BDCR_LSCOSEL)
/** @defgroup RCC_LL_EC_LSCO_CLKSOURCE  LSCO Selection
  * @{
  */
#define LL_RCC_LSCO_CLKSOURCE_LSI          0x00000000U                 /*!< LSI selection for low speed clock  */
#define LL_RCC_LSCO_CLKSOURCE_LSE          RCC_BDCR_LSCOSEL            /*!< LSE selection for low speed clock  */
#endif
/**
  * @}
  */


/** @defgroup RCC_LL_EC_SYS_CLKSOURCE  System clock switch
  * @{
  */
#define LL_RCC_SYS_CLKSOURCE_HSISYS        0x00000000U                        /*!< HSISYS selection as system clock */
#define LL_RCC_SYS_CLKSOURCE_HSE           RCC_CFGR_SW_0                      /*!< HSE selection as system clock */
#define LL_RCC_SYS_CLKSOURCE_LSI           (RCC_CFGR_SW_1 | RCC_CFGR_SW_0)    /*!< LSI selection used as system clock */
#if defined(RCC_LSE_SUPPORT)
#define LL_RCC_SYS_CLKSOURCE_LSE           RCC_CFGR_SW_2                      /*!< LSE selection used as system clock */
#endif
/**
  * @}
  */

/** @defgroup RCC_LL_EC_SYS_CLKSOURCE_STATUS  System clock switch status
  * @{
  */
#define LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS 0x00000000U                         /*!< HSISYS used as system clock */
#define LL_RCC_SYS_CLKSOURCE_STATUS_HSE    RCC_CFGR_SWS_0                      /*!< HSE used as system clock */
#define LL_RCC_SYS_CLKSOURCE_STATUS_LSI    (RCC_CFGR_SWS_1 | RCC_CFGR_SWS_0)   /*!< LSI used as system clock */
#if defined(RCC_LSE_SUPPORT)
#define LL_RCC_SYS_CLKSOURCE_STATUS_LSE    RCC_CFGR_SWS_2                      /*!< LSE used as system clock */
#endif
/**
  * @}
  */

/** @defgroup RCC_LL_EC_SYSCLK_DIV  AHB prescaler
  * @{
  */
#define LL_RCC_SYSCLK_DIV_1                0x00000000U                                                             /*!< SYSCLK not divided */
#define LL_RCC_SYSCLK_DIV_2                RCC_CFGR_HPRE_3                                                         /*!< SYSCLK divided by 2 */
#define LL_RCC_SYSCLK_DIV_4                (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_0)                                     /*!< SYSCLK divided by 4 */
#define LL_RCC_SYSCLK_DIV_8                (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1)                                     /*!< SYSCLK divided by 8 */
#define LL_RCC_SYSCLK_DIV_16               (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0)                   /*!< SYSCLK divided by 16 */
#define LL_RCC_SYSCLK_DIV_64               (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2)                                     /*!< SYSCLK divided by 64 */
#define LL_RCC_SYSCLK_DIV_128              (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_0)                   /*!< SYSCLK divided by 128 */
#define LL_RCC_SYSCLK_DIV_256              (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1)                   /*!< SYSCLK divided by 256 */
#define LL_RCC_SYSCLK_DIV_512              (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0) /*!< SYSCLK divided by 512 */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_APB1_DIV  APB1 low-speed prescaler (APB1)
  * @{
  */
#define LL_RCC_APB1_DIV_1                  0x00000000U                                           /*!< HCLK not divided */
#define LL_RCC_APB1_DIV_2                  RCC_CFGR_PPRE_2                                       /*!< HCLK divided by 2 */
#define LL_RCC_APB1_DIV_4                  (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_0)                   /*!< HCLK divided by 4 */
#define LL_RCC_APB1_DIV_8                  (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_1)                   /*!< HCLK divided by 8 */
#define LL_RCC_APB1_DIV_16                 (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_1 | RCC_CFGR_PPRE_0) /*!< HCLK divided by 16 */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_HSI_DIV  HSI division factor
  * @{
  */
#define LL_RCC_HSI_DIV_1                  0x00000000U                                /*!< HSI not divided */
#define LL_RCC_HSI_DIV_2                  RCC_CR_HSIDIV_0                            /*!< HSI divided by 2 */
#define LL_RCC_HSI_DIV_4                  RCC_CR_HSIDIV_1                            /*!< HSI divided by 4 */
#define LL_RCC_HSI_DIV_8                  (RCC_CR_HSIDIV_1 | RCC_CR_HSIDIV_0)        /*!< HSI divided by 8 */
#define LL_RCC_HSI_DIV_16                 RCC_CR_HSIDIV_2                            /*!< HSI divided by 16 */
#define LL_RCC_HSI_DIV_32                 (RCC_CR_HSIDIV_2 | RCC_CR_HSIDIV_0)        /*!< HSI divided by 32 */
#define LL_RCC_HSI_DIV_64                 (RCC_CR_HSIDIV_2 | RCC_CR_HSIDIV_1)        /*!< HSI divided by 64 */
#define LL_RCC_HSI_DIV_128                RCC_CR_HSIDIV                              /*!< HSI divided by 128 */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_MCO1SOURCE  MCO1 SOURCE selection
  * @{
  */
#define LL_RCC_MCO1SOURCE_NOCLOCK          0x00000000U                            /*!< MCO output disabled, no clock on MCO */
#define LL_RCC_MCO1SOURCE_SYSCLK           RCC_CFGR_MCOSEL_0                      /*!< SYSCLK selection as MCO1 source */
#define LL_RCC_MCO1SOURCE_HSI              (RCC_CFGR_MCOSEL_0| RCC_CFGR_MCOSEL_1) /*!< HSI selection as MCO1 source */
#define LL_RCC_MCO1SOURCE_HSE              RCC_CFGR_MCOSEL_2                      /*!< HSE selection as MCO1 source */
#define LL_RCC_MCO1SOURCE_LSI              (RCC_CFGR_MCOSEL_1|RCC_CFGR_MCOSEL_2)  /*!< LSI selection as MCO1 source */
#if defined(RCC_LSE_SUPPORT)
#define LL_RCC_MCO1SOURCE_LSE              (RCC_CFGR_MCOSEL_0|RCC_CFGR_MCOSEL_1|RCC_CFGR_MCOSEL_2) /*!< LSE selection as MCO1 source */
#endif
/**
  * @}
  */

/** @defgroup RCC_LL_EC_MCO Peripheral MCO get clock source
  * @{
  */
#define LL_RCC_MCO1_CLKSOURCE             RCC_CFGR_MCOSEL /*!< MCO1 Clock source selection */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_MCO1_DIV  MCO1 prescaler
  * @{
  */
#define LL_RCC_MCO1_DIV_1                  0x00000000U                                                 /*!< MCO1 not divided */
#define LL_RCC_MCO1_DIV_2                  RCC_CFGR_MCOPRE_0                                           /*!< MCO1 divided by 2 */
#define LL_RCC_MCO1_DIV_4                  RCC_CFGR_MCOPRE_1                                           /*!< MCO1 divided by 4 */
#define LL_RCC_MCO1_DIV_8                  (RCC_CFGR_MCOPRE_1 | RCC_CFGR_MCOPRE_0)                     /*!< MCO1 divided by 8 */
#define LL_RCC_MCO1_DIV_16                 RCC_CFGR_MCOPRE_2                                           /*!< MCO1 divided by 16 */
#define LL_RCC_MCO1_DIV_32                 (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_0)                     /*!< MCO1 divided by 32 */
#define LL_RCC_MCO1_DIV_64                 (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_1)                     /*!< MCO1 divided by 64 */
#define LL_RCC_MCO1_DIV_128                (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_1 | RCC_CFGR_MCOPRE_0) /*!< MCO1 divided by 128 */
/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RCC_LL_EC_PERIPH_FREQUENCY Peripheral clock frequency
  * @{
  */
#define LL_RCC_PERIPH_FREQUENCY_NO        0x00000000U                 /*!< No clock enabled for the peripheral            */
#define LL_RCC_PERIPH_FREQUENCY_NA        0xFFFFFFFFU                 /*!< Frequency cannot be provided as external clock */
/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

#if defined(COMP1)
/** @defgroup RCC_LL_EC_COMPx_CLKSOURCE Peripheral COMP clock source selection
  * @{
  */
#define LL_RCC_COMP1_CLKSOURCE_PCLK1      (RCC_CCIPR_COMP1SEL | (0x00000000U >> 8U))                    /*!< PCLK1 selected as COMP1 clock */
#define LL_RCC_COMP1_CLKSOURCE_LSC        (RCC_CCIPR_COMP1SEL | (RCC_CCIPR_COMP1SEL >> 8U))             /*!< LSC selected as COMP1 clock */
#if defined(COMP2)
#define LL_RCC_COMP2_CLKSOURCE_PCLK1      (RCC_CCIPR_COMP2SEL | (0x00000000U >> 8U))                    /*!< PCLK1 selected as COMP2 clock */
#define LL_RCC_COMP2_CLKSOURCE_LSC        (RCC_CCIPR_COMP2SEL | (RCC_CCIPR_COMP2SEL >> 8U))             /*!< LSC selected as COMP2 clock */
#endif
/**
  * @}
  */
#endif /* COMP1 && COMP2 */

#if defined(RCC_CCIPR_LPTIMSEL)
/** @defgroup RCC_LL_EC_LPTIMx_CLKSOURCE Peripheral LPTIM clock source selection
  * @{
  */
#define LL_RCC_LPTIM1_CLKSOURCE_NONE       RCC_CCIPR_LPTIMSEL_1          /*!< No clock used as LPTIM1 clock */
#define LL_RCC_LPTIM1_CLKSOURCE_PCLK1      0x00000000U                   /*!< PCLK1 selected as LPTIM1 clock */
#define LL_RCC_LPTIM1_CLKSOURCE_LSI        RCC_CCIPR_LPTIMSEL_0          /*!< LSI selected as LPTIM1 clock */
#if defined(RCC_LSE_SUPPORT)
#define LL_RCC_LPTIM1_CLKSOURCE_LSE        RCC_CCIPR_LPTIMSEL            /*!< LSE selected as LPTIM1 clock */
#endif
/**
  * @}
  */
#endif /* RCC_CCIPR_LPTIMSEL */

#if defined(COMP1)
/** @defgroup RCC_LL_EC_COMP Peripheral COMP get clock source
  * @{
  */
#define LL_RCC_COMP1_CLKSOURCE            RCC_CCIPR_COMP1SEL /*!< COMP1 Clock source selection */
#if defined(COMP2)
#define LL_RCC_COMP2_CLKSOURCE            RCC_CCIPR_COMP2SEL /*!< COMP2 Clock source selection */
#endif /* COMP2 */
/**
  * @}
  */
#endif /* COMP1 */

#if defined(RCC_CCIPR_LPTIMSEL)
/** @defgroup RCC_LL_EC_LPTIM Peripheral LPTIM get clock source
  * @{
  */
#define LL_RCC_LPTIM1_CLKSOURCE            RCC_CCIPR_LPTIMSEL /*!< LPTIM1 Clock source selection */
/**
  * @}
  */
#endif /* RCC_CCIPR_LPTIMSEL */

/** @defgroup RCC_HSI_EC_Calibration HSI Calibration
* @{
*/
#define LL_RCC_HSICALIBRATION_24MHz        ((*(uint32_t *)(0x1FFF0100)) & 0xFFFF)  /*!< 24MHz HSI calibration trimming value */
#if defined(RCC_HSI48M_SUPPORT)
#define LL_RCC_HSICALIBRATION_48MHz        ((*(uint32_t *)(0x1FFF0104)) & 0xFFFF)  /*!< 48MHz HSI calibration trimming value */
#endif
/**
  * @}
  */
	
/** @defgroup RCC_LSI_EC_Calibration LSI Calibration
* @{
*/
#define LL_RCC_LSICALIBRATION_32768Hz        ((*(uint32_t *)(0x1FFF0144)) & 0x1FF)  /*!< 32.768KHz LSI calibration trimming value */
#define LL_RCC_LSICALIBRATION_38400Hz        ((*(uint32_t *)(0x1FFF0148)) & 0x1FF)  /*!< 38.4KHz LSI calibration trimming value */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RCC_LL_Exported_Macros RCC Exported Macros
  * @{
  */

/** @defgroup RCC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RCC register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_RCC_WriteReg(__REG__, __VALUE__) WRITE_REG((RCC->__REG__), (__VALUE__))

/**
  * @brief  Read a value in RCC register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_RCC_ReadReg(__REG__) READ_REG(RCC->__REG__)
/**
  * @}
  */

/** @defgroup RCC_LL_EM_CALC_FREQ Calculate frequencies
  * @{
  */

/**
  * @brief  Helper macro to calculate the HCLK frequency
  * @param  __SYSCLKFREQ__ SYSCLK frequency
  * @param  __AHBPRESCALER__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  * @retval HCLK clock frequency (in Hz)
  */
#define __LL_RCC_CALC_HCLK_FREQ(__SYSCLKFREQ__,__AHBPRESCALER__)  \
  ((__SYSCLKFREQ__) >> (AHBPrescTable[((__AHBPRESCALER__) & RCC_CFGR_HPRE) >>  RCC_CFGR_HPRE_Pos] & 0x1FU))

/**
  * @brief  Helper macro to calculate the PCLK1 frequency (APB1)
  * @param  __HCLKFREQ__ HCLK frequency
  * @param  __APB1PRESCALER__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  * @retval PCLK1 clock frequency (in Hz)
  */
#define __LL_RCC_CALC_PCLK1_FREQ(__HCLKFREQ__, __APB1PRESCALER__)  \
  ((__HCLKFREQ__) >> (APBPrescTable[(__APB1PRESCALER__) >>  RCC_CFGR_PPRE_Pos] & 0x1FU))

/**
  * @brief  Helper macro to calculate the HSISYS frequency
  * @param  __HSIDIV__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_HSI_DIV_1
  *         @arg @ref LL_RCC_HSI_DIV_2
  *         @arg @ref LL_RCC_HSI_DIV_4
  *         @arg @ref LL_RCC_HSI_DIV_8
  *         @arg @ref LL_RCC_HSI_DIV_16
  *         @arg @ref LL_RCC_HSI_DIV_32
  *         @arg @ref LL_RCC_HSI_DIV_64
  *         @arg @ref LL_RCC_HSI_DIV_128
  * @retval HSISYS clock frequency (in Hz)
  */
#define __LL_RCC_CALC_HSI_FREQ(__HSIDIV__)  \
  (HSIFreqTable[(RCC->ICSCR & RCC_ICSCR_HSI_FS) >> RCC_ICSCR_HSI_FS_Pos] / (1U << ((__HSIDIV__)>> RCC_CR_HSIDIV_Pos)))

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup RCC_LL_Exported_Functions RCC Exported Functions
  * @{
  */

/** @defgroup RCC_LL_EF_HSI HSI
  * @{
  */

/**
  * @brief  Enable HSI oscillator
  * @rmtoll CR           HSION         LL_RCC_HSI_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSI_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSION);
}

/**
  * @brief  Disable HSI oscillator
  * @rmtoll CR           HSION         LL_RCC_HSI_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSI_Disable(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

/**
  * @brief  Check if HSI clock is ready
  * @rmtoll CR           HSIRDY        LL_RCC_HSI_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_HSI_IsReady(void)
{
  return ((READ_BIT(RCC->CR, RCC_CR_HSIRDY) == (RCC_CR_HSIRDY)) ? 1UL : 0UL);
}

/**
  * @brief  Set HSI Calibration trimming
  * @param  Value Between Min_Data = 0 and Max_Data = 0x1FFF
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSI_SetCalibTrimming(uint32_t Value)
{
  MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_TRIM, Value << RCC_ICSCR_HSI_TRIM_Pos);
}

/**
  * @brief  Get HSI Calibration trimming
  * @rmtoll ICSCR        HSITRIM       LL_RCC_HSI_GetCalibTrimming
  * @retval Between Min_Data = 0 and Max_Data = 0x1FFF
  */
__STATIC_INLINE uint32_t LL_RCC_HSI_GetCalibTrimming(void)
{
  return (uint32_t)(READ_BIT(RCC->ICSCR, RCC_ICSCR_HSI_TRIM) >> RCC_ICSCR_HSI_TRIM_Pos);
}

/**
  * @brief  Set HSI Calibration Frequency
  * @param  Value This parameter can be one of the following values:
  *         @arg @ref LL_RCC_HSICALIBRATION_24MHz
  *         @arg @ref LL_RCC_HSICALIBRATION_48MHz
  * @note   Depending on devices and packages, some calibration values may not be available.
  *         Refer to device datasheet for calibration values availability.
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSI_SetCalibFreq(uint32_t Value)
{
  MODIFY_REG(RCC->ICSCR, (RCC_ICSCR_HSI_FS | RCC_ICSCR_HSI_TRIM), (Value << RCC_ICSCR_HSI_TRIM_Pos));
  SystemCoreClockUpdate();
}

/**
  * @brief  Get HSI Frequency
  * @retval HSI clock frequency (in Hz)
  */
__STATIC_INLINE uint32_t LL_RCC_HSI_GetFreq(void)
{
  return (uint32_t)HSIFreqTable[(RCC->ICSCR & RCC_ICSCR_HSI_FS) >> RCC_ICSCR_HSI_FS_Pos];
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_HSE HSE
  * @{
  */
/**
  * @brief  Enable external clock source (HSE bypass).
  * @rmtoll CR         HSEEN        LL_RCC_HSE_EnableBypass
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSE_EnableBypass(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSEEN);
}

/**
  * @brief  Disable external clock source (HSE bypass).
  * @rmtoll CR         HSEEN        LL_RCC_HSE_DisableBypass
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSE_DisableBypass(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_HSEEN);
}

/**
  * @brief  Check if HSE Bypass clock is on
  * @rmtoll CR           HSEEN        LL_RCC_HSE_IsBypass
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_HSE_IsBypass(void)
{
  return ((READ_BIT(RCC->CR, RCC_CR_HSEEN) == (RCC_CR_HSEEN)) ? 1UL : 0UL);
}
/**
  * @}
  */

#if defined(RCC_LSE_SUPPORT)
/** @defgroup RCC_LL_EF_LSE LSE
  * @{
  */

/**
  * @brief  Enable  Low Speed External (LSE) crystal.
  * @rmtoll BDCR         LSEON         LL_RCC_LSE_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_Enable(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
}

/**
  * @brief  Disable  Low Speed External (LSE) crystal.
  * @rmtoll BDCR         LSEON         LL_RCC_LSE_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_Disable(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
}

/**
  * @brief  Enable external clock source (LSE bypass).
  * @rmtoll BDCR         LSEBYP        LL_RCC_LSE_EnableBypass
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_EnableBypass(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
}

/**
  * @brief  Disable external clock source (LSE bypass).
  * @rmtoll BDCR         LSEBYP        LL_RCC_LSE_DisableBypass
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_DisableBypass(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
}

/**
  * @brief  Set LSE oscillator drive capability
  * @note The oscillator is in Xtal mode when it is not in bypass mode.
  * @param  LSEDrive This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LSEDRIVE_CLOSE
  *         @arg @ref LL_RCC_LSEDRIVE_LOW
  *         @arg @ref LL_RCC_LSEDRIVE_MEDIUM
  *         @arg @ref LL_RCC_LSEDRIVE_HIGH
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_SetDriveCapability(uint32_t LSEDrive)
{
  MODIFY_REG(RCC->ECSCR, RCC_ECSCR_LSE_DRIVER, LSEDrive);
}

/**
  * @brief  Get LSE oscillator drive capability
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_LSEDRIVE_CLOSE
  *         @arg @ref LL_RCC_LSEDRIVE_LOW
  *         @arg @ref LL_RCC_LSEDRIVE_MEDIUM
  *         @arg @ref LL_RCC_LSEDRIVE_HIGH
  */
__STATIC_INLINE uint32_t LL_RCC_LSE_GetDriveCapability(void)
{
  return (uint32_t)(READ_BIT(RCC->ECSCR, RCC_ECSCR_LSE_DRIVER));
}

/**
  * @brief  Set LSE startup time
  * @param  LSEStartup This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LSESTARTUP_DELAY_LOW      
  *         @arg @ref LL_RCC_LSESTARTUP_DELAY_MEDIUM   
  *         @arg @ref LL_RCC_LSESTARTUP_DELAY_HIGH     
  *         @arg @ref LL_RCC_LSESTARTUP_DELAY_VERY_HIGH
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_SetStartupTime(uint32_t LSEStartup)
{
  MODIFY_REG(RCC->ECSCR, RCC_ECSCR_LSE_STARTUP, LSEStartup);
}

/**
  * @brief  Get LSE startup time
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_LSESTARTUP_DELAY_LOW      
  *         @arg @ref LL_RCC_LSESTARTUP_DELAY_MEDIUM   
  *         @arg @ref LL_RCC_LSESTARTUP_DELAY_HIGH     
  *         @arg @ref LL_RCC_LSESTARTUP_DELAY_VERY_HIGH
  */
__STATIC_INLINE uint32_t LL_RCC_LSE_GetStartupTime(void)
{
  return (uint32_t)(READ_BIT(RCC->ECSCR, RCC_ECSCR_LSE_STARTUP));
}

/**
  * @brief  Enable Clock security system on LSE.
  * @rmtoll BDCR         LSECSSON      LL_RCC_LSE_EnableCSS
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_EnableCSS(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_LSECSSON);
}

/**
  * @brief  Disable Clock security system on LSE.
  * @note Clock security system can be disabled only after a LSE
  *       failure detection. In that case it MUST be disabled by software.
  * @rmtoll BDCR         LSECSSON      LL_RCC_LSE_DisableCSS
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_DisableCSS(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSECSSON);
}


/**
  * @brief  Check if LSE oscillator Ready
  * @rmtoll BDCR         LSERDY        LL_RCC_LSE_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_LSE_IsReady(void)
{
  return ((READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY) == (RCC_BDCR_LSERDY)) ? 1UL : 0UL);
}

/**
  * @brief  Check if CSS on LSE failure Detection
  * @rmtoll BDCR         LSECSSD       LL_RCC_LSE_IsCSSDetected
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_LSE_IsCSSDetected(void)
{
  return ((READ_BIT(RCC->BDCR, RCC_BDCR_LSECSSD) == (RCC_BDCR_LSECSSD)) ? 1UL : 0UL);
}

/**
  * @}
  */
#endif

/** @defgroup RCC_LL_EF_LSI LSI
  * @{
  */

/**
  * @brief  Enable LSI Oscillator
  * @rmtoll CSR          LSION         LL_RCC_LSI_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSI_Enable(void)
{
  SET_BIT(RCC->CSR, RCC_CSR_LSION);
}

/**
  * @brief  Disable LSI Oscillator
  * @rmtoll CSR          LSION         LL_RCC_LSI_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSI_Disable(void)
{
  CLEAR_BIT(RCC->CSR, RCC_CSR_LSION);
}

/**
  * @brief  Check if LSI is Ready
  * @rmtoll CSR          LSIRDY        LL_RCC_LSI_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_LSI_IsReady(void)
{
  return ((READ_BIT(RCC->CSR, RCC_CSR_LSIRDY) == (RCC_CSR_LSIRDY)) ? 1UL : 0UL);
}

/**
  * @brief  Set LSI Calibration trimming
  * @param  Value This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LSICALIBRATION_32768Hz
  *         @arg @ref LL_RCC_LSICALIBRATION_38400Hz
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSI_SetCalibTrimming(uint32_t Value)
{
  MODIFY_REG(RCC->ICSCR, RCC_ICSCR_LSI_TRIM, (Value << RCC_ICSCR_LSI_TRIM_Pos));
}

/**
  * @brief  Get LSI Calibration trimming
  * @rmtoll ICSCR        LSI_TRIM       LL_RCC_LSI_GetCalibTrimming
  * @retval Between Min_Data = 0 and Max_Data = 0x1FFF
  */
__STATIC_INLINE uint32_t LL_RCC_LSI_GetCalibTrimming(void)
{
  return (uint32_t)(READ_BIT(RCC->ICSCR, RCC_ICSCR_LSI_TRIM) >> RCC_ICSCR_LSI_TRIM_Pos);
}

/**
  * @brief  Get LSI Frequency
  * @retval HSI clock frequency (in Hz)
  */
__STATIC_INLINE uint32_t LL_RCC_LSI_GetFreq(void)
{
  return ((LL_RCC_LSI_GetCalibTrimming() == LL_RCC_LSICALIBRATION_32768Hz) ? 32768UL : \
		     ((LL_RCC_LSI_GetCalibTrimming() == LL_RCC_LSICALIBRATION_38400Hz) ? 38400UL : 0));
}

/**
  * @}
  */
#if defined(RCC_BDCR_LSCOEN)
/** @defgroup RCC_LL_EF_LSCO LSCO
  * @{
  */

/**
  * @brief  Enable Low speed clock
  * @rmtoll BDCR         LSCOEN        LL_RCC_LSCO_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSCO_Enable(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_LSCOEN);
}

/**
  * @brief  Disable Low speed clock
  * @rmtoll BDCR         LSCOEN        LL_RCC_LSCO_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSCO_Disable(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSCOEN);
}
#if defined(RCC_BDCR_LSCOSEL)
/**
  * @brief  Configure Low speed clock selection
  * @rmtoll BDCR         LSCOSEL       LL_RCC_LSCO_SetSource
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LSCO_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_LSCO_CLKSOURCE_LSE
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSCO_SetSource(uint32_t Source)
{
  MODIFY_REG(RCC->BDCR, RCC_BDCR_LSCOSEL, Source);
}

/**
  * @brief  Get Low speed clock selection
  * @rmtoll BDCR         LSCOSEL       LL_RCC_LSCO_GetSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_LSCO_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_LSCO_CLKSOURCE_LSE
  */
__STATIC_INLINE uint32_t LL_RCC_LSCO_GetSource(void)
{
  return (uint32_t)(READ_BIT(RCC->BDCR, RCC_BDCR_LSCOSEL));
}

/**
  * @}
  */
#endif
#endif
/** @defgroup RCC_LL_EF_System System
  * @{
  */

/**
  * @brief  Configure the system clock source
  * @rmtoll CFGR         SW            LL_RCC_SetSysClkSource
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_HSISYS
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_HSE
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_LSE
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetSysClkSource(uint32_t Source)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, Source);
}

/**
  * @brief  Get the system clock source
  * @rmtoll CFGR         SWS           LL_RCC_GetSysClkSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_HSE
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_LSI
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_LSE
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  */
__STATIC_INLINE uint32_t LL_RCC_GetSysClkSource(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_SWS));
}

/**
  * @brief  Set AHB prescaler
  * @rmtoll CFGR         HPRE          LL_RCC_SetAHBPrescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetAHBPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, Prescaler);
}

/**
  * @brief  Set APB1 prescaler
  * @rmtoll CFGR         PPRE         LL_RCC_SetAPB1Prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetAPB1Prescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, Prescaler);
}

/**
  * @brief  Set HSI division factor
  * @rmtoll CR         HSIDIV          LL_RCC_SetHSIDiv
  * @note  HSIDIV parameter is only applied to SYSCLK_Frequency when HSI is used as
  * system clock source.
  * @param  HSIDiv  This parameter can be one of the following values:
  *         @arg @ref LL_RCC_HSI_DIV_1
  *         @arg @ref LL_RCC_HSI_DIV_2
  *         @arg @ref LL_RCC_HSI_DIV_4
  *         @arg @ref LL_RCC_HSI_DIV_8
  *         @arg @ref LL_RCC_HSI_DIV_16
  *         @arg @ref LL_RCC_HSI_DIV_32
  *         @arg @ref LL_RCC_HSI_DIV_64
  *         @arg @ref LL_RCC_HSI_DIV_128
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetHSIDiv(uint32_t HSIDiv)
{
  MODIFY_REG(RCC->CR, RCC_CR_HSIDIV, HSIDiv);
}
/**
  * @brief  Get AHB prescaler
  * @rmtoll CFGR         HPRE          LL_RCC_GetAHBPrescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  */
__STATIC_INLINE uint32_t LL_RCC_GetAHBPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_HPRE));
}

/**
  * @brief  Get APB1 prescaler
  * @rmtoll CFGR         PPRE         LL_RCC_GetAPB1Prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  */
__STATIC_INLINE uint32_t LL_RCC_GetAPB1Prescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PPRE));
}

/**
  * @brief  Get HSI division factor
  * @rmtoll CR         HSIDIV         LL_RCC_GetHSIDiv
  * @note  HSIDIV parameter is only applied to SYSCLK_Frequency when HSI is used as
  * system clock source.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_HSI_DIV_1
  *         @arg @ref LL_RCC_HSI_DIV_2
  *         @arg @ref LL_RCC_HSI_DIV_4
  *         @arg @ref LL_RCC_HSI_DIV_8
  *         @arg @ref LL_RCC_HSI_DIV_16
  *         @arg @ref LL_RCC_HSI_DIV_32
  *         @arg @ref LL_RCC_HSI_DIV_64
  *         @arg @ref LL_RCC_HSI_DIV_128
  */
__STATIC_INLINE uint32_t LL_RCC_GetHSIDiv(void)
{
  return (uint32_t)(READ_BIT(RCC->CR, RCC_CR_HSIDIV));
}
/**
  * @}
  */

/** @defgroup RCC_LL_EF_MCO MCO
  * @{
  */

/**
  * @brief  Configure MCOx
  * @rmtoll CFGR         MCOSEL        LL_RCC_ConfigMCO\n
  *         CFGR         MCOPRE        LL_RCC_ConfigMCO
  * @param  MCOxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_MCO1SOURCE_NOCLOCK
  *         @arg @ref LL_RCC_MCO1SOURCE_SYSCLK
  *         @arg @ref LL_RCC_MCO1SOURCE_HSI
  *         @arg @ref LL_RCC_MCO1SOURCE_HSE
  *         @arg @ref LL_RCC_MCO1SOURCE_LSI
  *         @arg @ref LL_RCC_MCO1SOURCE_LSE
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  * @param  MCOxPrescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_MCO1_DIV_1
  *         @arg @ref LL_RCC_MCO1_DIV_2
  *         @arg @ref LL_RCC_MCO1_DIV_4
  *         @arg @ref LL_RCC_MCO1_DIV_8
  *         @arg @ref LL_RCC_MCO1_DIV_16
  *         @arg @ref LL_RCC_MCO1_DIV_32
  *         @arg @ref LL_RCC_MCO1_DIV_64
  *         @arg @ref LL_RCC_MCO1_DIV_128
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ConfigMCO(uint32_t MCOxSource, uint32_t MCOxPrescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE, MCOxSource | MCOxPrescaler);
}

/**
  * @brief  Get MCO clock source
  * @rmtoll CFGR         MCOSEL       LL_RCC_GetMCOClockSource
  * @param  MCOx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_MCO1_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_MCO1SOURCE_NOCLOCK
  *         @arg @ref LL_RCC_MCO1SOURCE_SYSCLK
  *         @arg @ref LL_RCC_MCO1SOURCE_HSI
  *         @arg @ref LL_RCC_MCO1SOURCE_HSE
  *         @arg @ref LL_RCC_MCO1SOURCE_LSI
  *         @arg @ref LL_RCC_MCO1SOURCE_LSE
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  */
__STATIC_INLINE uint32_t LL_RCC_GetMCOClockSource(uint32_t MCOx)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, MCOx));
}

/**
  * @brief  Get MCO division factor
  * @rmtoll CFGR         MCOPRE        LL_RCC_GetMCODiv
  * @param  MCOx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_MCO1_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_MCO1_DIV_1
  *         @arg @ref LL_RCC_MCO1_DIV_2
  *         @arg @ref LL_RCC_MCO1_DIV_4
  *         @arg @ref LL_RCC_MCO1_DIV_8
  *         @arg @ref LL_RCC_MCO1_DIV_16
  *         @arg @ref LL_RCC_MCO1_DIV_32
  *         @arg @ref LL_RCC_MCO1_DIV_64
  *         @arg @ref LL_RCC_MCO1_DIV_128
  */
__STATIC_INLINE uint32_t LL_RCC_GetMCODiv(uint32_t MCOx)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_MCOPRE));
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_Peripheral_Clock_Source Peripheral Clock Source
  * @{
  */

#if defined(COMP1)
/**
  * @brief  Configure COMPx clock source
  * @rmtoll CCIPR        COMPxSEL     LL_RCC_SetCOMPClockSource
  * @param  COMPxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_COMP1_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_COMP1_CLKSOURCE_LSC
  *         @arg @ref LL_RCC_COMP2_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_COMP2_CLKSOURCE_LSC
  * @note   Depending on devices and packages,some COMP may not be available.
  *         Refer to device datasheet for COMP availability.
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetCOMPClockSource(uint32_t COMPxSource)
{
  register uint32_t regTmp1 = (RCC->CCIPR & 0x0000FF00U) & (~(COMPxSource & 0x0000FF00U));
  regTmp1 = regTmp1 | (regTmp1 >> 2);
  register uint32_t regTmp2 = ((COMPxSource & 0xFFU) | ((COMPxSource & 0xFFU) >> 2)) << 8U;
  MODIFY_REG(RCC->CCIPR, (COMPxSource & 0x0000FF00U), (regTmp1 | regTmp2));
}
#endif /* COMP1 */

#if defined(COMP1)
/**
  * @brief  Get COMPx clock source
  * @rmtoll CCIPR        COMPxSEL     LL_RCC_GetCOMPClockSource
  * @param  COMPx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_COMP1_CLKSOURCE
  *         @arg @ref LL_RCC_COMP2_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_COMP1_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_COMP1_CLKSOURCE_LSC
  *         @arg @ref LL_RCC_COMP2_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_COMP2_CLKSOURCE_LSC
  * @note   Depending on devices and packages,some COMP may not be available.
  *         Refer to device datasheet for COMP availability.
  */
__STATIC_INLINE uint32_t LL_RCC_GetCOMPClockSource(uint32_t COMPx)
{
  return (uint32_t)((READ_BIT(RCC->CCIPR, COMPx) >> 8U) | COMPx);
}
#endif /* COMP1 */

#if defined(RCC_CCIPR_LPTIMSEL)
/**
  * @brief  Configure LPTIMx clock source
  * @rmtoll CCIPR        LPTIMxSEL     LL_RCC_SetLPTIMClockSource
  * @param  LPTIMxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_NONE
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_LSE
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetLPTIMClockSource(uint32_t LPTIMxSource)
{
  register uint32_t regTmp1 = (RCC->CCIPR & 0x0000FF00U);
  register uint32_t regTmp2 = ((RCC->CCIPR & 0x0000FF00U) >> 2);
  register uint32_t regTmp = regTmp1 | regTmp2;
  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIMSEL, (LPTIMxSource | regTmp));
}

/**
  * @brief  Get LPTIMx clock source
  * @rmtoll CCIPR        LPTIMxSEL     LL_RCC_GetLPTIMClockSource
  * @param  LPTIMx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_NONE
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_LSE
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  */
__STATIC_INLINE uint32_t LL_RCC_GetLPTIMClockSource(uint32_t LPTIMx)
{
  return (uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_LPTIMSEL));
}
#endif /* RCC_CCIPR_LPTIMSEL */

/**
  * @}
  */

/** @defgroup RCC_LL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Clear LSI ready interrupt flag
  * @rmtoll CICR         LSIRDYC       LL_RCC_ClearFlag_LSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_LSIRDY(void)
{
  SET_BIT(RCC->CICR, RCC_CICR_LSIRDYC);
}

#if defined(RCC_LSE_SUPPORT)
/**
  * @brief  Clear LSE ready interrupt flag
  * @rmtoll CICR         LSERDYC       LL_RCC_ClearFlag_LSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_LSERDY(void)
{
  SET_BIT(RCC->CICR, RCC_CICR_LSERDYC);
}
#endif

/**
  * @brief  Clear HSI ready interrupt flag
  * @rmtoll CICR         HSIRDYC       LL_RCC_ClearFlag_HSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_HSIRDY(void)
{
  SET_BIT(RCC->CICR, RCC_CICR_HSIRDYC);
}

#if defined(RCC_LSE_SUPPORT)
/**
  * @brief  Clear LSE Clock security system interrupt flag
  * @rmtoll CICR         LSECSSC       LL_RCC_ClearFlag_LSECSS
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_LSECSS(void)
{
  SET_BIT(RCC->CICR, RCC_CICR_LSECSSC);
}
#endif

/**
  * @brief  Check if LSI ready interrupt occurred or not
  * @rmtoll CIFR         LSIRDYF       LL_RCC_IsActiveFlag_LSIRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_LSIRDY(void)
{
  return ((READ_BIT(RCC->CIFR, RCC_CIFR_LSIRDYF) == (RCC_CIFR_LSIRDYF)) ? 1UL : 0UL);
}
#if defined(RCC_LSE_SUPPORT)
/**
  * @brief  Check if LSE ready interrupt occurred or not
  * @rmtoll CIFR         LSERDYF       LL_RCC_IsActiveFlag_LSERDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_LSERDY(void)
{
  return ((READ_BIT(RCC->CIFR, RCC_CIFR_LSERDYF) == (RCC_CIFR_LSERDYF)) ? 1UL : 0UL);
}
#endif
/**
  * @brief  Check if HSI ready interrupt occurred or not
  * @rmtoll CIFR         HSIRDYF       LL_RCC_IsActiveFlag_HSIRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_HSIRDY(void)
{
  return ((READ_BIT(RCC->CIFR, RCC_CIFR_HSIRDYF) == (RCC_CIFR_HSIRDYF)) ? 1UL : 0UL);
}

#if defined(RCC_LSE_SUPPORT)
/**
  * @brief  Check if LSE Clock security system interrupt occurred or not
  * @rmtoll CIFR         LSECSSF       LL_RCC_IsActiveFlag_LSECSS
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_LSECSS(void)
{
  return ((READ_BIT(RCC->CIFR, RCC_CIFR_LSECSSF) == (RCC_CIFR_LSECSSF)) ? 1UL : 0UL);
}
#endif

/**
  * @brief  Check if RCC flag Independent Watchdog reset is set or not.
  * @rmtoll CSR          IWDGRSTF      LL_RCC_IsActiveFlag_IWDGRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_IWDGRST(void)
{
  return ((READ_BIT(RCC->CSR, RCC_CSR_IWDGRSTF) == (RCC_CSR_IWDGRSTF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if RCC flag Option byte reset is set or not.
  * @rmtoll CSR          OBLRSTF       LL_RCC_IsActiveFlag_OBLRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_OBLRST(void)
{
  return ((READ_BIT(RCC->CSR, RCC_CSR_OBLRSTF) == (RCC_CSR_OBLRSTF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if RCC flag Pin reset is set or not.
  * @rmtoll CSR          PINRSTF       LL_RCC_IsActiveFlag_PINRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_PINRST(void)
{
  return ((READ_BIT(RCC->CSR, RCC_CSR_PINRSTF) == (RCC_CSR_PINRSTF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if RCC flag Software reset is set or not.
  * @rmtoll CSR          SFTRSTF       LL_RCC_IsActiveFlag_SFTRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_SFTRST(void)
{
  return ((READ_BIT(RCC->CSR, RCC_CSR_SFTRSTF) == (RCC_CSR_SFTRSTF)) ? 1UL : 0UL);
}

/**
  * @brief  Check if RCC flag BOR or POR/PDR reset is set or not.
  * @rmtoll CSR          PWRRSTF       LL_RCC_IsActiveFlag_PWRRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_PWRRST(void)
{
  return ((READ_BIT(RCC->CSR, RCC_CSR_PWRRSTF) == (RCC_CSR_PWRRSTF)) ? 1UL : 0UL);
}

/**
  * @brief  Set RMVF bit to clear the reset flags.
  * @rmtoll CSR          RMVF          LL_RCC_ClearResetFlags
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearResetFlags(void)
{
  SET_BIT(RCC->CSR, RCC_CSR_RMVF);
}

/**
  * @brief  Enable NRST filter
  * @rmtoll CSR          NRST_FLTDIS         LL_RCC_EnableNRSTFilter
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableNRSTFilter(void)
{
  CLEAR_BIT(RCC->CSR, RCC_CSR_NRST_FLTDIS);
}

/**
  * @brief  Disable NRST filter
  * @rmtoll CSR          NRST_FLTDIS         LL_RCC_DisableNRSTFilter
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableNRSTFilter(void)
{
  SET_BIT(RCC->CSR, RCC_CSR_NRST_FLTDIS);
}

/**
  * @brief  Check if NRST filter is enable
  * @rmtoll CSR          NRST_FLTDIS         LL_RCC_IsEnableNRSTFilter
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnableNRSTFilter(void)
{
  return ((READ_BIT(RCC->CSR, RCC_CSR_NRST_FLTDIS) == (RCC_CSR_NRST_FLTDIS)) ? 0UL : 1UL);
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_IT_Management IT Management
  * @{
  */

/**
  * @brief  Enable LSI ready interrupt
  * @rmtoll CIER         LSIRDYIE      LL_RCC_EnableIT_LSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_LSIRDY(void)
{
  SET_BIT(RCC->CIER, RCC_CIER_LSIRDYIE);
}
#if defined(RCC_LSE_SUPPORT)
/**
  * @brief  Enable LSE ready interrupt
  * @rmtoll CIER         LSERDYIE      LL_RCC_EnableIT_LSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_LSERDY(void)
{
  SET_BIT(RCC->CIER, RCC_CIER_LSERDYIE);
}
#endif
/**
  * @brief  Enable HSI ready interrupt
  * @rmtoll CIER         HSIRDYIE      LL_RCC_EnableIT_HSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_HSIRDY(void)
{
  SET_BIT(RCC->CIER, RCC_CIER_HSIRDYIE);
}

/**
  * @brief  Disable LSI ready interrupt
  * @rmtoll CIER         LSIRDYIE      LL_RCC_DisableIT_LSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_LSIRDY(void)
{
  CLEAR_BIT(RCC->CIER, RCC_CIER_LSIRDYIE);
}
#if defined(RCC_LSE_SUPPORT)
/**
  * @brief  Disable LSE ready interrupt
  * @rmtoll CIER         LSERDYIE      LL_RCC_DisableIT_LSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_LSERDY(void)
{
  CLEAR_BIT(RCC->CIER, RCC_CIER_LSERDYIE);
}
#endif
/**
  * @brief  Disable HSI ready interrupt
  * @rmtoll CIER         HSIRDYIE      LL_RCC_DisableIT_HSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_HSIRDY(void)
{
  CLEAR_BIT(RCC->CIER, RCC_CIER_HSIRDYIE);
}

/**
  * @brief  Checks if LSI ready interrupt source is enabled or disabled.
  * @rmtoll CIER         LSIRDYIE      LL_RCC_IsEnabledIT_LSIRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_LSIRDY(void)
{
  return ((READ_BIT(RCC->CIER, RCC_CIER_LSIRDYIE) == (RCC_CIER_LSIRDYIE)) ? 1UL : 0UL);
}
#if defined(RCC_LSE_SUPPORT)
/**
  * @brief  Checks if LSE ready interrupt source is enabled or disabled.
  * @rmtoll CIER         LSERDYIE      LL_RCC_IsEnabledIT_LSERDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_LSERDY(void)
{
  return ((READ_BIT(RCC->CIER, RCC_CIER_LSERDYIE) == (RCC_CIER_LSERDYIE)) ? 1UL : 0UL);
}
#endif
/**
  * @brief  Checks if HSI ready interrupt source is enabled or disabled.
  * @rmtoll CIER         HSIRDYIE      LL_RCC_IsEnabledIT_HSIRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_HSIRDY(void)
{
  return ((READ_BIT(RCC->CIER, RCC_CIER_HSIRDYIE) == (RCC_CIER_HSIRDYIE)) ? 1UL : 0UL);
}

/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RCC_LL_EF_Init De-initialization function
  * @{
  */
ErrorStatus LL_RCC_DeInit(void);
/**
  * @}
  */

/** @defgroup RCC_LL_EF_Get_Freq Get system and peripherals clocks frequency functions
  * @{
  */
void        LL_RCC_GetSystemClocksFreq(LL_RCC_ClocksTypeDef *RCC_Clocks);
uint32_t    LL_RCC_GetMCOClockFreq(uint32_t MCOx);
uint32_t    LL_RCC_GetLSCClockFreq(void);
#if defined(COMP1)
uint32_t    LL_RCC_GetCOMPClockFreq(uint32_t COMPx);
#endif
#if defined(LPTIM1)
uint32_t    LL_RCC_GetLPTIMClockFreq(uint32_t LPTIMx);
#endif 
/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(RCC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __PY32F002B_LL_RCC_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
