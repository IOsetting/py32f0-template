/**
  ******************************************************************************
  * @file    py32f0xx_ll_lptim.h
  * @author  MCU Application Team
  * @brief   Header file of LPTIM LL module.
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
#ifndef PY32F0XX_LL_LPTIM_H
#define PY32F0XX_LL_LPTIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx.h"

/** @addtogroup PY32F0XX_LL_Driver
  * @{
  */

#if defined (LPTIM)

/** @defgroup LPTIM_LL LPTIM
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup LPTIM_LL_Private_Macros LPTIM Private Macros
  * @{
  */
/**
  * @}
  */
#endif /*USE_FULL_LL_DRIVER*/

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup LPTIM_LL_ES_INIT LPTIM Exported Init structure
  * @{
  */

/**
  * @brief  LPTIM Init structure definition
  */
typedef struct
{

  uint32_t Prescaler;      /*!< Specifies the prescaler division ratio.
                                This parameter can be a value of @ref LPTIM_LL_EC_PRESCALER.

                                This feature can be modified afterwards using using unitary
                                function @ref LL_LPTIM_SetPrescaler().*/
  
  uint32_t UpdateMode;     /*!< Specifies whether to update immediately or after the end 
                                of current period.
                                This parameter can be a value of @ref LPTIM_LL_EC_UPDATE_MODE 
  
                                This feature can be modified afterwards using using unitary
                                function @ref LL_LPTIM_SetUpdateMode().*/
} LL_LPTIM_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup LPTIM_LL_Exported_Constants LPTIM Exported Constants
  * @{
  */

/** @defgroup LPTIM_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_LPTIM_ReadReg function
  * @{
  */
#define LL_LPTIM_ISR_ARRM                     LPTIM_ISR_ARRM     /*!< Autoreload match */
/**
  * @}
  */

/** @defgroup LPTIM_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_LPTIM_ReadReg and  LL_LPTIM_WriteReg functions
  * @{
  */
#define LL_LPTIM_IER_ARRMIE                   LPTIM_IER_ARRMIE     /*!< Autoreload match */
/**
  * @}
  */

/** @defgroup LPTIM_LL_EC_OPERATING_MODE Operating Mode
  * @{
  */
#define LL_LPTIM_OPERATING_MODE_ONESHOT       LPTIM_CR_SNGSTRT /*!<LP Tilmer starts in single mode*/
/**
  * @}
  */

/** @defgroup LPTIM_LL_EC_UPDATE_MODE Update Mode
  * @{
  */
#define LL_LPTIM_UPDATE_MODE_IMMEDIATE        0x00000000U        /*!<Preload is disabled: registers are updated after each APB bus write access*/
#define LL_LPTIM_UPDATE_MODE_ENDOFPERIOD      LPTIM_CFGR_PRELOAD /*!<preload is enabled: registers are updated at the end of the current LPTIM period*/
/**
  * @}
  */

/** @defgroup LPTIM_LL_EC_PRESCALER Prescaler Value
  * @{
  */
#define LL_LPTIM_PRESCALER_DIV1               0x00000000U                               /*!<Prescaler division factor is set to 1*/
#define LL_LPTIM_PRESCALER_DIV2               LPTIM_CFGR_PRESC_0                        /*!<Prescaler division factor is set to 2*/
#define LL_LPTIM_PRESCALER_DIV4               LPTIM_CFGR_PRESC_1                        /*!<Prescaler division factor is set to 4*/
#define LL_LPTIM_PRESCALER_DIV8               (LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_0) /*!<Prescaler division factor is set to 8*/
#define LL_LPTIM_PRESCALER_DIV16              LPTIM_CFGR_PRESC_2                        /*!<Prescaler division factor is set to 16*/
#define LL_LPTIM_PRESCALER_DIV32              (LPTIM_CFGR_PRESC_2 | LPTIM_CFGR_PRESC_0) /*!<Prescaler division factor is set to 32*/
#define LL_LPTIM_PRESCALER_DIV64              (LPTIM_CFGR_PRESC_2 | LPTIM_CFGR_PRESC_1) /*!<Prescaler division factor is set to 64*/
#define LL_LPTIM_PRESCALER_DIV128             LPTIM_CFGR_PRESC                          /*!<Prescaler division factor is set to 128*/
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LPTIM_LL_Exported_Macros LPTIM Exported Macros
  * @{
  */

/** @defgroup LPTIM_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in LPTIM register
  * @param  __INSTANCE__ LPTIM Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_LPTIM_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG((__INSTANCE__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in LPTIM register
  * @param  __INSTANCE__ LPTIM Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_LPTIM_ReadReg(__INSTANCE__, __REG__) READ_REG((__INSTANCE__)->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup LPTIM_LL_Exported_Functions LPTIM Exported Functions
  * @{
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup LPTIM_LL_EF_Init Initialisation and deinitialisation functions
  * @{
  */

ErrorStatus LL_LPTIM_DeInit(LPTIM_TypeDef *LPTIMx);
void LL_LPTIM_StructInit(LL_LPTIM_InitTypeDef *LPTIM_InitStruct);
ErrorStatus LL_LPTIM_Init(LPTIM_TypeDef *LPTIMx, LL_LPTIM_InitTypeDef *LPTIM_InitStruct);
/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/** @defgroup LPTIM_LL_EF_LPTIM_Configuration LPTIM Configuration
  * @{
  */

/**
  * @brief  Enable the LPTIM instance
  * @note After setting the ENABLE bit, a delay of two counter clock is needed
  *       before the LPTIM instance is actually enabled.
  * @rmtoll CR           ENABLE        LL_LPTIM_Enable
  * @param  LPTIMx Low-Power Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_Enable(LPTIM_TypeDef *LPTIMx)
{
  SET_BIT(LPTIMx->CR, LPTIM_CR_ENABLE);
}

/**
  * @brief  Disable the LPTIM instance
  * @rmtoll CR           ENABLE        LL_LPTIM_Disable
  * @param  LPTIMx Low-Power Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_Disable(LPTIM_TypeDef *LPTIMx)
{
  CLEAR_BIT(LPTIMx->CR, LPTIM_CR_ENABLE);
}

/**
  * @brief  Indicates whether the LPTIM instance is enabled.
  * @rmtoll CR           ENABLE        LL_LPTIM_IsEnabled
  * @param  LPTIMx Low-Power Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPTIM_IsEnabled(LPTIM_TypeDef *LPTIMx)
{
  return (((READ_BIT(LPTIMx->CR, LPTIM_CR_ENABLE) == LPTIM_CR_ENABLE) ? 1UL : 0UL));
}

/**
  * @brief  Starts the LPTIM counter in the desired mode.
  * @note LPTIM instance must be enabled before starting the counter.
  * @rmtoll CR           SNGSTRT       LL_LPTIM_StartCounter
  * @param  LPTIMx Low-Power Timer instance
  * @param  OperatingMode This parameter can be one of the following values:
  *         @arg @ref LL_LPTIM_OPERATING_MODE_ONESHOT
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_StartCounter(LPTIM_TypeDef *LPTIMx, uint32_t OperatingMode)
{
  MODIFY_REG(LPTIMx->CR, LPTIM_CR_SNGSTRT, OperatingMode);
}

/**
  * @brief  Enable reset after read.
  * @note After calling this function any read access to LPTIM_CNT
  *        register will asynchronously reset the LPTIM_CNT register content.
  * @rmtoll CR           RSTARE        LL_LPTIM_EnableResetAfterRead
  * @param  LPTIMx Low-Power Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_EnableResetAfterRead(LPTIM_TypeDef *LPTIMx)
{
  SET_BIT(LPTIMx->CR, LPTIM_CR_RSTARE);
}

/**
  * @brief  Disable reset after read.
  * @rmtoll CR           RSTARE        LL_LPTIM_DisableResetAfterRead
  * @param  LPTIMx Low-Power Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_DisableResetAfterRead(LPTIM_TypeDef *LPTIMx)
{
  CLEAR_BIT(LPTIMx->CR, LPTIM_CR_RSTARE);
}

/**
  * @brief  Indicate whether the reset after read feature is enabled.
  * @rmtoll CR           RSTARE        LL_LPTIM_IsEnabledResetAfterRead
  * @param  LPTIMx Low-Power Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPTIM_IsEnabledResetAfterRead(LPTIM_TypeDef *LPTIMx)
{
  return (((READ_BIT(LPTIMx->CR, LPTIM_CR_RSTARE) == LPTIM_CR_RSTARE) ? 1UL : 0UL));
}

/**
  * @brief  Set the LPTIM registers update mode (enable/disable register preload)
  * @note This function must be called when the LPTIM instance is disabled.
  * @rmtoll CFGR         PRELOAD       LL_LPTIM_SetUpdateMode
  * @param  LPTIMx Low-Power Timer instance
  * @param  UpdateMode This parameter can be one of the following values:
  *         @arg @ref LL_LPTIM_UPDATE_MODE_IMMEDIATE
  *         @arg @ref LL_LPTIM_UPDATE_MODE_ENDOFPERIOD
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_SetUpdateMode(LPTIM_TypeDef *LPTIMx, uint32_t UpdateMode)
{
  MODIFY_REG(LPTIMx->CFGR, LPTIM_CFGR_PRELOAD, UpdateMode);
}

/**
  * @brief  Get the LPTIM registers update mode
  * @rmtoll CFGR         PRELOAD       LL_LPTIM_GetUpdateMode
  * @param  LPTIMx Low-Power Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPTIM_UPDATE_MODE_IMMEDIATE
  *         @arg @ref LL_LPTIM_UPDATE_MODE_ENDOFPERIOD
  */
__STATIC_INLINE uint32_t LL_LPTIM_GetUpdateMode(LPTIM_TypeDef *LPTIMx)
{
  return (uint32_t)(READ_BIT(LPTIMx->CFGR, LPTIM_CFGR_PRELOAD));
}

/**
  * @brief  Set the auto reload value
  * @note The LPTIMx_ARR register content must only be modified when the LPTIM is enabled
  * @note After a write to the LPTIMx_ARR register a new write operation to the
  *       same register can only be performed when the previous write operation
  *       is completed. Any successive write before  the ARROK flag is set, will
  *       lead to unpredictable results.
  * @note autoreload value be strictly greater than the compare value.
  * @rmtoll ARR          ARR           LL_LPTIM_SetAutoReload
  * @param  LPTIMx Low-Power Timer instance
  * @param  AutoReload Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_SetAutoReload(LPTIM_TypeDef *LPTIMx, uint32_t AutoReload)
{
  MODIFY_REG(LPTIMx->ARR, LPTIM_ARR_ARR, AutoReload);
}

/**
  * @brief  Get actual auto reload value
  * @rmtoll ARR          ARR           LL_LPTIM_GetAutoReload
  * @param  LPTIMx Low-Power Timer instance
  * @retval AutoReload Value between Min_Data=0x00 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t LL_LPTIM_GetAutoReload(LPTIM_TypeDef *LPTIMx)
{
  return (uint32_t)(READ_BIT(LPTIMx->ARR, LPTIM_ARR_ARR));
}

/**
  * @brief  Get actual counter value
  * @note When the LPTIM instance is running with an asynchronous clock, reading
  *       the LPTIMx_CNT register may return unreliable values. So in this case
  *       it is necessary to perform two consecutive read accesses and verify
  *       that the two returned values are identical.
  * @rmtoll CNT          CNT           LL_LPTIM_GetCounter
  * @param  LPTIMx Low-Power Timer instance
  * @retval Counter value
  */
__STATIC_INLINE uint32_t LL_LPTIM_GetCounter(LPTIM_TypeDef *LPTIMx)
{
  return (uint32_t)(READ_BIT(LPTIMx->CNT, LPTIM_CNT_CNT));
}

/**
  * @brief  Set actual prescaler division ratio.
  * @note This function must be called when the LPTIM instance is disabled.
  * @note When the LPTIM is configured to be clocked by an internal clock source
  *       and the LPTIM counter is configured to be updated by active edges
  *       detected on the LPTIM external Input1, the internal clock provided to
  *       the LPTIM must be not be prescaled.
  * @rmtoll CFGR         PRESC         LL_LPTIM_SetPrescaler
  * @param  LPTIMx Low-Power Timer instance
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_LPTIM_PRESCALER_DIV1
  *         @arg @ref LL_LPTIM_PRESCALER_DIV2
  *         @arg @ref LL_LPTIM_PRESCALER_DIV4
  *         @arg @ref LL_LPTIM_PRESCALER_DIV8
  *         @arg @ref LL_LPTIM_PRESCALER_DIV16
  *         @arg @ref LL_LPTIM_PRESCALER_DIV32
  *         @arg @ref LL_LPTIM_PRESCALER_DIV64
  *         @arg @ref LL_LPTIM_PRESCALER_DIV128
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_SetPrescaler(LPTIM_TypeDef *LPTIMx, uint32_t Prescaler)
{
  MODIFY_REG(LPTIMx->CFGR, LPTIM_CFGR_PRESC, Prescaler);
}

/**
  * @brief  Get actual prescaler division ratio.
  * @rmtoll CFGR         PRESC         LL_LPTIM_GetPrescaler
  * @param  LPTIMx Low-Power Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_LPTIM_PRESCALER_DIV1
  *         @arg @ref LL_LPTIM_PRESCALER_DIV2
  *         @arg @ref LL_LPTIM_PRESCALER_DIV4
  *         @arg @ref LL_LPTIM_PRESCALER_DIV8
  *         @arg @ref LL_LPTIM_PRESCALER_DIV16
  *         @arg @ref LL_LPTIM_PRESCALER_DIV32
  *         @arg @ref LL_LPTIM_PRESCALER_DIV64
  *         @arg @ref LL_LPTIM_PRESCALER_DIV128
  */
__STATIC_INLINE uint32_t LL_LPTIM_GetPrescaler(LPTIM_TypeDef *LPTIMx)
{
  return (uint32_t)(READ_BIT(LPTIMx->CFGR, LPTIM_CFGR_PRESC));
}

/**
  * @}
  */

/** @defgroup LPTIM_LL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Clear the autoreload match flag (ARRMCF)
  * @rmtoll ICR          ARRMCF        LL_LPTIM_ClearFLAG_ARRM
  * @param  LPTIMx Low-Power Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_ClearFLAG_ARRM(LPTIM_TypeDef *LPTIMx)
{
  SET_BIT(LPTIMx->ICR, LPTIM_ICR_ARRMCF);
}

/**
  * @brief  Inform application whether a autoreload match interrupt has occurred.
  * @rmtoll ISR          ARRM          LL_LPTIM_IsActiveFlag_ARRM
  * @param  LPTIMx Low-Power Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPTIM_IsActiveFlag_ARRM(LPTIM_TypeDef *LPTIMx)
{
  return (((READ_BIT(LPTIMx->ISR, LPTIM_ISR_ARRM) == LPTIM_ISR_ARRM) ? 1UL : 0UL));
}

/**
  * @}
  */

/** @defgroup LPTIM_LL_EF_IT_Management Interrupt Management
  * @{
  */

/**
  * @brief  Enable autoreload match interrupt (ARRMIE).
  * @rmtoll IER          ARRMIE        LL_LPTIM_EnableIT_ARRM
  * @param  LPTIMx Low-Power Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_EnableIT_ARRM(LPTIM_TypeDef *LPTIMx)
{
  SET_BIT(LPTIMx->IER, LPTIM_IER_ARRMIE);
}

/**
  * @brief  Disable autoreload match interrupt (ARRMIE).
  * @rmtoll IER          ARRMIE        LL_LPTIM_DisableIT_ARRM
  * @param  LPTIMx Low-Power Timer instance
  * @retval None
  */
__STATIC_INLINE void LL_LPTIM_DisableIT_ARRM(LPTIM_TypeDef *LPTIMx)
{
  CLEAR_BIT(LPTIMx->IER, LPTIM_IER_ARRMIE);
}

/**
  * @brief  Indicates whether the autoreload match interrupt (ARRMIE) is enabled.
  * @rmtoll IER          ARRMIE        LL_LPTIM_IsEnabledIT_ARRM
  * @param  LPTIMx Low-Power Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_LPTIM_IsEnabledIT_ARRM(LPTIM_TypeDef *LPTIMx)
{
  return (((READ_BIT(LPTIMx->IER, LPTIM_IER_ARRMIE) == LPTIM_IER_ARRMIE) ? 1UL : 0UL));
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

#endif /* LPTIM */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* PY32F0XX_LL_LPTIM_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
