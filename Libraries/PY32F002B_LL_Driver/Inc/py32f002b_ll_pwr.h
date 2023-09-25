/**
  ******************************************************************************
  * @file    py32f002b_ll_pwr.h
  * @author  MCU Application Team
  * @brief   Header file of PWR LL module.
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
#ifndef PY32F002B_LL_PWR_H
#define PY32F002B_LL_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx.h"

/** @addtogroup PY32F002B_LL_Driver
  * @{
  */

#if defined(PWR)

/** @defgroup PWR_LL PWR
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup PWR_LL_Exported_Constants PWR Exported Constants
  * @{
  */

/** @defgroup PWR_LL_EC_WAKEUP_HSION_MODE WAKEUP HSI ON MODE
  * @{
  */
#define LL_PWR_WAKEUP_HSION_AFTER_MR       0x00000000U         /* Wake up from the STOP mode, After the MR becomes stable, enable HSI */
#define LL_PWR_WAKEUP_HSION_IMMEDIATE      PWR_CR1_HSION_CTRL  /* Wake up from the STOP mode, Enable HSI immediately */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_SRAM_RETENTIONE_VOLTAGE_CONTROL SRAM RETENTIONE VOLTAGE CONTROL
  * @{
  */
#define LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO     0x00000001U  /* SRAM voltage is the same as LDO output */
#define LL_PWR_SRAM_RETENTION_VOLT_CTRL_LOW     0x00000000U  /* SRAM voltage is low */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_LOW_POWER_REGULATOR_MODE LOW POWER REGULATOR MODE
  * @{
  */
#define LL_PWR_LPR_MODE_MR         0x00000000U                 /* MR mode */
#define LL_PWR_LPR_MODE_LPR        PWR_CR1_LPR_0               /* Low Power Run mode */
#if defined(PWR_DEEPSTOP_SUPPORT)
#define LL_PWR_LPR_MODE_DLPR       PWR_CR1_LPR_1               /* Deep Low Power Run mode */
#endif /* PWR_DEEPSTOP_SUPPORT */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_FLASH_DELAY WAKEUP FLASH DELAY
  * @{
  */
#define LL_PWR_WAKEUP_FLASH_DELAY_0US      (PWR_CR1_FLS_SLPTIME_1 | PWR_CR1_FLS_SLPTIME_0) /* Wake up from the STOP mode, Enable flash immediately*/
#define LL_PWR_WAKEUP_FLASH_DELAY_2US      (                        PWR_CR1_FLS_SLPTIME_0) /* Wake up from the STOP mode, Delay 2us enable flash*/
#define LL_PWR_WAKEUP_FLASH_DELAY_3US      (PWR_CR1_FLS_SLPTIME_1                        ) /* Wake up from the STOP mode, Delay 3us enable flash*/
#define LL_PWR_WAKEUP_FLASH_DELAY_5US      0x00000000U                                     /* Wake up from the STOP mode, Delay 5us enable flash*/
/**
  * @}
  */

/** @defgroup PWR_LL_EC_BIAS_CURRENTS_SOURCE BIAS CURRENTS SOURCE
  * @{
  */
#define LL_PWR_BIAS_CURRENTS_FROM_FACTORY_BYTES  0x00000000U            /* MR bias currents source load from Factory config bytes */
#define LL_PWR_BIAS_CURRENTS_FROM_BIAS_CR        (PWR_CR1_BIAS_CR_SEL)  /* MR bias currents source load from BIAS_CR */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_GPIO_BIT GPIO BIT
  * @{
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PWR_LL_Exported_Macros PWR Exported Macros
  * @{
  */

/** @defgroup PWR_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in PWR register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_PWR_WriteReg(__REG__, __VALUE__) WRITE_REG(PWR->__REG__, (__VALUE__))

/**
  * @brief  Read a value in PWR register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_PWR_ReadReg(__REG__) READ_REG(PWR->__REG__)
/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup PWR_LL_Exported_Functions PWR Exported Functions
  * @{
  */

/** @defgroup PWR_LL_EF_Configuration Configuration
  * @{
  */
/**
  * @brief  Set the HSI turn on mode after wake up 
  * @rmtoll CR1          HSION_CTRL          LL_PWR_SetWakeUpHSIOnMode
  * @param  HsiOnMode This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_HSION_AFTER_MR
  *         @arg @ref LL_PWR_WAKEUP_HSION_IMMEDIATE
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetWakeUpHSIOnMode(uint32_t HsiOnMode)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_HSION_CTRL, HsiOnMode);
}

/**
  * @brief  Get the HSI turn on mode after wake up
  * @rmtoll CR1          HSION_CTRL          LL_PWR_GetWakeUpHSIOnMode
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_HSION_AFTER_MR
  *         @arg @ref LL_PWR_WAKEUP_HSION_IMMEDIATE
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpHSIOnMode(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_HSION_CTRL));
}

#if defined(PWR_DEEPSTOP_SUPPORT)
/**
  * @brief  Set SRAM retention voltage control in deep stop mode.
  * @note   Depending on devices and packages, Deep Low Power Run mode may not be available.
  *         Refer to device datasheet for Deep Low Power Run mode availability.
  * @param  VoltCtrl This parameter can be one of the following values:
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_CTRL_LOW
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetDeepStopModeSramVoltCtrl(uint32_t VoltCtrl)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_SRAM_RETV_DLP, (VoltCtrl << PWR_CR1_SRAM_RETV_DLP_Pos));
}

/**
  * @brief  Get SRAM retention voltage control in deep stop mode.
  * @note   Depending on devices and packages, Deep Low Power Run mode may not be available.
  *         Refer to device datasheet for Deep Low Power Run mode availability.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_CTRL_LOW
  */
__STATIC_INLINE uint32_t LL_PWR_GetDeepStopModeSramVoltCtrl(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_SRAM_RETV_DLP) >> PWR_CR1_SRAM_RETV_DLP_Pos);
}
#endif /* PWR_DEEPSTOP_SUPPORT */

/**
  * @brief  Set SRAM retention voltage control in stop mode.
  * @param  VoltCtrl This parameter can be one of the following values:
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_CTRL_LOW
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetStopModeSramVoltCtrl(uint32_t VoltCtrl)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_SRAM_RETV, (VoltCtrl << PWR_CR1_SRAM_RETV_Pos));
}

/**
  * @brief  Get SRAM retention voltage control in stop mode.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_CTRL_LOW
  */
__STATIC_INLINE uint32_t LL_PWR_GetStopModeSramVoltCtrl(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_SRAM_RETV) >> PWR_CR1_SRAM_RETV_Pos);
}

/**
  * @brief  Set the Low power regulator mode.
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_PWR_LPR_MODE_MR
  *         @arg @ref LL_PWR_LPR_MODE_LPR
  *         @arg @ref LL_PWR_LPR_MODE_DLPR
  * @note   Depending on devices and packages, Deep Low Power Run mode may not be available.
  *         Refer to device datasheet for Deep Low Power Run mode availability.
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetLprMode(uint32_t Mode)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_LPR, Mode);
}

/**
  * @brief  Get the Low power regulator mode.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_LPR_MODE_MR
  *         @arg @ref LL_PWR_LPR_MODE_LPR
  *         @arg @ref LL_PWR_LPR_MODE_DLPR
  */
__STATIC_INLINE uint32_t LL_PWR_GetLprMode(void)
{
  return (READ_BIT(PWR->CR1, PWR_CR1_LPR));
}

/**
  * @brief  Set the flash delay time after wake up 
  * @rmtoll CR1          FLS_SLPTIME          LL_PWR_SetWakeUpFlashDelay
  * @param  FlashDelay This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_0US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_2US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_3US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_5US
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetWakeUpFlashDelay(uint32_t FlashDelay)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_FLS_SLPTIME, FlashDelay);
}

/**
  * @brief  Get the flash delay time after wake up 
  * @rmtoll CR1          FLS_SLPTIME          LL_PWR_GetWakeUpFlashDelay
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_0US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_2US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_3US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_5US
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpFlashDelay(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_FLS_SLPTIME));
}

/**
  * @brief  Set the bias currents load source and bias currents config value.
  * @rmtoll CR1          BIAS_CR_SEL | BIAS_CR          LL_PWR_SetBiasCurrents
  * @param  BiasCurSel This parameter can be one of the following values:
  *         @arg @ref LL_PWR_BIAS_CURRENTS_FROM_FACTORY_BYTES
  *         @arg @ref LL_PWR_BIAS_CURRENTS_FROM_BIAS_CR
  * @param  BiasCur    This parameter must be a number between 0x0000 and 0xFFFF
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetBiasCurrents(uint32_t BiasCurSel, uint32_t BiasCur)
{
  MODIFY_REG(PWR->CR1, (PWR_CR1_BIAS_CR_SEL | PWR_CR1_BIAS_CR), (BiasCurSel | BiasCur));
}

/**
  * @brief  Get the bias currents load source
  * @rmtoll CR1          BIAS_CR_SEL          LL_PWR_GetBiasCurrentsLoadSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_BIAS_CURRENTS_FROM_FACTORY_BYTES
  *         @arg @ref LL_PWR_BIAS_CURRENTS_FROM_BIAS_CR
  */
__STATIC_INLINE uint32_t LL_PWR_GetBiasCurrentsLoadSource(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_BIAS_CR_SEL));
}

/**
  * @brief  Get the bias currents config value
  * @rmtoll CR1          BIAS_CR          LL_PWR_GetBiasCRValue
  * @retval Returned value can be number between 0x00 and 0x0F
  */
__STATIC_INLINE uint32_t LL_PWR_GetBiasCRValue(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_BIAS_CR));
}

/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup PWR_LL_EF_Init De-initialization function
  * @{
  */
ErrorStatus LL_PWR_DeInit(void);
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

#endif /* defined(PWR) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* PY32F002B_LL_PWR_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
