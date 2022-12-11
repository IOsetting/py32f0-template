/**
  ******************************************************************************
  * @file    py32f0xx_ll_pwr.h
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
#ifndef PY32F0XX_LL_PWR_H
#define PY32F0XX_LL_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx.h"

/** @addtogroup PY32F0XX_LL_Driver
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

/** @defgroup PWR_LL_EC_REGU_VOLTAGE REGU VOLTAGE
  * @{
  */
#define LL_PWR_REGU_VOLTAGE_SCALE1         0x0                /* After entering stop mode, VDD=1.2V */
#define LL_PWR_REGU_VOLTAGE_SCALE2         PWR_CR1_VOS        /* After entering stop mode, VDD=1.0V */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_SRAM_RETENTION_VOLTAGE SRAM RETENTION VOLTAGE
  * @{
  */
#define LL_PWR_SRAM_RETENTION_VOLT_0p9     (                     PWR_CR1_SRAM_RETV_1 | PWR_CR1_SRAM_RETV_0)  /* Set SRAM to 0.9V voltage in stop mode */
#define LL_PWR_SRAM_RETENTION_VOLT_VOS     (PWR_CR1_SRAM_RETV_2                                           )  /* Set SRAM voltage as set by bit VOS in stop mode */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_HSION_MODE WAKEUP HSI ON MODE
  * @{
  */
#define LL_PWR_WAKEUP_HSION_AFTER_MR       0x00000000U         /* Wake up from the STOP mode, After the MR becomes stable, enable HSI */
#define LL_PWR_WAKEUP_HSION_IMMEDIATE      PWR_CR1_HSION_CTRL  /* Wake up from the STOP mode, Enable HSI immediately */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_FLASH_DELAY WAKEUP FLASH DELAY
  * @{
  */
#define LL_PWR_WAKEUP_FLASH_DELAY_0US      (PWR_CR1_FLS_SLPTIME_1 | PWR_CR1_FLS_SLPTIME_0) /* Wake up from the STOP mode, Enable falsh immediately*/
#define LL_PWR_WAKEUP_FLASH_DELAY_2US      (                        PWR_CR1_FLS_SLPTIME_0) /* Wake up from the STOP mode, Delay 2us enable falsh*/
#define LL_PWR_WAKEUP_FLASH_DELAY_3US      (PWR_CR1_FLS_SLPTIME_1                        ) /* Wake up from the STOP mode, Delay 3us enable falsh*/
#define LL_PWR_WAKEUP_FLASH_DELAY_5US      0x00000000U                                     /* Wake up from the STOP mode, Delay 5us enable falsh*/
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_LP_TO_VR_READY_TIME WAKEUP LP TO VR READY TIME
  * @{
  */
#define LL_PWR_WAKEUP_LP_TO_VR_READY_2US      0x00000000U                                   /* Wake up from the STOP mode, LP to VR ready time 2us */
#define LL_PWR_WAKEUP_LP_TO_VR_READY_3US      (                       PWR_CR1_MRRDY_TIME_0) /* Wake up from the STOP mode, LP to VR ready time 3us */
#define LL_PWR_WAKEUP_LP_TO_VR_READY_4US      (PWR_CR1_MRRDY_TIME_1                       ) /* Wake up from the STOP mode, LP to VR ready time 4us */
#define LL_PWR_WAKEUP_LP_TO_VR_READY_5US      (PWR_CR1_MRRDY_TIME_1 | PWR_CR1_MRRDY_TIME_0) /* Wake up from the STOP mode, LP to VR ready time 5us */
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

#if defined(PWR_CR2_PVDE)
/** @defgroup PWR_LL_EC_PVDLEVEL PVDLEVEL
  * @{
  */
#define LL_PWR_PVDLEVEL_0                  0x000000000u                                                 /* VPVD0 (around 1.8V) */
#define LL_PWR_PVDLEVEL_1                  (PWR_CR2_PVDT_0)                                             /* VPVD1 (around 2.0V) */
#define LL_PWR_PVDLEVEL_2                  (PWR_CR2_PVDT_1)                                             /* VPVD2 (around 2.2V) */
#define LL_PWR_PVDLEVEL_3                  (PWR_CR2_PVDT_1 | PWR_CR2_PVDT_0)                           /* VPVD3 (around 2.4V) */
#define LL_PWR_PVDLEVEL_4                  (PWR_CR2_PVDT_2)                                             /* VPVD4 (around 2.6V) */
#define LL_PWR_PVDLEVEL_5                  (PWR_CR2_PVDT_2 | PWR_CR2_PVDT_0)                           /* VPVD5 (around 2. 8V) */
#define LL_PWR_PVDLEVEL_6                  (PWR_CR2_PVDT_2 | PWR_CR2_PVDT_1)                           /* VPVD6 (around 3.0V) */
#define LL_PWR_PVDLEVEL_7                  (PWR_CR2_PVDT_2 | PWR_CR2_PVDT_1 | PWR_CR2_PVDT_0)         /* VPVD7 (around 3.2V) */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_PVDSOURCE PVDSOURCE
  * @{
  */
#define LL_PWR_PVD_SOURCE_VCC               0
#define LL_PWR_PVD_SOURCE_PB7               PWR_CR2_SRCSEL
/**
  * @}
  */
 
/** @defgroup PWR_LL_EC_PVDFILTER PVDFILTER
  * @{
  */
#define LL_PWR_PVD_FILTER_1CLOCK          (0x00000000u)                                                   /*!< PVD filter 1    clock */
#define LL_PWR_PVD_FILTER_2CLOCK          (                                          PWR_CR2_FLT_TIME_0)  /*!< PVD filter 2    clock */
#define LL_PWR_PVD_FILTER_4CLOCK          (                     PWR_CR2_FLT_TIME_1                     )  /*!< PVD filter 2    clock */
#define LL_PWR_PVD_FILTER_16CLOCK         (                     PWR_CR2_FLT_TIME_1 | PWR_CR2_FLT_TIME_0)  /*!< PVD filter 4    clock */
#define LL_PWR_PVD_FILTER_64CLOCK         (PWR_CR2_FLT_TIME_2                                          )  /*!< PVD filter 16   clock */
#define LL_PWR_PVD_FILTER_128CLOCK        (PWR_CR2_FLT_TIME_2                      | PWR_CR2_FLT_TIME_0)  /*!< PVD filter 128  clock */
#define LL_PWR_PVD_FILTER_1024CLOCK       (PWR_CR2_FLT_TIME_2 | PWR_CR2_FLT_TIME_1                     )  /*!< PVD filter 1024 clock */
/**
  * @}
  */

#endif /* PWR_CR2_PVDE */


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
  * @brief  Set the main internal regulator output voltage
  * @rmtoll CR1          VOS           LL_PWR_SetRegulVoltageScaling
  * @param  VoltageScaling This parameter can be one of the following values:
  *         @arg @ref LL_PWR_REGU_VOLTAGE_SCALE1
  *         @arg @ref LL_PWR_REGU_VOLTAGE_SCALE2
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetRegulVoltageScaling(uint32_t VoltageScaling)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_VOS, VoltageScaling);
}

/**
  * @brief  Get the main internal regulator output voltage
  * @rmtoll CR1          VOS           LL_PWR_GetRegulVoltageScaling
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_REGU_VOLTAGE_SCALE1
  *         @arg @ref LL_PWR_REGU_VOLTAGE_SCALE2
  */
__STATIC_INLINE uint32_t LL_PWR_GetRegulVoltageScaling(void)
{
  return (READ_BIT(PWR->CR1, PWR_CR1_VOS));
}

/**
  * @brief  Switch the regulator from main mode to low-power mode
  * @rmtoll CR1          LPR           LL_PWR_EnableLowPowerRunMode
  * @retval None
  */
__STATIC_INLINE void LL_PWR_EnableLowPowerRunMode(void)
{
  SET_BIT(PWR->CR1, PWR_CR1_LPR);
}

/**
  * @brief  Switch the regulator from low-power mode to main mode
  * @rmtoll CR1          LPR           LL_PWR_DisableLowPowerRunMode
  * @retval None
  */
__STATIC_INLINE void LL_PWR_DisableLowPowerRunMode(void)
{
  CLEAR_BIT(PWR->CR1, PWR_CR1_LPR);
}

/**
  * @brief  Check if the regulator is in low-power mode
  * @rmtoll CR1          LPR           LL_PWR_IsEnabledLowPowerRunMode
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledLowPowerRunMode(void)
{
  return (READ_BIT(PWR->CR1, PWR_CR1_LPR) == (PWR_CR1_LPR));
}

/**
  * @brief  Switch the regulator from main mode to low-power mode.
  * @rmtoll CR1          LPR           LL_PWR_EnterLowPowerRunMode
  * @retval None
  */
__STATIC_INLINE void LL_PWR_EnterLowPowerRunMode(void)
{
  LL_PWR_EnableLowPowerRunMode();
}

/**
  * @brief  Switch the regulator from low-power mode to main mode.
  * @rmtoll CR1          LPR           LL_PWR_ExitLowPowerRunMode
  * @retval None
  */
__STATIC_INLINE void LL_PWR_ExitLowPowerRunMode(void)
{
  LL_PWR_DisableLowPowerRunMode();
}

/**
  * @brief  Enable access to the backup domain
  * @rmtoll CR1          DBP           LL_PWR_EnableBkUpAccess
  * @retval None
  */
__STATIC_INLINE void LL_PWR_EnableBkUpAccess(void)
{
  SET_BIT(PWR->CR1, PWR_CR1_DBP);
}

/**
  * @brief  Disable access to the backup domain
  * @rmtoll CR1          DBP           LL_PWR_DisableBkUpAccess
  * @retval None
  */
__STATIC_INLINE void LL_PWR_DisableBkUpAccess(void)
{
  CLEAR_BIT(PWR->CR1, PWR_CR1_DBP);
}

/**
  * @brief  Check if the backup domain is enabled
  * @rmtoll CR1          DBP           LL_PWR_IsEnabledBkUpAccess
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledBkUpAccess(void)
{
  return (READ_BIT(PWR->CR1, PWR_CR1_DBP) == (PWR_CR1_DBP));
}

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

/**
  * @brief  Set the SRAM retention voltage in stop mode
  * @rmtoll CR1          SRAM_RETV          LL_PWR_SetSramRetentionVolt
  * @param  RetentionVolt This parameter can be one of the following values:
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_0p9
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_VOS
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetSramRetentionVolt(uint32_t RetentionVolt)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_SRAM_RETV, RetentionVolt);
}

/**
  * @brief  Get the SRAM retention voltage in stop mode
  * @rmtoll CR1          SRAM_RETV          LL_PWR_GetSramRetentionVolt
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_0p9
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_VOS
  */
__STATIC_INLINE uint32_t LL_PWR_GetSramRetentionVolt(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_SRAM_RETV));
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
  * @brief  Set VDD voltage from LP to VR ready time after wake up 
  * @rmtoll CR1          MRRDY_TIME          LL_PWR_SetWakeUpLPToVRReadyTime
  * @param  ReadyTime This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_LP_TO_VR_READY_2US
  *         @arg @ref LL_PWR_WAKEUP_LP_TO_VR_READY_3US
  *         @arg @ref LL_PWR_WAKEUP_LP_TO_VR_READY_4US
  *         @arg @ref LL_PWR_WAKEUP_LP_TO_VR_READY_5US
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetWakeUpLPToVRReadyTime(uint32_t ReadyTime)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_MRRDY_TIME, ReadyTime);
}

/**
  * @brief  Get the LP to VR ready time after wake up 
  * @rmtoll CR1          MRRDY_TIME          LL_PWR_GetWakeUpLPToVRReadyTime
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_LP_TO_VR_READY_2US
  *         @arg @ref LL_PWR_WAKEUP_LP_TO_VR_READY_3US
  *         @arg @ref LL_PWR_WAKEUP_LP_TO_VR_READY_4US
  *         @arg @ref LL_PWR_WAKEUP_LP_TO_VR_READY_5US
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpLPToVRReadyTime(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_MRRDY_TIME));
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

#if defined (PWR_CR2_PVDE)
/**
  * @brief  Configure the voltage threshold detected by the Power Voltage Detector
  * @param  PVDLevel This parameter can be one of the following values:
  *         @arg @ref LL_PWR_PVDLEVEL_0
  *         @arg @ref LL_PWR_PVDLEVEL_1
  *         @arg @ref LL_PWR_PVDLEVEL_2
  *         @arg @ref LL_PWR_PVDLEVEL_3
  *         @arg @ref LL_PWR_PVDLEVEL_4
  *         @arg @ref LL_PWR_PVDLEVEL_5
  *         @arg @ref LL_PWR_PVDLEVEL_6
  *         @arg @ref LL_PWR_PVDLEVEL_7
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetPVDLevel(uint32_t PVDLevel)
{
  MODIFY_REG(PWR->CR2, PWR_CR2_PVDT, PVDLevel);
}

/**
  * @brief  Get the voltage threshold detection
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_PVDLEVEL_0
  *         @arg @ref LL_PWR_PVDLEVEL_1
  *         @arg @ref LL_PWR_PVDLEVEL_2
  *         @arg @ref LL_PWR_PVDLEVEL_3
  *         @arg @ref LL_PWR_PVDLEVEL_4
  *         @arg @ref LL_PWR_PVDLEVEL_5
  *         @arg @ref LL_PWR_PVDLEVEL_6
  *         @arg @ref LL_PWR_PVDLEVEL_7
  */
__STATIC_INLINE uint32_t LL_PWR_GetPVDLevel(void)
{
  return (uint32_t)(READ_BIT(PWR->CR2, PWR_CR2_PVDT));
}

/**
  * @brief  Enable Power Voltage Detector
  * @rmtoll CR2          PVDE          LL_PWR_EnablePVD
  * @retval None
  */
__STATIC_INLINE void LL_PWR_EnablePVD(void)
{
  SET_BIT(PWR->CR2, PWR_CR2_PVDE);
}

/**
  * @brief  Disable Power Voltage Detector
  * @rmtoll CR2          PVDE          LL_PWR_DisablePVD
  * @retval None
  */
__STATIC_INLINE void LL_PWR_DisablePVD(void)
{
  CLEAR_BIT(PWR->CR2, PWR_CR2_PVDE);
}

/**
  * @brief  Check if Power Voltage Detector is enabled
  * @rmtoll CR2          PVDE          LL_PWR_IsEnabledPVD
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledPVD(void)
{
  return (READ_BIT(PWR->CR2, PWR_CR2_PVDE) == (PWR_CR2_PVDE));
}

/**
  * @brief  PVD detection power supply selection
  * @rmtoll CR2          PWR_CR2_SRCSEL          LL_PWR_SetPVDSource
  * @param  PVDSrcSel This parameter can be one of the following values:
  *         @arg @ref LL_PWR_PVD_SOURCE_VCC
  *         @arg @ref LL_PWR_PVD_SOURCE_PB7
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetPVDSource(uint32_t PVDSrc)
{
  MODIFY_REG(PWR->CR2, PWR_CR2_SRCSEL, PVDSrc);
}

/**
  * @brief  Get PVD detection power supply 
  * @rmtoll CR2          PWR_CR2_SRCSEL          LL_PWR_GetPVDSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_PVD_SOURCE_VCC
  *         @arg @ref LL_PWR_PVD_SOURCE_PB7
  */
__STATIC_INLINE uint32_t LL_PWR_GetPVDSource(void)
{
  return (uint32_t)(READ_BIT(PWR->CR2, PWR_CR2_SRCSEL));
}

/**
  * @brief  Enable PVD Filter
  * @rmtoll CR2          FLTEN          LL_PWR_EnablePVDFilter
  * @retval None
  */
__STATIC_INLINE void LL_PWR_EnablePVDFilter(void)
{
  SET_BIT(PWR->CR2, PWR_CR2_FLTEN);
}

/**
  * @brief  Disable PVD Filter
  * @rmtoll CR2          FLTEN          LL_PWR_DisablePVDFilter
  * @retval None
  */
__STATIC_INLINE void LL_PWR_DisablePVDFilter(void)
{
  CLEAR_BIT(PWR->CR2, PWR_CR2_FLTEN);
}

/**
  * @brief  Check if PVD Filter is enabled
  * @rmtoll CR2          FLTEN          LL_PWR_IsEnabledPVDFilter
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledPVDFilter(void)
{
  return (READ_BIT(PWR->CR2, PWR_CR2_FLTEN) == (PWR_CR2_FLTEN));
}

/**
  * @brief  PVD detection power supply selection
  * @rmtoll CR2          PWR_CR2_FLT_TIME          LL_PWR_SetPVDFilter
  * @param  PVDSrcSel This parameter can be one of the following values:
  *         @arg @ref LL_PWR_PVD_FILTER_1CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_2CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_4CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_16CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_64CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_128CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_1024CLOCK
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetPVDFilter(uint32_t PVDFilter)
{
  MODIFY_REG(PWR->CR2, PWR_CR2_FLT_TIME, PVDFilter);
}

/**
  * @brief  Get PVD detection power supply 
  * @rmtoll CR2          PWR_CR2_FLT_TIME          LL_PWR_GetPVDFilter
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_PVD_FILTER_1CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_2CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_4CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_16CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_64CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_128CLOCK
  *         @arg @ref LL_PWR_PVD_FILTER_1024CLOCK
  */
__STATIC_INLINE uint32_t LL_PWR_GetPVDFilter(void)
{
  return (uint32_t)(READ_BIT(PWR->CR2, PWR_CR2_FLT_TIME));
}


/**
  * @}
  */

/** @defgroup PWR_LL_EF_FLAG_Management FLAG_Management
  * @{
  */
/**
  * @brief  Indicate whether Detected voltage is below or above the selected PVD
  *         threshold
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsActiveFlag_PVDO(void)
{
  return (READ_BIT(PWR->SR, PWR_SR_PVDO) == (PWR_SR_PVDO));
}
#endif /* PWR_CR2_PVDE */

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

#endif /* PY32F0XX_LL_PWR_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
