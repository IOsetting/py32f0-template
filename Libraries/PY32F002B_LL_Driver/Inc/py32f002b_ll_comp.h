/**
  ******************************************************************************
  * @file    py32f002b_ll_comp.h
  * @author  MCU Application Team
  * @brief   Header file of COMP LL module.
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
#ifndef __PY32F002B_LL_COMP_H
#define __PY32F002B_LL_COMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx.h"

/** @addtogroup PY32F002B_LL_Driver
  * @{
  */

#if defined (COMP1) || defined (COMP2)

/** @defgroup COMP_LL COMP
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup COMP_LL_Private_Constants COMP Private Constants
  * @{
  */

/* Internal mask for pair of comparators instances window mode:               */
/* To select into literals LL_COMP_WINDOWMODE_COMPx_INPUT_PLUS_COMMON         */
/* the relevant bits for:                                                     */
/* (concatenation of multiple bits used in different registers)               */
/* - Comparator instance selected as master for window mode : register offset */
/* - Window mode enable or disable: bit value */
#define LL_COMP_WINDOWMODE_COMP_ODD_REGOFFSET_MASK  (0x00000000U) /* Register of COMP instance odd (COMP1_CSR, ...) defined as reference register */
#define LL_COMP_WINDOWMODE_COMP_EVEN_REGOFFSET_MASK (0x00000004U) /* Register of COMP instance even (COMP2_CSR, ...) offset vs register of COMP instance odd */
#define LL_COMP_WINDOWMODE_COMPX_REGOFFSET_MASK     (LL_COMP_WINDOWMODE_COMP_ODD_REGOFFSET_MASK | LL_COMP_WINDOWMODE_COMP_EVEN_REGOFFSET_MASK)
#define LL_COMP_WINDOWMODE_COMPX_SETTING_MASK       (COMP_CSR_WINMODE)
#define LL_COMP_WINDOWOUTPUT_BOTH_POS_VS_WINDOW     (1U)

/* COMP registers bits positions */
#define LL_COMP_WINDOWMODE_BITOFFSET_POS   (11U) /* Value equivalent to POSITION_VAL(COMP_CSR_WINMODE) */
#define LL_COMP_OUTPUT_LEVEL_BITOFFSET_POS (30U) /* Value equivalent to POSITION_VAL(COMP_CSR_COMP_OUT) */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup COMP_LL_Private_Macros COMP Private Macros
  * @{
  */

/**
  * @brief  Driver macro reserved for internal use: set a pointer to
  *         a register from a register basis from which an offset
  *         is applied.
  * @param  __REG__ Register basis from which the offset is applied.
  * @param  __REG_OFFFSET__ Offset to be applied (unit: number of registers).
  * @retval Pointer to register address
  */
#define __COMP_PTR_REG_OFFSET(__REG__, __REG_OFFFSET__)                        \
  ((__IO uint32_t *)((uint32_t) ((uint32_t)(&(__REG__)) + ((__REG_OFFFSET__) << 2U))))

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup COMP_LL_ES_INIT COMP Exported Init structure
  * @{
  */

/**
  * @brief  Structure definition of some features of COMP instance.
  */
typedef struct
{
  uint32_t InputPlus;                   /*!< Set comparator input plus (non-inverting input).
                                             This parameter can be a value of @ref COMP_LL_EC_INPUT_PLUS

                                             This feature can be modified afterwards using unitary function @ref LL_COMP_SetInputPlus(). */

  uint32_t InputMinus;                  /*!< Set comparator input minus (inverting input).
                                             This parameter can be a value of @ref COMP_LL_EC_INPUT_MINUS

                                             This feature can be modified afterwards using unitary function @ref LL_COMP_SetInputMinus(). */
  
  uint32_t OutputPolarity;              /*!< Set comparator output polarity.
                                             This parameter can be a value of @ref COMP_LL_EC_OUTPUT_POLARITY

                                             This feature can be modified afterwards using unitary function @ref LL_COMP_SetOutputPolarity(). */

  uint32_t DigitalFilter;               /*!< Specifies the digital filter. 
                                             This parameter must be a number between 0 and 0xFFFF 

                                             The filter is prohibited,when the value is zero. 
                                             This feature can be modified afterwards using unitary function @ref LL_COMP_SetDigitalFilter(). */

} LL_COMP_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Constants COMP Exported Constants
  * @{
  */

/** @defgroup COMP_LL_EC_COMMON_WINDOWMODE Comparator common modes - Window mode
  * @{
  */
#define LL_COMP_WINDOWMODE_DISABLE                 (0x00000000U)                                                    /*!< Window mode disable: Comparators 1 and 2 are independent */
#define LL_COMP_WINDOWMODE_COMP2_INPUT_PLUS_COMMON (COMP_CSR_WINMODE | LL_COMP_WINDOWMODE_COMP_ODD_REGOFFSET_MASK)  /*!< Window mode enable: if used from COMP1 or COMP2 instance, comparators instances pair COMP1 and COMP2 have their input plus connected together, the common input is COMP2 input plus (COMP1 input plus is no more accessible). */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_INPUT_PLUS Comparator inputs - Input plus (input non-inverting) selection
  * @{
  */
#define LL_COMP_INPUT_PLUS_IO1          (0x00000000U)     /*!< Comparator input plus connected to IO1 (Reserved for COMP1, pin PA3 for COMP2) */
#define LL_COMP_INPUT_PLUS_IO2          (COMP_CSR_INPSEL) /*!< Comparator input plus connected to IO2 (Reserved for COMP1, VREFCMP for COMP2) */

/**
  * @}
  */

/** @defgroup COMP_LL_EC_INPUT_MINUS Comparator inputs - Input minus (input inverting) selection
  * @{
  */
#define LL_COMP_INPUT_MINUS_IO1         (0x00000000U)     /*!< Comparator input minus connected to IO1 (pin PB0 for COMP1, pin PA4 for COMP2) */
#define LL_COMP_INPUT_MINUS_IO2         (COMP_CSR_INNSEL) /*!< Comparator input minus connected to IO2 (pin PB1 for COMP1, pin PA3 for COMP2) */
/**
  * @}
  */
  
/** @defgroup COMP_LL_EC_INPUT_MINUS Comparator inputs - Input minus (input inverting) selection
  * @{
  */
#define LL_COMP_VREFCMP_DIV_1_16VREFCMP     (0x00000000U)     
#define LL_COMP_VREFCMP_DIV_2_16VREFCMP     (                                                                        COMP_CSR_COMP_VCDIV_0) 
#define LL_COMP_VREFCMP_DIV_3_16VREFCMP     (                                                COMP_CSR_COMP_VCDIV_1                        ) 
#define LL_COMP_VREFCMP_DIV_4_16VREFCMP     (                                                COMP_CSR_COMP_VCDIV_1 | COMP_CSR_COMP_VCDIV_0) 
#define LL_COMP_VREFCMP_DIV_5_16VREFCMP     (                        COMP_CSR_COMP_VCDIV_2                                                ) 
#define LL_COMP_VREFCMP_DIV_6_16VREFCMP     (                        COMP_CSR_COMP_VCDIV_2                         | COMP_CSR_COMP_VCDIV_0)  
#define LL_COMP_VREFCMP_DIV_7_16VREFCMP     (                        COMP_CSR_COMP_VCDIV_2 | COMP_CSR_COMP_VCDIV_1                        ) 
#define LL_COMP_VREFCMP_DIV_8_16VREFCMP     (                        COMP_CSR_COMP_VCDIV_2 | COMP_CSR_COMP_VCDIV_1 | COMP_CSR_COMP_VCDIV_0)  
#define LL_COMP_VREFCMP_DIV_9_16VREFCMP     (COMP_CSR_COMP_VCDIV_3                                                                        )  
#define LL_COMP_VREFCMP_DIV_10_16VREFCMP    (COMP_CSR_COMP_VCDIV_3                                                 | COMP_CSR_COMP_VCDIV_0) 
#define LL_COMP_VREFCMP_DIV_11_16VREFCMP    (COMP_CSR_COMP_VCDIV_3                         | COMP_CSR_COMP_VCDIV_1                        )  
#define LL_COMP_VREFCMP_DIV_12_16VREFCMP    (COMP_CSR_COMP_VCDIV_3                         | COMP_CSR_COMP_VCDIV_1 | COMP_CSR_COMP_VCDIV_0) 
#define LL_COMP_VREFCMP_DIV_13_16VREFCMP    (COMP_CSR_COMP_VCDIV_3 | COMP_CSR_COMP_VCDIV_2                                                ) 
#define LL_COMP_VREFCMP_DIV_14_16VREFCMP    (COMP_CSR_COMP_VCDIV_3 | COMP_CSR_COMP_VCDIV_2                         | COMP_CSR_COMP_VCDIV_0) 
#define LL_COMP_VREFCMP_DIV_15_16VREFCMP    (COMP_CSR_COMP_VCDIV_3 | COMP_CSR_COMP_VCDIV_2 | COMP_CSR_COMP_VCDIV_1                        ) 
#define LL_COMP_VREFCMP_DIV_VREFCMP         (COMP_CSR_COMP_VCDIV_3 | COMP_CSR_COMP_VCDIV_2 | COMP_CSR_COMP_VCDIV_1 | COMP_CSR_COMP_VCDIV_0) 
/**
  * @}
  */  
 
/** @defgroup COMP_LL_EC_INPUT_MINUS Comparator inputs - Input minus (input inverting) selection
  * @{
  */
#define LL_COMP_VREFCMP_SOURCE_VREFBUF       (0x00000000U)     
#define LL_COMP_VREFCMP_SOURCE_VCC           (COMP_CSR_COMP_VCSEL) 

/**
  * @}
  */   

/** @defgroup COMP_LL_EC_OUTPUT_POLARITY Comparator output - Output polarity
  * @{
  */
#define LL_COMP_OUTPUTPOL_NONINVERTED   (0x00000000U)           /*!< COMP output polarity is not inverted: comparator output is high when the plus (non-inverting) input is at a higher voltage than the minus (inverting) input */
#define LL_COMP_OUTPUTPOL_INVERTED      (COMP_CSR_POLARITY)     /*!< COMP output polarity is inverted: comparator output is low when the plus (non-inverting) input is at a lower voltage than the minus (inverting) input */
/**
  * @}
  */

/** @defgroup COMP_LL_EC_OUTPUT_LEVEL Comparator output - Output level
  * @{
  */
#define LL_COMP_OUTPUT_LEVEL_LOW        (0x00000000U)          /*!< Comparator output level low (if the polarity is not inverted, otherwise to be complemented) */
#define LL_COMP_OUTPUT_LEVEL_HIGH       (0x00000001U)          /*!< Comparator output level high (if the polarity is not inverted, otherwise to be complemented) */

/**
  * @}
  */

/** @defgroup COMP_LL_EC_HW_DELAYS  Definitions of COMP hardware constraints delays
  * @note   Only COMP peripheral HW delays are defined in COMP LL driver driver,
  *         not timeout values.
  *         For details on delays values, refer to descriptions in source code
  *         above each literal definition.
  * @{
  */

/* Delay for comparator startup time.                                         */
/* Note: Delay required to reach propagation delay specification.             */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSTART").                                                       */
/* Unit: us                                                                   */
#define LL_COMP_DELAY_STARTUP_US          ( 80U) /*!< Delay for COMP startup time */

/* Delay for comparator voltage scaler stabilization time.                    */
/* Note: Voltage scaler is used when selecting comparator input               */
/*       based on VrefInt: VrefInt or subdivision of VrefInt.                 */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSTART_SCALER").                                                */
/* Unit: us                                                                   */
#define LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US ( 200U) /*!< Delay for COMP voltage scaler stabilization time */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Macros COMP Exported Macros
  * @{
  */
/** @defgroup COMP_LL_EM_WRITE_READ Common write and read registers macro
  * @{
  */

/**
  * @brief  Write a value in COMP register
  * @param  __INSTANCE__ comparator instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_COMP_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG((__INSTANCE__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in COMP register
  * @param  __INSTANCE__ comparator instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_COMP_ReadReg(__INSTANCE__, __REG__) READ_REG((__INSTANCE__)->__REG__)
/**
  * @}
  */

/** @defgroup COMP_LL_EM_HELPER_MACRO COMP helper macro
  * @{
  */

/**
  * @brief  Helper macro to select the COMP common instance
  *         to which is belonging the selected COMP instance.
  * @note   COMP common register instance can be used to
  *         set parameters common to several COMP instances.
  *         Refer to functions having argument "COMPxy_COMMON" as parameter.
  * @param  __COMPx__ COMP instance
  * @retval COMP common instance or value "0" if there is no COMP common instance.
  */
#define __LL_COMP_COMMON_INSTANCE(__COMPx__)     (COMP12_COMMON)

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Functions COMP Exported Functions
  * @{
  */

/** @defgroup COMP_LL_EF_Configuration_comparator_common Configuration of COMP hierarchical scope: common to several COMP instances
  * @{
  */

/**
  * @brief  Set window mode of a pair of comparators instances
  * @rmtoll CSR      WINMODE        LL_COMP_SetCommonWindowMode
  * @param  COMPxy_COMMON Comparator common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __LL_COMP_COMMON_INSTANCE() )
  * @param  WindowMode This parameter can be one of the following values:
  *         @arg @ref LL_COMP_WINDOWMODE_DISABLE
  *         @arg @ref LL_COMP_WINDOWMODE_COMP2_INPUT_PLUS_COMMON 
  * @retval None  
  */
__STATIC_INLINE void LL_COMP_SetCommonWindowMode(COMP_Common_TypeDef *COMPxy_COMMON, uint32_t WindowMode)
{
  uint32_t window_mode_tmp = WindowMode;

  __IO uint32_t *preg = __COMP_PTR_REG_OFFSET(COMPxy_COMMON->CSR_ODD, (window_mode_tmp & LL_COMP_WINDOWMODE_COMPX_REGOFFSET_MASK));

  /* Clear the potential previous setting of window mode */
  __IO uint32_t *preg_clear = __COMP_PTR_REG_OFFSET(COMPxy_COMMON->CSR_ODD, (~(window_mode_tmp & LL_COMP_WINDOWMODE_COMPX_REGOFFSET_MASK) & LL_COMP_WINDOWMODE_COMP_EVEN_REGOFFSET_MASK));
  CLEAR_BIT(*preg_clear,COMP_CSR_WINMODE);

  /* Set window mode */
  MODIFY_REG(*preg, COMP_CSR_WINMODE,(window_mode_tmp & LL_COMP_WINDOWMODE_COMPX_SETTING_MASK));
}

/**
  * @brief  Get window mode of a pair of comparators instances
  * @rmtoll CSR      WINMODE        LL_COMP_GetCommonWindowMode
  * @param  COMPxy_COMMON Comparator common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __LL_COMP_COMMON_INSTANCE() )
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_WINDOWMODE_DISABLE 
  *         @arg @ref LL_COMP_WINDOWMODE_COMP2_INPUT_PLUS_COMMON 
  *
  */
__STATIC_INLINE uint32_t LL_COMP_GetCommonWindowMode(COMP_Common_TypeDef *COMPxy_COMMON)
{
   const uint32_t window_mode_comp_odd = (uint32_t)READ_BIT(COMPxy_COMMON->CSR_ODD, COMP_CSR_WINMODE);
   const uint32_t window_mode_comp_even = (uint32_t)READ_BIT(COMPxy_COMMON->CSR_EVEN, COMP_CSR_WINMODE);

   return (uint32_t)(window_mode_comp_odd | window_mode_comp_even
                    | ((window_mode_comp_even >> LL_COMP_WINDOWMODE_BITOFFSET_POS) * LL_COMP_WINDOWMODE_COMP_EVEN_REGOFFSET_MASK));
}

/**
  * @}
  */


/**
  * @}
  */

/** @defgroup COMP_LL_EF_Configuration_comparator_inputs Configuration of comparator inputs
  * @{
  */

/**
  * @brief  Set comparator inputs minus (inverting) and plus (non-inverting).
  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @rmtoll CSR      INMSEL         LL_COMP_ConfigInputs\n
  *         CSR      INPSEL         LL_COMP_ConfigInputs
  * @param  COMPx Comparator instance
  * @param  InputMinus This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_MINUS_IO1
  *         @arg @ref LL_COMP_INPUT_MINUS_IO2
  * @param  InputPlus This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_PLUS_IO1
  *         @arg @ref LL_COMP_INPUT_PLUS_IO2
  * @retval None
  */
__STATIC_INLINE void LL_COMP_ConfigInputs(COMP_TypeDef *COMPx, uint32_t InputMinus, uint32_t InputPlus)
{
  MODIFY_REG(COMPx->CSR,COMP_CSR_INNSEL | COMP_CSR_INPSEL,InputMinus | InputPlus);
}

/**
  * @brief  Set comparator input plus (non-inverting).
  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @rmtoll CSR      INPSEL         LL_COMP_SetInputPlus
  * @param  COMPx Comparator instance
  * @param  InputPlus This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_PLUS_IO1
  *         @arg @ref LL_COMP_INPUT_PLUS_IO2
  * @retval None
  */
__STATIC_INLINE void LL_COMP_SetInputPlus(COMP_TypeDef *COMPx, uint32_t InputPlus)
{
  MODIFY_REG(COMPx->CSR, COMP_CSR_INPSEL, InputPlus);
}

/**
  * @brief  Get comparator input plus (non-inverting).
  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @rmtoll CSR      INPSEL         LL_COMP_GetInputPlus
  * @param  COMPx Comparator instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_PLUS_IO1
  *         @arg @ref LL_COMP_INPUT_PLUS_IO2
  */
__STATIC_INLINE uint32_t LL_COMP_GetInputPlus(COMP_TypeDef *COMPx)
{
  return (uint32_t)(READ_BIT(COMPx->CSR, COMP_CSR_INPSEL));
}

/**
  * @brief  Set comparator input minus (inverting).
  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @rmtoll CSR      INMSEL         LL_COMP_SetInputMinus
  * @param  COMPx Comparator instance
  * @param  InputMinus This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_MINUS_IO1
  *         @arg @ref LL_COMP_INPUT_MINUS_IO2
  * @retval None
  */
__STATIC_INLINE void LL_COMP_SetInputMinus(COMP_TypeDef *COMPx, uint32_t InputMinus)
{
  MODIFY_REG(COMPx->CSR, COMP_CSR_INNSEL, InputMinus);
}

/**
  * @brief  Get comparator input minus (inverting).
  * @note   In case of comparator input selected to be connected to IO:
  *         GPIO pins are specific to each comparator instance.
  *         Refer to description of parameters or to reference manual.
  * @rmtoll CSR      INMSEL         LL_COMP_GetInputMinus
  * @param  COMPx Comparator instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_MINUS_IO1
  *         @arg @ref LL_COMP_INPUT_MINUS_IO2
  */
__STATIC_INLINE uint32_t LL_COMP_GetInputMinus(COMP_TypeDef *COMPx)
{
  return (uint32_t)(READ_BIT(COMPx->CSR, COMP_CSR_INNSEL));
}

/**
  * @brief  Enable comparator VrefCmp Divider
  * @rmtoll CSR      COMP_CSR_VCDIV_EN      LL_COMP_EnableVrefCmpDivider
  * @param  COMPx Comparator instance       
  * @retval None
  */
__STATIC_INLINE void LL_COMP_EnableVrefCmpDivider(COMP_TypeDef *COMPx)
{
  SET_BIT(COMP1->CSR, COMP_CSR_COMP_VCDIV_EN);
}

/**
  * @brief  Disable comparator VrefCmp Divider
  * @rmtoll CSR      COMP_CSR_COMP_VCDIV_EN      LL_COMP_DisableVrefCmpDivider
  * @param  COMPx Comparator instance       
  * @retval None
  */
__STATIC_INLINE void LL_COMP_DisableVrefCmpDivider(COMP_TypeDef *COMPx)
{
  CLEAR_BIT(COMP1->CSR, COMP_CSR_COMP_VCDIV_EN);
}

/**
  * @brief  Get comparator VrefCmp Divider enable state
  *         (0: VrefCmp Divider is disabled, 1: VrefCmp Divider is enabled)
  * @rmtoll CSR      COMP_CSR_COMP_VCDIV_EN      LL_COMP_IsEnabledVrefCmpDivider
  * @param  COMPx Comparator instance       
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_COMP_IsEnabledVrefCmpDivider(COMP_TypeDef *COMPx)
{
  return ((READ_BIT(COMP1->CSR, COMP_CSR_COMP_VCDIV_EN) == (COMP_CSR_COMP_VCDIV_EN)) ? 1U : 0U);
}

/**
  * @brief Set comparator VrefCmp voltage divider configuration.  
  * @rmtoll CSR      COMP_CSR_COMP_VCDIV      LL_COMP_SetVrefCmpDivider
  * @param  COMPx Comparator instance    
  * @param  VrefCmpDiv can be one of the following values:
  *         @arg @ref LL_COMP_VREF_DIV_1_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_2_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_3_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_4_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_5_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_6_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_7_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_8_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_9_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_10_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_11_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_12_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_13_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_14_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_15_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_VREF
  * @retval None
  */
__STATIC_INLINE void LL_COMP_SetVrefCmpDivider(COMP_TypeDef *COMPx, uint32_t VrefCmpDiv)
{
  MODIFY_REG(COMP1->CSR, COMP_CSR_COMP_VCDIV, VrefCmpDiv);
}

/**
  * @brief Get comparator VREFCMP voltage divider configuration.
  * @rmtoll CSR      COMP_CSR_COMP_VCDIV      LL_COMP_GetVrefCmpDivider
  * @param  COMPx Comparator instance       
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_VREF_DIV_1_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_2_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_3_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_4_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_5_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_6_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_7_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_8_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_9_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_10_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_11_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_12_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_13_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_14_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_15_16VREF
  *         @arg @ref LL_COMP_VREF_DIV_VREF
  */
__STATIC_INLINE uint32_t LL_COMP_GetVrefCmpDivider(COMP_TypeDef *COMPx)
{
   return (uint32_t)(READ_BIT(COMP1->CSR, COMP_CSR_COMP_VCDIV));
}


/**
  * @brief  Set comparator VrefCmp reference source selection. 
  * @rmtoll CSR      COMP_CSR_COMP_VCSEL      LL_COMP_SetVrefCmpSource
  * @param  COMPx Comparator instance       
  * @param  VrefCmpDiv can be one of the following values:
  *         @arg @ref LL_COMP_VREF_SOURCE_VREFBUF
  *         @arg @ref LL_COMP_VREF_SOURCE_VCC
  * @retval None
  */
__STATIC_INLINE void LL_COMP_SetVrefCmpSource(COMP_TypeDef *COMPx, uint32_t VrefCmpSource)
{
  MODIFY_REG(COMP1->CSR, COMP_CSR_COMP_VCSEL, VrefCmpSource);
}

/**
  * @brief  Get comparator VrefCmp reference source selection.
  * @rmtoll CSR      COMP_CSR_COMP_VCSEL      LL_COMP_GetVrefCmpSource
  * @param  COMPx Comparator instance       
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_VREF_SOURCE_VREFBUF
  *         @arg @ref LL_COMP_VREF_SOURCE_VCC
  */
__STATIC_INLINE uint32_t LL_COMP_GetVrefCmpSource(COMP_TypeDef *COMPx)
{
  return (uint32_t)(READ_BIT(COMP1->CSR, COMP_CSR_COMP_VCSEL));
}

/**
  * @}
  */

/** @defgroup COMP_LL_EF_Configuration_comparator_output Configuration of comparator output
  * @{
  */

/**
  * @brief  Set comparator instance output polarity.
  * @rmtoll CSR      POLARITY       LL_COMP_SetOutputPolarity
  * @param  COMPx Comparator instance
  * @param  OutputPolarity This parameter can be one of the following values:
  *         @arg @ref LL_COMP_OUTPUTPOL_NONINVERTED
  *         @arg @ref LL_COMP_OUTPUTPOL_INVERTED
  * @retval None
  */
__STATIC_INLINE void LL_COMP_SetOutputPolarity(COMP_TypeDef *COMPx, uint32_t OutputPolarity)
{
  MODIFY_REG(COMPx->CSR, COMP_CSR_POLARITY, OutputPolarity);
}

/**
  * @brief  Get comparator instance output polarity.
  * @rmtoll CSR      POLARITY       LL_COMP_GetOutputPolarity
  * @param  COMPx Comparator instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_OUTPUTPOL_NONINVERTED
  *         @arg @ref LL_COMP_OUTPUTPOL_INVERTED
  */
__STATIC_INLINE uint32_t LL_COMP_GetOutputPolarity(COMP_TypeDef *COMPx)
{
  return (uint32_t)(READ_BIT(COMPx->CSR, COMP_CSR_POLARITY));
}

/**
  * @}
  */

/** @defgroup COMP_LL_EF_Operation Operation on comparator instance
  * @{
  */

/**
  * @brief  Enable comparator instance.
  * @note   After enable from off state, comparator requires a delay
  *         to reach reach propagation delay specification.
  *         Refer to device datasheet, parameter "tSTART".
  * @rmtoll CSR      EN             LL_COMP_Enable
  * @param  COMPx Comparator instance
  * @retval None
  */
__STATIC_INLINE void LL_COMP_Enable(COMP_TypeDef *COMPx)
{
  SET_BIT(COMPx->CSR, COMP_CSR_EN);
}

/**
  * @brief  Disable comparator instance.
  * @rmtoll CSR      EN             LL_COMP_Disable
  * @param  COMPx Comparator instance
  * @retval None
  */
__STATIC_INLINE void LL_COMP_Disable(COMP_TypeDef *COMPx)
{
  CLEAR_BIT(COMPx->CSR, COMP_CSR_EN);
}

/**
  * @brief  Get comparator enable state
  *         (0: COMP is disabled, 1: COMP is enabled)
  * @rmtoll CSR      EN             LL_COMP_IsEnabled
  * @param  COMPx Comparator instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_COMP_IsEnabled(COMP_TypeDef *COMPx)
{
  return ((READ_BIT(COMPx->CSR, COMP_CSR_EN) == (COMP_CSR_EN)) ? 1U : 0U);
}

/**
  * @brief  Read comparator instance output level.
  * @note   The comparator output level depends on the selected polarity
  *         (Refer to function @ref LL_COMP_SetOutputPolarity()).
  *         If the comparator polarity is not inverted:
  *          - Comparator output is low when the input plus
  *            is at a lower voltage than the input minus
  *          - Comparator output is high when the input plus
  *            is at a higher voltage than the input minus
  *         If the comparator polarity is inverted:
  *          - Comparator output is high when the input plus
  *            is at a lower voltage than the input minus
  *          - Comparator output is low when the input plus
  *            is at a higher voltage than the input minus
  * @rmtoll CSR      COMP_CSR_COMP_OUT       LL_COMP_ReadOutputLevel
  * @param  COMPx Comparator instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_COMP_OUTPUT_LEVEL_LOW
  *         @arg @ref LL_COMP_OUTPUT_LEVEL_HIGH   
  */
__STATIC_INLINE uint32_t LL_COMP_ReadOutputLevel(COMP_TypeDef *COMPx)
{
  return (uint32_t)(READ_BIT(COMPx->CSR, COMP_CSR_COMP_OUT)>> LL_COMP_OUTPUT_LEVEL_BITOFFSET_POS);
}

/**
  * @}
  */

/** @defgroup COMP_LL_EF_DigitalFilter DigitalFilter on comparator instance
  * @{
  */
/**
  * @brief  Enable comparator DigitalFilter.
  * @rmtoll FR       FLTEN          LL_COMP_EnableDigitalFilter
  * @param  COMPx Comparator instance   
  * @retval None
  */
__STATIC_INLINE void LL_COMP_EnableDigitalFilter(COMP_TypeDef *COMPx)
{
  SET_BIT(COMPx->FR, COMP_FR_FLTEN);
}

/**
  * @brief  Disable comparator DigitalFilter.
  * @rmtoll FR       FLTEN          LL_COMP_DisableDigitalFilter
  * @param  COMPx Comparator instance   
  * @retval None
  */
__STATIC_INLINE void LL_COMP_DisableDigitalFilter(COMP_TypeDef *COMPx)
{
  CLEAR_BIT(COMPx->FR, COMP_FR_FLTEN);
}

/**
  * @brief  Get comparator DigitalFilter state
  *         (0: Filter is Disabled, 1: Filter is Enabled).
  * @rmtoll FR       FLTEN          LL_COMP_IsEnabledDigitalFilter
  * @param  COMPx Comparator instance    
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_COMP_IsEnabledDigitalFilter(COMP_TypeDef *COMPx)
{
  return ((READ_BIT(COMPx->FR, COMP_FR_FLTEN) == (COMP_FR_FLTEN)) ? 1U : 0U);
}

/**
  * @brief  Set comparator DigitalFilter Value.
  * @rmtoll FR       FLTCNT         LL_COMP_SetDigitalFilter
  * @param  COMPx Comparator instance      
  * @param  DigitalFilter Value between Min_Data=0x0000 and Max_Data=0xFFFF
  * @retval None
  */
__STATIC_INLINE void LL_COMP_SetDigitalFilter(COMP_TypeDef *COMPx,uint32_t FLTCNTValue)
{
  MODIFY_REG(COMPx->FR,COMP_FR_FLTCNT,FLTCNTValue << COMP_FR_FLTCNT_Pos);
}

/**
  * @brief  Get comparator DigitalFilter Value
  * @rmtoll FR       FLTCNT         LL_COMP_GetDigitalFilter
  * @param  COMPx Comparator instance          
  * @retval DigitalFilter Value between Min_Data=0x0000 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t LL_COMP_GetDigitalFilter(COMP_TypeDef *COMPx)
{
    return (uint32_t)(READ_REG(COMPx->FR)>>COMP_FR_FLTCNT_Pos);
}


/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup COMP_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus LL_COMP_DeInit(COMP_TypeDef *COMPx);
ErrorStatus LL_COMP_Init(COMP_TypeDef *COMPx, LL_COMP_InitTypeDef *COMP_InitStruct);
void        LL_COMP_StructInit(LL_COMP_InitTypeDef *COMP_InitStruct);

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

/**
  * @}
  */

#endif /* COMP1 || COMP2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* PY32F002B_LL_COMP_H */


/************************ (C) COPYRIGHT Puya *****END OF FILE****/
