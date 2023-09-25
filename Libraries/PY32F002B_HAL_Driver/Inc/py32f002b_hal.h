/**
  ******************************************************************************
  * @file    py32f002b_hal.h
  * @author  MCU Application Team
  * @brief   This file contains all the functions prototypes for the HAL
  *          module driver.
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
#ifndef __PY32F002B_HAL_H
#define __PY32F002B_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f002b_hal_conf.h"

/** @addtogroup PY32F002B_HAL_Driver
  * @{
  */

/** @addtogroup HAL
  * @{
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup HAL_Exported_Constants HAL Exported Constants
  * @{
  */

/** @defgroup HAL_TICK_FREQ Tick Frequency
  * @{
  */
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;
/**
  * @}
  */
/* Exported types ------------------------------------------------------------*/
extern uint32_t uwTickPrio;
extern uint32_t uwTickFreq;
/** @defgroup SYSCFG_Exported_Constants SYSCFG Exported Constants
  * @{
  */

/** @defgroup SYSCFG_BootMode Boot Mode
  * @{
  */
#define SYSCFG_BOOT_MAINFLASH          0x00000000U                      /*!< Main Flash memory mapped at 0x0000 0000   */
#define SYSCFG_BOOT_SYSTEMFLASH        SYSCFG_CFGR1_MEM_MODE_0          /*!< System Flash memory mapped at 0x0000 0000 */
#define SYSCFG_BOOT_SRAM               (SYSCFG_CFGR1_MEM_MODE_1 | SYSCFG_CFGR1_MEM_MODE_0)  /*!< Embedded SRAM mapped at 0x0000 0000 */
/**
  * @}
  */

/** @defgroup SYSTEM_CH1_SRC TIM1 CH1 SOURCE
  */
#define SYSCFG_CH1_SRC_TIM1_GPIO          0x00000000U
#define SYSCFG_CH1_SRC_TIM1_COMP1         SYSCFG_CFGR1_TIM1_IC1_SRC_0
#define SYSCFG_CH1_SRC_TIM1_COMP2         SYSCFG_CFGR1_TIM1_IC1_SRC_1
/**
  * @}
  */

/** @defgroup SYSTEM_I2C_FMP I2C FAST MODE ENABLE CONTORL
  * @{
  */
#define SYSCFG_I2C_FMP_PA2                SYSCFG_CFGR1_I2C_PA2_FMP
#define SYSCFG_I2C_FMP_PB3                SYSCFG_CFGR1_I2C_PB3_FMP
#define SYSCFG_I2C_FMP_PB4                SYSCFG_CFGR1_I2C_PB4_FMP
#define SYSCFG_I2C_FMP_PB6                SYSCFG_CFGR1_I2C_PB6_FMP
/**
  * @}
  */

#if defined(SYSCFG_CFGR2_ETR_SRC_TIM1)
#define SYSCFG_ETR_SRC_TIM1_GPIO               0x00000000
#define SYSCFG_ETR_SRC_TIM1_COMP1              SYSCFG_CFGR2_ETR_SRC_TIM1_0
#define SYSCFG_ETR_SRC_TIM1_COMP2              SYSCFG_CFGR2_ETR_SRC_TIM1_1
#define SYSCFG_ETR_SRC_TIM1_ADC                (SYSCFG_CFGR2_ETR_SRC_TIM1_1 | SYSCFG_CFGR2_ETR_SRC_TIM1_0)
#endif /* SYSCFG_CFGR2_ETR_SRC_TIM1 */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup HAL_Exported_Macros HAL Exported Macros
  * @{
  */

/** @defgroup DBGMCU_Freeze_Unfreeze Freeze Unfreeze Peripherals in Debug mode
  * @brief   Freeze/Unfreeze Peripherals in Debug mode
  * Note:
  *       Debug registers DBGMCU_IDCODE and DBGMCU_CR are accessible only in
  *       debug mode (not accessible by the user software in normal mode).
  *       Refer to errata sheet of these devices for more details.
  * @{
  */

/* Peripherals on APB1 */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup HAL_Exported_Macros HAL Exported Macros
  * @{
  */

/** @defgroup HAL_Freeze_Unfreeze_Peripherals HAL Freeze Unfreeze Peripherals
  * @brief  Freeze/Unfreeze Peripherals in Debug mode
  * @{
  */
#if defined(DBGMCU_APB_FZ1_DBG_IWDG_STOP)
#define __HAL_DBGMCU_FREEZE_IWDG()           (DBGMCU->APBFZ1 |= (DBGMCU_APB_FZ1_DBG_IWDG_STOP))
#define __HAL_DBGMCU_UNFREEZE_IWDG()         (DBGMCU->APBFZ1 &= ~(DBGMCU_APB_FZ1_DBG_IWDG_STOP))
#endif /* DBGMCU_APB_FZ1_DBG_IWDG_STOP */

#if defined(DBGMCU_APB_FZ1_DBG_I2C1_STOP)
#define __HAL_DBGMCU_FREEZE_I2C1()            (DBGMCU->APBFZ1 |= (DBGMCU_APB_FZ1_DBG_I2C1_STOP))
#define __HAL_DBGMCU_UNFREEZE_I2C1()          (DBGMCU->APBFZ1 &= ~(DBGMCU_APB_FZ1_DBG_I2C1_STOP))
#endif /* DBGMCU_APB_FZ1_DBG_I2C1_STOP */

#if defined(DBGMCU_APB_FZ1_DBG_LPTIM_STOP)
#define __HAL_DBGMCU_FREEZE_LPTIM()            (DBGMCU->APBFZ1 |= (DBGMCU_APB_FZ1_DBG_LPTIM_STOP))
#define __HAL_DBGMCU_UNFREEZE_LPTIM()          (DBGMCU->APBFZ1 &= ~(DBGMCU_APB_FZ1_DBG_LPTIM_STOP))
#endif /* DBGMCU_APB_FZ1_DBG_LPTIM_STOP */

#if defined(DBGMCU_APB_FZ2_DBG_TIM1_STOP)
#define __HAL_DBGMCU_FREEZE_TIM1()           (DBGMCU->APBFZ2 |= (DBGMCU_APB_FZ2_DBG_TIM1_STOP))
#define __HAL_DBGMCU_UNFREEZE_TIM1()         (DBGMCU->APBFZ2 &= ~(DBGMCU_APB_FZ2_DBG_TIM1_STOP))
#endif /* DBGMCU_APB_FZ2_DBG_TIM1_STOP */

#if defined(DBGMCU_APB_FZ2_DBG_TIM14_STOP)
#define __HAL_DBGMCU_FREEZE_TIM14()          (DBGMCU->APBFZ2 |= (DBGMCU_APB_FZ2_DBG_TIM14_STOP))
#define __HAL_DBGMCU_UNFREEZE_TIM14()        (DBGMCU->APBFZ2 &= ~(DBGMCU_APB_FZ2_DBG_TIM14_STOP))
#endif /* DBGMCU_APB_FZ2_DBG_TIM14_STOP */
/**
  * @}
  */

/** @defgroup HAL_Private_Macros HAL Private Macros
  * @{
  */
#define IS_TICKFREQ(FREQ) (((FREQ) == HAL_TICK_FREQ_10HZ)  || \
                           ((FREQ) == HAL_TICK_FREQ_100HZ) || \
                           ((FREQ) == HAL_TICK_FREQ_1KHZ))
/** @brief  Enable or disable COMP1 output as TIM1 ocref_Clr input.
  */
#define __HAL_SYSCFG_COMP1_OCREF_CLR_TIM1_ENABLE()           SET_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_COMP1_OCREF_CLR_TIM1)
#define __HAL_SYSCFG_COMP1_OCREF_CLR_TIM1_DISABLE()          CLEAR_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_COMP1_OCREF_CLR_TIM1)

/** @brief  Enable or disable COMP2 output as TIM1 ocref_Clr input.
  */
#define __HAL_SYSCFG_COMP2_OCREF_CLR_TIM1_ENABLE()           SET_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_COMP2_OCREF_CLR_TIM1)
#define __HAL_SYSCFG_COMP2_OCREF_CLR_TIM1_DISABLE()          CLEAR_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_COMP2_OCREF_CLR_TIM1)


/** @brief  SYSCFG Break Cortex-M0+ Lockup lock.
  *         Enables and locks the connection of Cortex-M0+ LOCKUP (Hardfault) output to TIM1 Break input
  * @note   The selected configuration is locked and can be unlocked only by system reset.
  */
#define __HAL_SYSCFG_BREAK_LOCKUP_LOCK()        SET_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_LOCKUP_LOCK)

#if defined(SYSCFG_CFGR2_COMP1_BRK_TIM1)
/** @brief  COMP1 as Timer1 Break input
  */
#define __HAL_SYSCFG_COMP1_BREAK_TIM1()           SET_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_COMP1_BRK_TIM1)
#endif

#if defined(SYSCFG_CFGR2_COMP2_BRK_TIM1)
/** @brief  COMP2 as Timer1 Break input
  */
#define __HAL_SYSCFG_COMP2_BREAK_TIM1()           SET_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_COMP2_BRK_TIM1)
#endif

/* Initialization and de-initialization functions  ******************************/
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
/**
  * @}
  */

/** @addtogroup HAL_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  ************************************************/
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(uint32_t Freq);
uint32_t HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
void HAL_DBGMCU_EnableDBGMCUStopMode(void);
void HAL_DBGMCU_DisableDBGMCUStopMode(void);
void HAL_SYSCFG_SetRemapMemory(uint32_t Memory);
uint32_t HAL_SYSCFG_GetRemapMemory(void);
void HAL_SYSCFG_SetTIM1CH1Source(uint32_t Source);
uint32_t HAL_SYSCFG_GetTIM1CH1Source(void);
void HAL_SYSCFG_EnableI2CFastModePlus(uint32_t I2CFastModePlus);
void HAL_SYSCFG_DisableI2CFastModePlus(uint32_t I2CFastModePlus);
void HAL_SYSCFG_TIM1ETRSource(uint32_t ETRSource);
void HAL_SYSCFG_EnableGPIONoiseFilter(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
void HAL_SYSCFG_DisableGPIONoiseFilter(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup HAL_Private_Variables HAL Private Variables
  * @{
  */
/**
  * @}
  */
/* Private constants ---------------------------------------------------------*/
/** @defgroup HAL_Private_Constants HAL Private Constants
  * @{
  */
/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __PY32F002B_HAL_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
