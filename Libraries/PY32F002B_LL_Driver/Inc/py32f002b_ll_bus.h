/**
  ******************************************************************************
  * @file    py32f002b_ll_bus.h
  * @author  MCU Application Team
  * @brief   Header file of BUS LL module.

  @verbatim
                      ##### RCC Limitations #####
  ==============================================================================
    [..]
      A delay between an RCC peripheral clock enable and the effective peripheral
      enabling should be taken into account in order to manage the peripheral read/write
      from/to registers.
      (+) This delay depends on the peripheral mapping.
        (++) AHB & APB1 peripherals, 1 dummy read is necessary

    [..]
      Workarounds:
      (#) For AHB & APB1 peripherals, a dummy read to the peripheral register has been
          inserted in each LL_{BUS}_GRP{x}_EnableClock() function.

  @endverbatim
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
#ifndef __PY32F002B_LL_BUS_H
#define __PY32F002B_LL_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx.h"

/** @addtogroup PY32F002B_LL_Driver
  * @{
  */

#if defined(RCC)

/** @defgroup BUS_LL BUS
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup BUS_LL_Exported_Constants BUS Exported Constants
  * @{
  */

/** @defgroup BUS_LL_EC_AHB1_GRP1_PERIPH  AHB1 GRP1 PERIPH
  * @{
  */
#define LL_AHB1_GRP1_PERIPH_ALL            0xFFFFFFFFU
#define LL_AHB1_GRP1_PERIPH_FLASH          RCC_AHBENR_FLASHEN
#define LL_AHB1_GRP1_PERIPH_SRAM           RCC_AHBENR_SRAMEN
#define LL_AHB1_GRP1_PERIPH_CRC            RCC_AHBENR_CRCEN
/**
  * @}
  */


/** @defgroup BUS_LL_EC_APB1_GRP1_PERIPH  APB1 GRP1 PERIPH
  * @{
  */
#define LL_APB1_GRP1_PERIPH_ALL            0xFFFFFFFFU
#define LL_APB1_GRP1_PERIPH_I2C1           RCC_APBENR1_I2CEN
#define LL_APB1_GRP1_PERIPH_DBGMCU         RCC_APBENR1_DBGEN
#define LL_APB1_GRP1_PERIPH_PWR            RCC_APBENR1_PWREN
#define LL_APB1_GRP1_PERIPH_LPTIM1         RCC_APBENR1_LPTIMEN
/**
  * @}
  */

/** @defgroup BUS_LL_EC_APB1_GRP2_PERIPH  APB1 GRP2 PERIPH
  * @{
  */
#define LL_APB1_GRP2_PERIPH_ALL            0xFFFFFFFFU
#define LL_APB1_GRP2_PERIPH_SYSCFG         RCC_APBENR2_SYSCFGEN
#define LL_APB1_GRP2_PERIPH_TIM1           RCC_APBENR2_TIM1EN
#define LL_APB1_GRP2_PERIPH_SPI1           RCC_APBENR2_SPI1EN
#define LL_APB1_GRP2_PERIPH_USART1         RCC_APBENR2_USART1EN
#if defined(TIM14)
#define LL_APB1_GRP2_PERIPH_TIM14          RCC_APBENR2_TIM14EN
#endif
#define LL_APB1_GRP2_PERIPH_ADC1           RCC_APBENR2_ADCEN
#if defined(COMP1)
#define LL_APB1_GRP2_PERIPH_COMP1          RCC_APBENR2_COMP1EN
#endif
#if defined(COMP2)
#define LL_APB1_GRP2_PERIPH_COMP2          RCC_APBENR2_COMP2EN
#endif
/**
  * @}
  */

/** @defgroup BUS_LL_EC_IOP_GRP1_PERIPH  IOP GRP1 PERIPH
  * @{
  */
#define LL_IOP_GRP1_PERIPH_ALL             0xFFFFFFFFU
#define LL_IOP_GRP1_PERIPH_GPIOA           RCC_IOPENR_GPIOAEN
#define LL_IOP_GRP1_PERIPH_GPIOB           RCC_IOPENR_GPIOBEN
#define LL_IOP_GRP1_PERIPH_GPIOC           RCC_IOPENR_GPIOCEN
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup BUS_LL_Exported_Functions BUS Exported Functions
  * @{
  */

/** @defgroup BUS_LL_EF_AHB1 AHB1
  * @{
  */

/**
  * @brief  Enable AHB1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_AHB1_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->AHBENR, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB1 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval State of Periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_AHB1_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return ((READ_BIT(RCC->AHBENR, Periphs) == Periphs) ? 1UL : 0UL);
}

/**
  * @brief  Disable AHB1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_AHB1_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->AHBENR, Periphs);
}

/**
  * @brief  Force AHB1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_AHB1_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->AHBRSTR, Periphs);
}

/**
  * @brief  Release AHB1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_AHB1_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->AHBRSTR, Periphs);
}
/**
  * @}
  */

/** @defgroup BUS_LL_EF_APB1_GRP1 APB1 GRP1
  * @{
  */

/**
  * @brief  Enable APB1 GRP1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_DBGMCU
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_LPTIM1
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_APB1_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->APBENR1, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APBENR1, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 GRP1 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_DBGMCU
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_LPTIM1
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval State of Periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB1_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return ((READ_BIT(RCC->APBENR1, Periphs) == (Periphs)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB1 GRP1 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_DBGMCU
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_LPTIM1
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_APB1_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APBENR1, Periphs);
}

/**
  * @brief  Force APB1 GRP1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_DBGMCU
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_LPTIM1
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_APB1_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->APBRSTR1, Periphs);
}

/**
  * @brief  Release APB1 GRP1 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_DBGMCU
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_LPTIM1
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_APB1_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APBRSTR1, Periphs);
}
/**
  * @}
  */

/** @defgroup BUS_LL_EF_APB1_GRP2 APB1 GRP2
  * @{
  */

/**
  * @brief  Enable APB1 GRP2 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_SYSCFG
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM1
  *         @arg @ref LL_APB1_GRP2_PERIPH_SPI1
  *         @arg @ref LL_APB1_GRP2_PERIPH_USART1
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM14
  *         @arg @ref LL_APB1_GRP2_PERIPH_ADC1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP2
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_APB1_GRP2_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->APBENR2, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APBENR2, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 GRP2 peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_SYSCFG
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM1
  *         @arg @ref LL_APB1_GRP2_PERIPH_SPI1
  *         @arg @ref LL_APB1_GRP2_PERIPH_USART1
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM14
  *         @arg @ref LL_APB1_GRP2_PERIPH_ADC1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP2
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval State of Periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_APB1_GRP2_IsEnabledClock(uint32_t Periphs)
{
  return ((READ_BIT(RCC->APBENR2, Periphs) == (Periphs)) ? 1UL : 0UL);
}

/**
  * @brief  Disable APB1 GRP2 peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_SYSCFG
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM1
  *         @arg @ref LL_APB1_GRP2_PERIPH_SPI1
  *         @arg @ref LL_APB1_GRP2_PERIPH_USART1
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM14
  *         @arg @ref LL_APB1_GRP2_PERIPH_ADC1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP2
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_APB1_GRP2_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APBENR2, Periphs);
}

/**
  * @brief  Force APB1 GRP2 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_SYSCFG
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM1
  *         @arg @ref LL_APB1_GRP2_PERIPH_SPI1
  *         @arg @ref LL_APB1_GRP2_PERIPH_USART1
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM14
  *         @arg @ref LL_APB1_GRP2_PERIPH_ADC1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP2
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @retval None
  */
__STATIC_INLINE void LL_APB1_GRP2_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->APBRSTR2, Periphs);
}

/**
  * @brief  Release APB1 GRP2 peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP2_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP2_PERIPH_SYSCFG
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM1
  *         @arg @ref LL_APB1_GRP2_PERIPH_SPI1
  *         @arg @ref LL_APB1_GRP2_PERIPH_USART1
  *         @arg @ref LL_APB1_GRP2_PERIPH_TIM14
  *         @arg @ref LL_APB1_GRP2_PERIPH_ADC1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP1
  *         @arg @ref LL_APB1_GRP2_PERIPH_COMP2
  * @note   Depending on devices and packages, some peripherals may not be available.
  *         Refer to device datasheet for peripherals availability.
  * @note (*) peripheral not available on all devices
  * @retval None
  */
__STATIC_INLINE void LL_APB1_GRP2_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APBRSTR2, Periphs);
}

/**
  * @}
  */

/** @defgroup BUS_LL_EF_IOP IOP
  * @{
  */

/**
  * @brief  Enable IOP peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOC
  * @retval None
  */
__STATIC_INLINE void LL_IOP_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->IOPENR, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->IOPENR, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if IOP peripheral clock is enabled or not
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOC
  * @retval State of Periphs (1 or 0).
  */
__STATIC_INLINE uint32_t LL_IOP_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return ((READ_BIT(RCC->IOPENR, Periphs) == Periphs) ? 1UL : 0UL);
}

/**
  * @brief  Disable IOP peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOC
  * @retval None
  */
__STATIC_INLINE void LL_IOP_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->IOPENR, Periphs);
}

/**
  * @brief  Disable IOP peripherals clock.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_IOP_GRP1_PERIPH_ALL
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOC
  * @retval None
  */
__STATIC_INLINE void LL_IOP_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->IOPRSTR, Periphs);
}

/**
  * @brief  Release IOP peripherals reset.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_IOP_GRP1_PERIPH_ALL
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_IOP_GRP1_PERIPH_GPIOC
  * @retval None
  */
__STATIC_INLINE void LL_IOP_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->IOPRSTR, Periphs);
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

#endif /* RCC */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* PY32F002B_LL_BUS_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
