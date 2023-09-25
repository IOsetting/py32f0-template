/**
  ******************************************************************************
  * @file    py32f002b_hal_gpio_ex.h
  * @author  MCU Application Team
  * @brief   Header file of GPIO HAL Extended module.
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
#ifndef __PY32F002B_HAL_GPIO_EX_H
#define __PY32F002B_HAL_GPIO_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f002b_hal_def.h"

/** @addtogroup PY32F002B_HAL_Driver
  * @{
  */

/** @defgroup GPIOEx GPIOEx
  * @brief GPIO Extended HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIOEx_Exported_Constants GPIOEx Exported Constants
  * @{
  */

/** @defgroup GPIOEx_Alternate_function_selection GPIOEx Alternate function selection
  * @{
  */

/**
  * @brief   AF 0 selection
  */
#define GPIO_AF0_SPI1          (0x0000000U)   /*!< SPI1 Alternate Function mapping */
#define GPIO_AF0_SWJ           (0x0000000U)   /*!< SWJ (SWD) Alternate Function mapping */

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_USART1        (0x0000001U)   /*!< USART1 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          (0x0000002U)   /*!< TIM1 Alternate Function mapping */
#define GPIO_AF2_SPI1          (0x0000002U)   /*!< SPI1 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_USART1        (0x0000003U)   /*!< USART1 Alternate Function mapping*/
#define GPIO_AF3_TIM1          (0x0000003U)   /*!< TIM1 Alternate Function mapping*/

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_MCO           (0x0000004U)   /*!< MCO Alternate Function mapping*/
#define GPIO_AF4_COMP1         (0x0000004U)   /*!< COMP1 Alternate Function mapping*/
#define GPIO_AF4_COMP2         (0x0000004U)   /*!< COMP2 Alternate Function mapping*/

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_TIM14         (0x0000005U)   /*!< TIM14 Alternate Function mapping*/

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_I2C1           (0x0000006U)   /*!< I2C1 Alternate Function mapping */

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_EVENTOUT      (0x0000007U)   /*!< EVENTOUT Alternate Function mapping */
#define IS_GPIO_AF(AF)         ((AF) <= (uint8_t)0x07)
/**
  * @}
  */

/**
  * @}
  */


#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL : 2uL)


/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __PY32F002B_HAL_GPIO_EX_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
