/**
  ******************************************************************************
  * @file    py32f0xx_hal_gpio_ex.h
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
#ifndef __PY32F0xx_HAL_GPIO_EX_H
#define __PY32F0xx_HAL_GPIO_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_hal_def.h"

/** @addtogroup PY32F0xx_HAL_Driver
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
/*------------------------- PY32F0xx ------------------------*/
#if (defined(PY32F003PRE)|| defined(PY32F030PRE))
/**
  * @brief   AF 0 selection
  */

#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /*!< SWJ (SWD) Alternate Function mapping */
#define GPIO_AF0_SPI1          ((uint8_t)0x00)  /*!< SPI1 Alternate Function mapping */
#define GPIO_AF0_SPI2          ((uint8_t)0x00)  /*!< SPI2 Alternate Function mapping */
#define GPIO_AF0_TIM14         ((uint8_t)0x00)  /*!< TIM14 Alternate Function mapping */
#define GPIO_AF0_USART1        ((uint8_t)0x00)  /*!< USART1 Alternate Function mapping */

/**
  * @brief   AF 1 selection
  */

#define GPIO_AF1_IR            ((uint8_t)0x01)  /*!< IR Alternate Function mapping */
#define GPIO_AF1_SPI2          ((uint8_t)0x01)  /*!< SPI2 Alternate Function mapping */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)  /*!< TIM1 Alternate Function mapping */
#define GPIO_AF1_TIM3          ((uint8_t)0x01)  /*!< TIM3 Alternate Function mapping */
#define GPIO_AF1_USART1        ((uint8_t)0x01)  /*!< USART1 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_SPI2          ((uint8_t)0x02)  /*!< SPI2 Alternate Function mapping */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)  /*!< TIM1 Alternate Function mapping */
#define GPIO_AF2_TIM14         ((uint8_t)0x02)  /*!< TIM14 Alternate Function mapping */
#define GPIO_AF2_TIM16         ((uint8_t)0x02)  /*!< TIM16 Alternate Function mapping */
#define GPIO_AF2_TIM17         ((uint8_t)0x02)  /*!< TIM17 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_LED           ((uint8_t)0x03U)  /*!< AF3: LED Alternate Function mapping      */
#define GPIO_AF3_USART1        ((uint8_t)0x03U)  /*!< AF3: USART1 Alternate Function mapping      */
#define GPIO_AF3_USART2        ((uint8_t)0x03U)  /*!< AF3: USART2 Alternate Function mapping      */
#define GPIO_AF3_SPI2          ((uint8_t)0x03U)  /*!< AF3: SPI2 Alternate Function mapping      */
/**
  * @brief   AF 4 selection
  */

#define GPIO_AF4_TIM14         ((uint8_t)0x04)  /*!< TIM14 Alternate Function mapping */
#define GPIO_AF4_USART2        ((uint8_t)0x04)  /*!< USART2 Alternate Function mapping */
/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_LPTIM         ((uint8_t)0x05)  /*!< LPTIM1 Alternate Function mapping */
#define GPIO_AF5_USART2        ((uint8_t)0x05)  /*!< USART2 Alternate Function mapping */
#define GPIO_AF5_EVENTOUT      ((uint8_t)0x05)  /*!< EVENTOUT Alternate Function mapping */
#define GPIO_AF5_TIM16         ((uint8_t)0x05)  /*!< TIM16 Alternate Function mapping */
#define GPIO_AF5_TIM17         ((uint8_t)0x05)  /*!< TIM17 Alternate Function mapping */
#define GPIO_AF5_MCO           ((uint8_t)0x05)  /*!< MCO Alternate Function mapping */
/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_I2C          ((uint8_t)0x06)  /*!< I2C1 Alternate Function mapping */
#define GPIO_AF6_LED           ((uint8_t)0x06U)  /*!< AF6: LED Alternate Function mapping      */
#define GPIO_AF6_MCO           ((uint8_t)0x06)  /*!< MCO Alternate Function mapping */
#define GPIO_AF6_EVENTOUT     ((uint8_t)0x06U)  /*!< AF6: EVENTOUT Alternate Function mapping  */
/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_EVENTOUT      ((uint8_t)0x07)  /*!< EVENTOUT Alternate Function mapping */
#define GPIO_AF7_COMP1        ((uint8_t)0x07U)  /*!< AF7: COMP1 Alternate Function mapping     */
#define GPIO_AF7_COMP2        ((uint8_t)0x07U)  /*!< AF7: COMP2 Alternate Function mapping     */
/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_USART1        ((uint8_t)0x08U)  /*!< AF8: USART1 Alternate Function mapping      */
/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_USART2        ((uint8_t)0x09)  /*!< USART2 Alternate Function mapping */
/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_SPI1          ((uint8_t)0x0A)  /*!< SPI2 Alternate Function mapping */
/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_SPI2          ((uint8_t)0x0B)  /*!< SPI2 Alternate Function mapping */
/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_I2C          ((uint8_t)0x0C)  /*!< I2C1 Alternate Function mapping */
/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_TIM1          ((uint8_t)0x0D)  /*!< TIM1 Alternate Function mapping */
#define GPIO_AF13_TIM3          ((uint8_t)0x0D)  /*!< TIM3 Alternate Function mapping */
#define GPIO_AF13_TIM14         ((uint8_t)0x0D)  /*!< TIM14 Alternate Function mapping */
#define GPIO_AF13_TIM17         ((uint8_t)0x0D)  /*!< TIM17 Alternate Function mapping */

/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM1          ((uint8_t)0x0E)  /*!< TIM1 Alternate Function mapping */
/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_RTCOUT        ((uint8_t)0x0fU)
#define GPIO_AF15_MCO           ((uint8_t)0x0FU)  /*!< MCO Alternate Function mapping */
#define GPIO_AF15_IR            ((uint8_t)0x0FU)  /*!< IR Alternate Function mapping */
#define IS_GPIO_AF(AF)         ((AF) <= (uint8_t)0x0f)

#elif defined(PY32F002APRE)
/**
  * @brief   AF 0 selection
  */

#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /*!< SWJ (SWD) Alternate Function mapping */
#define GPIO_AF0_SPI1          ((uint8_t)0x00)  /*!< SPI1 Alternate Function mapping */
#define GPIO_AF0_USART1        ((uint8_t)0x00)  /*!< USART1 Alternate Function mapping */

/**
  * @brief   AF 1 selection
  */

#define GPIO_AF1_IR            ((uint8_t)0x01)  /*!< IR Alternate Function mapping */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)  /*!< TIM1 Alternate Function mapping */
#define GPIO_AF1_USART1        ((uint8_t)0x01)  /*!< USART1 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)  /*!< TIM1 Alternate Function mapping */
#define GPIO_AF2_TIM16         ((uint8_t)0x02)  /*!< TIM16 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_USART1        ((uint8_t)0x03U)  /*!< AF3: USART1 Alternate Function mapping      */
/**
  * @brief   AF 4 selection
  */

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_LPTIM         ((uint8_t)0x05)  /*!< LPTIM1 Alternate Function mapping */
#define GPIO_AF5_EVENTOUT      ((uint8_t)0x05)  /*!< EVENTOUT Alternate Function mapping */
#define GPIO_AF5_TIM16         ((uint8_t)0x05)  /*!< TIM16 Alternate Function mapping */
#define GPIO_AF5_MCO           ((uint8_t)0x05)  /*!< MCO Alternate Function mapping */
/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_I2C          ((uint8_t)0x06)  /*!< I2C1 Alternate Function mapping */
#define GPIO_AF6_MCO           ((uint8_t)0x06)  /*!< MCO Alternate Function mapping */
#define GPIO_AF6_EVENTOUT     ((uint8_t)0x06U)  /*!< AF6: EVENTOUT Alternate Function mapping  */
/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_EVENTOUT      ((uint8_t)0x07)  /*!< EVENTOUT Alternate Function mapping */
#define GPIO_AF7_COMP1        ((uint8_t)0x07U)  /*!< AF7: COMP1 Alternate Function mapping     */
#define GPIO_AF7_COMP2        ((uint8_t)0x07U)  /*!< AF7: COMP2 Alternate Function mapping     */
/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_USART1        ((uint8_t)0x08U)  /*!< AF8: USART1 Alternate Function mapping      */
/**
  * @brief   AF 9 selection
  */

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_SPI1          ((uint8_t)0x0A)  /*!< SPI2 Alternate Function mapping */
/**
  * @brief   AF 11 selection
  */

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_I2C          ((uint8_t)0x0C)  /*!< I2C1 Alternate Function mapping */
/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_TIM1          ((uint8_t)0x0D)  /*!< TIM1 Alternate Function mapping */

/**
  * @brief   AF 14 selection
  */
#define GPIO_AF14_TIM1          ((uint8_t)0x0E)  /*!< TIM1 Alternate Function mapping */
/**
  * @brief   AF 15 selection
  */
#define GPIO_AF15_MCO           ((uint8_t)0x0FU)  /*!< MCO Alternate Function mapping */
#define GPIO_AF15_IR            ((uint8_t)0x0FU)  /*!< IR Alternate Function mapping */
#define IS_GPIO_AF(AF)         ((AF) <= (uint8_t)0x0f)
#endif
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

#endif /* __PY32F0xx_HAL_GPIO_EX_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
