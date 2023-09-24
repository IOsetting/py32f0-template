/**
  ******************************************************************************
  * @file    py32f0xx_bsp_clock.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F0XX_BSP_CLOCK_H
#define PY32F0XX_BSP_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "py32f0xx_hal.h"
#include "py32f0xx_hal_rcc.h"

HAL_StatusTypeDef BSP_HSI_24MHzClockConfig(void);
HAL_StatusTypeDef BSP_HSE_ClockConfig(void);

#if defined(RCC_PLL_SUPPORT)
HAL_StatusTypeDef BSP_HSI_PLL_48MHzClockConfig(void);
HAL_StatusTypeDef BSP_HSI_PLL_32MHzClockConfig(void);
HAL_StatusTypeDef BSP_HSE_PLL_ClockConfig(void);
#endif


#ifdef __cplusplus
}
#endif

#endif /* PY32F0XX_BSP_CLOCK_H */
