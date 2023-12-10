/**
  ******************************************************************************
  * @file    py32f07x_bsp_clock.h
  * @author  MCU Application Team
  * @brief   
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F07X_BSP_CLOCK_H
#define PY32F07X_BSP_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "py32f07x_hal.h"

HAL_StatusTypeDef BSP_HSI_8MHzClockConfig(void);
HAL_StatusTypeDef BSP_HSI_24MHzClockConfig(void);
HAL_StatusTypeDef BSP_HSI_PLL_72MHzClockConfig(void);

HAL_StatusTypeDef BSP_HSE_ClockConfig(void);
HAL_StatusTypeDef BSP_HSE_PLLx2_ClockConfig(void);
HAL_StatusTypeDef BSP_HSE_PLLx3_ClockConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* PY32F0XX_BSP_CLOCK_H */
