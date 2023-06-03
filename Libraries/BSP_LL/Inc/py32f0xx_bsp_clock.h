/**
  ******************************************************************************
  * @file    py32f0xx_bsp_clock.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F003_BSP_CLOCK_H
#define PY32F003_BSP_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "py32f0xx_ll_rcc.h"
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_system.h"
#include "py32f0xx_ll_exti.h"
#include "py32f0xx_ll_cortex.h"
#include "py32f0xx_ll_utils.h"
#include "py32f0xx_ll_pwr.h"
#include "py32f0xx_ll_dma.h"
#include "py32f0xx_ll_gpio.h"
#include "py32f0xx_ll_usart.h"

void BSP_RCC_HSI_24MConfig(void);
void BSP_RCC_HSI_8MConfig(void);
void BSP_RCC_HSE_Config(void);

#if defined(RCC_PLL_SUPPORT)
void BSP_RCC_HSI_PLL48MConfig(void);
void BSP_RCC_HSE_PLLConfig(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* PY32F003_BSP_CLOCK_H */
