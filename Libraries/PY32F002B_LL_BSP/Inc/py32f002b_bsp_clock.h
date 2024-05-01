/**
  ******************************************************************************
  * @file    py32f002b_bsp_clock.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PY32F002B_BSP_CLOCK_H
#define PY32F002B_BSP_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "py32f002b_ll_rcc.h"
#include "py32f002b_ll_bus.h"
#include "py32f002b_ll_system.h"
#include "py32f002b_ll_exti.h"
#include "py32f002b_ll_cortex.h"
#include "py32f002b_ll_utils.h"
#include "py32f002b_ll_pwr.h"
#include "py32f002b_ll_gpio.h"
#include "py32f002b_ll_usart.h"

void BSP_RCC_HSI_24MConfig(void);
void BSP_RCC_HSI_48MConfig(void);
void BSP_RCC_LSE_Config(void);
void APP_RCC_LSI_32K768Config(void);
void APP_RCC_LSI_38K4Config(void);


#ifdef __cplusplus
}
#endif

#endif /* PY32F002B_BSP_CLOCK_H */
