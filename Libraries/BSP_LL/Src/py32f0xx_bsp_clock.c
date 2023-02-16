#include "py32f0xx_bsp_clock.h"

void BSP_HSI_PLL_48MConfig(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSI_Enable();
  /* Change this value to adjust clock frequency, larger is faster */
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while (LL_RCC_HSI_IsReady() != 1);

  UTILS_ClkInitStruct.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
  UTILS_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_1;
  LL_PLL_ConfigSystemClock_HSI(&UTILS_ClkInitStruct);

  /* Re-init frequency of SysTick source, reload = freq/ticks = 48000000/1000 = 48000 */
  LL_InitTick(48000000, 1000U);
}

void BSP_HSI_24MConfig(void)
{
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while(LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update global SystemCoreClock(or through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
  /* Re-init frequency of SysTick source */
  LL_InitTick(24000000, 1000U);
}

void BSP_HSE_PLL_Config(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSE_Enable();
  LL_RCC_HSE_SetFreqRegion(LL_RCC_HSE_16_32MHz);
  while(LL_RCC_HSE_IsReady() != 1);

  UTILS_ClkInitStruct.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
  UTILS_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_1;
  LL_PLL_ConfigSystemClock_HSE(HSE_VALUE, LL_UTILS_HSEBYPASS_OFF, &UTILS_ClkInitStruct);
  /* Re-init frequency of SysTick source */
  LL_InitTick(HSE_VALUE * 2, 1000U);
}

void BSP_HSE_Config(void)
{
  LL_RCC_HSE_Enable();
  LL_RCC_HSE_SetFreqRegion(LL_RCC_HSE_16_32MHz);
  while(LL_RCC_HSE_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update global SystemCoreClock(or through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(HSE_VALUE);
  /* Re-init frequency of SysTick source */
  LL_InitTick(HSE_VALUE, 1000U);
}