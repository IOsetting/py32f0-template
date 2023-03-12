#include "py32f0xx_bsp_clock.h"


void BSP_RCC_HSI_24MConfig(void)
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
  LL_Init1msTick(24000000);
}

void BSP_RCC_HSI_8MConfig(void)
{
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_SetSystemCoreClock(8000000);
  LL_Init1msTick(8000000);
}

void BSP_RCC_HSE_Config(void)
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
  LL_Init1msTick(HSE_VALUE);
}

#if defined(RCC_PLL_SUPPORT)
void BSP_RCC_HSI_PLL48MConfig(void)
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
  LL_Init1msTick(48000000);
}

void BSP_RCC_HSE_PLLConfig(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSE_Enable();
  LL_RCC_HSE_SetFreqRegion(LL_RCC_HSE_16_32MHz);
  while(LL_RCC_HSE_IsReady() != 1);

  UTILS_ClkInitStruct.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
  UTILS_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_1;
  LL_PLL_ConfigSystemClock_HSE(HSE_VALUE, LL_UTILS_HSEBYPASS_OFF, &UTILS_ClkInitStruct);
  /* Re-init frequency of SysTick source */
  LL_Init1msTick(HSE_VALUE * 2);
}
#endif