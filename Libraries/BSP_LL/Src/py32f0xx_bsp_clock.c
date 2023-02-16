#include "py32f0xx_bsp_clock.h"

void BSP_HSI_48MConfig(void)
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

