#include "py32f002b_bsp_clock.h"

void BSP_RCC_HSI_24MConfig(void)
{
  /*  Set FLASH Latency Before modifying the HSI */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  /* SET HSI to 24MHz */
  LL_RCC_HSI_SetCalibTrimming(LL_RCC_HSICALIBRATION_24MHz);
  /* Enable HSI */
  LL_RCC_HSI_Enable();

  while(LL_RCC_HSI_IsReady() != 1);
  /* Set AHB divider: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  /* HSISYS used as SYSCLK clock source  */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Set the 1ms time base of SysTick */
  LL_Init1msTick(24000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
}

void BSP_RCC_HSI_48MConfig(void)
{
  /*  Set FLASH Latency = 1 for 48MHz */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  /* SET HSI to 24MHz */
  LL_RCC_HSI_SetCalibTrimming(LL_RCC_HSICALIBRATION_48MHz);
  /* Enable HSI */
  LL_RCC_HSI_Enable();

  while(LL_RCC_HSI_IsReady() != 1);
  /* Set AHB divider: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  /* HSISYS used as SYSCLK clock source  */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(48000000);
  LL_SetSystemCoreClock(48000000);
}

void BSP_RCC_LSE_Config(void)
{
  /* Enable Low Speed External (LSE) crystal */
  LL_RCC_LSE_Enable();
  while(LL_RCC_LSE_IsReady() != 1);

  /* Set AHB divider:HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  /* Set FLASH Latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  /* Set LSE as SYSCLK clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_LSE);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_LSE);

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(LSE_VALUE);
  LL_SetSystemCoreClock(LSE_VALUE);
}

void APP_RCC_LSI_32K768Config(void)
{
    /* Set LSI to 32.768kHZ */
  LL_RCC_LSI_SetCalibTrimming(LL_RCC_LSICALIBRATION_32768Hz);
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1);

  /* Set AHB divider:HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  /* Set FLASH Latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  /* Set LSI as SYSCLK clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_LSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_LSI);

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(LSI_VALUE);
  LL_SetSystemCoreClock(LSI_VALUE);
}

void APP_RCC_LSI_38K4Config(void)
{
    /* Set LSI to 38.4kHZ */
  LL_RCC_LSI_SetCalibTrimming(LL_RCC_LSICALIBRATION_38400Hz);
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1);

  /* Set AHB divider:HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  /* Set FLASH Latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  /* Set LSI as SYSCLK clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_LSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_LSI);

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(38400);
  LL_SetSystemCoreClock(38400);
}