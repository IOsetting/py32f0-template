#include "py32f0xx_bsp_clock.h"

HAL_StatusTypeDef BSP_HSI_24MHzClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                            /* HSI ON */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;   /* Set HSI clock 24MHz */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                            /* No division */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                           /* OFF */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                           /* OFF */
  #if defined(RCC_LSE_SUPPORT)
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                           /* OFF */
  #endif
  #if defined(RCC_PLL_SUPPORT)
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                       /* OFF */
  #endif

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Reinitialize AHB,APB bus clock */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;              /* Select HSI as SYSCLK source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                  /* APH clock, no division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                   /* APB clock, no division */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef BSP_HSE_ClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                                     /* Turn on HSE */
  RCC_OscInitStruct.HSEFreq = (RCC_ECSCR_HSE_FREQ_0 | RCC_ECSCR_HSE_FREQ_1);   /* HSE frequency range */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return HAL_ERROR;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;                                        /* SYSCLK source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  /* 
   * Re-initialize RCC clock
   * -- clock <= 24MHz: FLASH_LATENCY_0
   * -- clock > 24MHz:  FLASH_LATENCY_1
   */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

#if defined(RCC_PLL_SUPPORT)
HAL_StatusTypeDef BSP_HSI_PLL_48MHzClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                            /* HSI ON */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                            /* No division */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;   /* HSI =16MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                           /* OFF */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                           /* OFF */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                           /* OFF */
  RCC_OscInitStruct.LSEDriver = RCC_ECSCR_LSE_DRIVER_1;               /* LSE default */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                        /* PLL ON */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;                /* PLL clock source (freq >= 12MHz) */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return HAL_ERROR;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;           /* Set PLL as SYSCLK source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                  /* APH no division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                   /* APB no division */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef BSP_HSI_PLL_32MHzClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                            /* HSI ON */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                            /* No division */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_16MHz;   /* HSI =16MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                           /* OFF */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                           /* OFF */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                           /* OFF */
  RCC_OscInitStruct.LSEDriver = RCC_ECSCR_LSE_DRIVER_1;               /* LSE default */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                        /* PLL ON */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;                /* PLL clock source (freq >= 12MHz) */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return HAL_ERROR;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;           /* Set PLL as SYSCLK source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                  /* APH no division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                   /* APB no division */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef BSP_HSE_PLL_ClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                            /* Turn on HSE */
  RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;                       /* HSE frequency range */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                        /* PLL ON */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;                /* PLL clock source from HSE (freq >= 12MHz) */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return HAL_ERROR;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;           /* Set PLL as SYSCLK source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                  /* APH no division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                   /* APB no division */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}
#endif
