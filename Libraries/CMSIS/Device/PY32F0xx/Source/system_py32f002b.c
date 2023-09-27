/**
  ******************************************************************************
  * @file    system_py32f002b.c
  * @author  MCU Application Team
  * @Version V1.0.0
  * @Date    2020-10-19
  * @brief   CMSIS Cortex-M0+ Device Peripheral Access Layer System Source File.
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

#include "py32f0xx.h"

#if !defined  (HSE_VALUE)
#define HSE_VALUE    24000000U    /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE  24000000U   /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

#if !defined  (LSI_VALUE)
#define LSI_VALUE   32768U      /*!< Value of LSI in Hz*/
#endif /* LSI_VALUE */

#if !defined  (LSE_VALUE)
#define LSE_VALUE  32768U      /*!< Value of LSE in Hz*/
#endif /* LSE_VALUE */


/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define FORBID_VECT_TAB_MIGRATION */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x100. */
/******************************************************************************/
/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
/* This variable is updated in three ways:
    1) by calling CMSIS function SystemCoreClockUpdate()
    2) by calling HAL API function HAL_RCC_GetHCLKFreq()
    3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
       Note: If you use this function to configure the system clock; then there
             is no need to call the 2 first functions listed above, since SystemCoreClock
             variable is updated automatically.
*/
uint32_t SystemCoreClock = HSI_VALUE;

const uint32_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint32_t APBPrescTable[8] =  {0, 0, 0, 0, 1, 2, 3, 4};
#if defined(RCC_HSI48M_SUPPORT)
const uint32_t HSIFreqTable[8] = {0U, 0U, 0U, 0U, 24000000U, 48000000U, 0U, 0U};
#else
const uint32_t HSIFreqTable[8] = {0U, 0U, 0U, 0U, 24000000U, 0U, 0U, 0U};
#endif

/* Private function prototypes -----------------------------------------------*/
#ifndef SWD_DELAY
static void DelayTime(uint32_t mdelay);
#endif /* SWD_DELAY */

/**
 * @brief  Clock functions.
 * @param  none
 * @return none
 */
void SystemCoreClockUpdate(void)             /* Get Core Clock Frequency      */
{
  uint32_t tmp;
  uint32_t hsidiv;
  uint32_t hsifs;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
  case RCC_CFGR_SWS_0:  /* HSE used as system clock */
    SystemCoreClock = HSE_VALUE;
    break;

  case (RCC_CFGR_SWS_1 | RCC_CFGR_SWS_0):  /* LSI used as system clock */
    SystemCoreClock = LSI_VALUE;
    break;
#if defined(RCC_LSE_SUPPORT)
  case RCC_CFGR_SWS_2:  /* LSE used as system clock */
    SystemCoreClock = LSE_VALUE;
    break;
#endif /* RCC_LSE_SUPPORT */
  case 0x00000000U:  /* HSI used as system clock */
  default:                /* HSI used as system clock */
    hsifs = ((READ_BIT(RCC->ICSCR, RCC_ICSCR_HSI_FS)) >> RCC_ICSCR_HSI_FS_Pos);
    hsidiv = (1UL << ((READ_BIT(RCC->CR, RCC_CR_HSIDIV)) >> RCC_CR_HSIDIV_Pos));
    SystemCoreClock = (HSIFreqTable[hsifs] / hsidiv);
    break;
  }
  /* Compute HCLK clock frequency --------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

/**
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 * @param  none
 * @return none
 */
void SystemInit(void)
{
  /*Set the HSI clock to 24MHz by default*/
  RCC->ICSCR = (RCC->ICSCR & 0xFFFF0000) | (*(uint32_t *)(0x1FFF0100));

  /*Set the LSI clock to 32.768KHz by default*/
  RCC->ICSCR = (RCC->ICSCR & 0xFE00FFFF) | ((*(uint32_t *)(0x1FFF0144)) << RCC_ICSCR_LSI_TRIM_Pos);

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif /* VECT_TAB_SRAM */

#ifndef SWD_DELAY
  /* When the SWD pin is reused for other functions, this function is used to solve the 
  problem of not being able to update the code. */
  DelayTime(100);
#endif /* SWD_DELAY */
}

#ifndef SWD_DELAY
/**
  * @brief  This function provides delay (in milliseconds) based on CPU cycles method.
  * @param  mdelay: specifies the delay time length, in milliseconds.
  * @retval None
  */
static void DelayTime(uint32_t mdelay)
{
  __IO uint32_t Delay = mdelay * (24000000U / 8U / 1000U);
  do
  {
    __NOP();
  }
  while (Delay --);
}
#endif /* SWD_DELAY */
/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
