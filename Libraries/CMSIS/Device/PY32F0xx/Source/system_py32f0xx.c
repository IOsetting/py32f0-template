/**
  ******************************************************************************
  * @file    system_py32f0xx.c
  * @author  MCU Application Team
  * @Version V1.0.0
  * @Date    2020-10-19
  * @brief   CMSIS Cortex-M0+ Device Peripheral Access Layer System Source File.
  ******************************************************************************
  */

#include "py32f0xx.h"

#if !defined  (HSE_VALUE)
#define HSE_VALUE    24000000U    /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE  8000000U   /*!< Value of the Internal oscillator in Hz*/
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
const uint32_t HSIFreqTable[8] = {4000000U, 8000000U, 16000000U, 22120000U, 24000000U, 4000000U, 4000000U, 4000000U};

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
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
#endif
#if defined(RCC_PLL_SUPPORT)
  case RCC_CFGR_SWS_1:  /* PLL used as system clock */
    if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI) /* HSI used as PLL clock source */
    {
      hsifs = ((READ_BIT(RCC->ICSCR, RCC_ICSCR_HSI_FS)) >> RCC_ICSCR_HSI_FS_Pos);
      SystemCoreClock = 2 * (HSIFreqTable[hsifs]);
    }
    else   /* HSE used as PLL clock source */
    {
      SystemCoreClock = 2 * HSE_VALUE;
    }
    break;
#endif
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
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit(void)
{
  /* Set the HSI clock to 8MHz by default */
  RCC->ICSCR = (RCC->ICSCR & 0xFFFF0000) | (0x1 << 13) | *(uint32_t *)(0x1fff0f04);

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}

#ifndef FORBID_VECT_TAB_MIGRATION
#ifndef VECT_TAB_SRAM
#if (defined (__CC_ARM)) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
extern int32_t $Super$$main(void);
uint32_t VECT_SRAM_TAB[48]__attribute__((section(".ARM.__at_0x20000000")));

/* re-define main function */
int $Sub$$main(void)
{
  uint8_t i;
  uint32_t *pFmcVect = (uint32_t *)(FLASH_BASE | VECT_TAB_OFFSET);
  for (i = 0; i < 48; i++)
  {
    VECT_SRAM_TAB[i] = pFmcVect[i];
  }

  SCB->VTOR = SRAM_BASE;

  $Super$$main();
  return 0;
}
#elif defined(__ICCARM__)
extern int32_t main(void);
/* __low_level_init will auto called by IAR cstartup */
extern void __iar_data_init3(void);
uint32_t VECT_SRAM_TAB[48] @SRAM_BASE;
int __low_level_init(void)
{
  uint8_t i;
  uint32_t *pFmcVect = (uint32_t *)(FLASH_BASE | VECT_TAB_OFFSET);
  /* call IAR table copy function. */
  __iar_data_init3();

  for (i = 0; i < 48; i++)
  {
    VECT_SRAM_TAB[i] = pFmcVect[i];
  }

  SCB->VTOR = SRAM_BASE;

  main();
  return 0;
}
#endif
#endif
#endif
