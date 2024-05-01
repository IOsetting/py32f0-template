/**
 * Run Mode Current Under HSI and LSE
 * 
 * Connections
 *  VCC      ---> Microammeter --> 3.3V
 *  GND      ---> GND 
 *  
 * - Don't connect any other pins to avoid possible current leakage
 * - MCU switches to LSE after 3 seconds, then turn off HSI and enable flash sleep, 
 *   finally the current will drop to around 85 uA
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"


void APP_RCC_LSE_Config(void)
{
  /* Enable Low Speed External (LSE) crystal */
  LL_RCC_LSE_Enable();
  while(LL_RCC_LSE_IsReady() != 1);

  /* Set AHB divider:HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Set FLASH Latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  /* LSE used as SYSCLK clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_LSE);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_LSE);

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(LSE_VALUE);

  /* Update the 1ms time base of SysTick source */
  LL_Init1msTick(32768);
}

int main(void)
{
  /* Configure HSI as Systemclock source */
  BSP_RCC_HSI_24MConfig();
  // The current is around 1.2 mA
  LL_mDelay(3000);

  /* Switch to LSE */
  APP_RCC_LSE_Config();
  // The current drops to around 350 uA
  LL_mDelay(3000);

  // Turn off HSI
  LL_RCC_HSI_Disable();
  // The current drops to around 170 uA
  LL_mDelay(3000);

  // Enable flash sleep
  SET_BIT(FLASH->STCR, FLASH_STCR_SLEEP_EN);
  // The current drops to around 85 uA
  while (1);
}

void APP_ErrorHandler(void)
{
  while (1);
}
