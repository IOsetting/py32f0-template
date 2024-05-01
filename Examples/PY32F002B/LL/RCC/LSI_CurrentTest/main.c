/**
 * Run Mode Current Under HSI and LSI
 * 
 * Connections
 *  VCC      ---> Microammeter --> 3.3V
 *  GND      ---> GND 
 * 
 * - Don't connect any other pins to avoid possible current leakage
 * - MCU switches to LSI after 3 seconds, then turn off HSI and enable flash sleep, 
 *   finally the current will drop to around 85 uA
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"


void APP_RCC_LSI_32K768Config(void)
{
  /* 
   * Set LSI to 32.768kHz
   * (or 38.4KHz with LL_RCC_LSICALIBRATION_38400Hz)
   */
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
  LL_SetSystemCoreClock(LSI_VALUE);
  LL_Init1msTick(32768);
}

int main(void)
{
  /* Configure HSI as Systemclock source */
  BSP_RCC_HSI_24MConfig();
  // The current is around 1.2 mA
  LL_mDelay(3000);

  /* Switch to LSI */
  APP_RCC_LSI_32K768Config();
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
