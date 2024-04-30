/**
 * Run Mode Current Under HSI and LSI
 * 
 * Connections
 *  VCC      ---> Microammeter --> 3.3V
 *  GND      ---> GND 
 *  
 * - Don't connect any other pins to avoid possible current leakage
 * - MCU switches to LSI after 3 seconds, then turn off HSI, enable flash sleep, 
 *   finally the current will drop to around 100 uA
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"


static void APP_RCC_LSI_Config(void)
{
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1);
  
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  
  /* Set LSI as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_LSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_LSI);
  
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_SetSystemCoreClock(LSI_VALUE);
  /* Re-init frequency of SysTick source */
  LL_Init1msTick(32768);
}

int main(void)
{
  // Set HSI 24MHz as system clock source
  BSP_RCC_HSI_24MConfig();
  // Run under HSI, the current is around 1.3 mA
  LL_mDelay(3000);
  // Switch system clock source to LSI
  APP_RCC_LSI_Config();
  // Run under LSI, but HSI is still on, the current drops to around 360 uA
  LL_mDelay(3000);
  // Turn off HSI
  LL_RCC_HSI_Disable();
  // The current drops to around 180 uA
  LL_mDelay(3000);
  // Enable flash sleep
  SET_BIT(FLASH->STCR, FLASH_STCR_SLEEP_EN);
  // The current drops to around 100 uA
  while (1);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
