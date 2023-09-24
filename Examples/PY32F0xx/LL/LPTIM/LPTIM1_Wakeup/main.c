/**
 * Demo of Low Power Timer Wakeup
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

static void APP_GPIO_Config(void);
static void APP_ConfigLPTIMOneShot(void);

int main(void)
{
  // Set HSI 24MHz as system clock source
  BSP_RCC_HSI_24MConfig();
  // Initialize UART on PA2:TX PA3:RX
  BSP_USART_Config(115200);
  printf("PY32F0 LPTIM Wakeup Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIO_Config();

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  APP_ConfigLPTIMOneShot();

  while (1)
  {
    LL_PWR_EnableLowPowerRunMode();

    LL_LPTIM_Disable(LPTIM1);
    LL_LPTIM_Enable(LPTIM1);
    LL_mDelay(1);

    LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);
    printf("Entering stop mode ...");
    LL_LPM_EnableDeepSleep();
    __WFI();
    printf("wakeup\r\n");
  }
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

static void APP_ConfigLPTIMOneShot(void)
{
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() == 0);
  // Set LSI as LPTIM1 clock source
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);
  // Prescaler = 64
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV128);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  LL_LPTIM_Enable(LPTIM1);


  /* 32768 / 128 = 256 */
  LL_LPTIM_SetAutoReload(LPTIM1, 255);

  NVIC_EnableIRQ(LPTIM1_IRQn);
  NVIC_SetPriority(LPTIM1_IRQn, 0);
}

void LPTIM1_IRQHandler(void)
{
  if (LL_LPTIM_IsActiveFlag_ARRM(LPTIM))
  {
    LL_LPTIM_ClearFLAG_ARRM(LPTIM);
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
    LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);
  }
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
