/**
 * Window Watchdog Demo
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

#define WINDOW_IN             /* Inside watchdog window */
//#define WINDOW_UPPER        /* Delay too short, watchdog will reset */
//#define WINDOW_LOWER        /* Delay too long, watchdog will reset during delay */

static void APP_GPIO_Config(void);
static void APP_WWDG_Config(void);
static uint32_t APP_TimeoutCalculate(uint32_t timevalue);

int main(void)
{
  uint8_t delay = 0;
  // Set HSI 24MHz as system clock source
  BSP_RCC_HSI_24MConfig();
  // Initialize UART on PA2:TX PA3:RX
  BSP_USART_Config(115200);
  printf("PY32F0 WWDG Window Test\r\nClock: %ld\r\n", SystemCoreClock);
  // Set PB5 for LED output
  APP_GPIO_Config();

  if (LL_RCC_IsActiveFlag_WWDGRST())
  {
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
    LL_mDelay(1000);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
    LL_mDelay(500);

    LL_RCC_ClearResetFlags();
  }
  else
  {
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
  }

  APP_WWDG_Config();

#if defined(WINDOW_UPPER)
  delay = APP_TimeoutCalculate((0x7F - 0x50) - 5) + 1;
#elif defined(WINDOW_IN)
  delay = APP_TimeoutCalculate((0x7F - 0x50) + 1) + 1;
#else 
  delay = APP_TimeoutCalculate((0x7F - 0x3F) + 1) + 1;
#endif

  while (1)
  {
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
    printf("Delay... ");
    LL_mDelay(delay);
    printf("set counter\r\n");
    LL_WWDG_SetCounter(WWDG, 0x7F);
  }
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

static void APP_WWDG_Config(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);
  // Set the Watchdog counter value (7-bit), always write 1 in the MSB b6 to avoid immediate reset
  LL_WWDG_SetCounter(WWDG, 0x7F);
  LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_8);
  // Set window to 0x50
  LL_WWDG_SetWindow(WWDG, 0x50);
  LL_WWDG_Enable(WWDG);
}

/**
 * Twwdg(second) = t_pclk1 * 4096 * 2^wdgtb * (T[5:0] + 1)
 *               = 4096 * 2^wdgtb * (counter_diff) / f_pclk1
*/
static uint32_t APP_TimeoutCalculate(uint32_t countDiff)
{
  LL_RCC_ClocksTypeDef RCC_Clocks;
  uint32_t f_pclk1 = 0;
  uint32_t wdgtb = 0;
  // Get HCLK
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
  f_pclk1 = RCC_Clocks.PCLK1_Frequency;
  // 2^wdgtb
  wdgtb = (1 << ((LL_WWDG_PRESCALER_8) >> 7)); /* 2^WDGTB */
  // Calculate
  return ((4096 * wdgtb * countDiff) / (f_pclk1 / 1000));
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
