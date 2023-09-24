/**
 * Demo Of Low Power TIM
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

static void APP_GPIO_Config(void);
static void APP_ConfigLPTIMOneShot(void);

int main(void)
{
  // Set HSI 8MHz as system clock source
  BSP_RCC_HSI_8MConfig();
  // Initialize UART on PA2:TX PA3:RX
  BSP_USART_Config(115200);
  printf("PY32F0 Low Power TIM Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIO_Config();

  APP_ConfigLPTIMOneShot();

  while (1);
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

static void APP_ConfigLPTIMOneShot(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
  // Prescaler = 64
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV64);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  LL_LPTIM_Enable(LPTIM1);

  // 8000000/64/62500 = 2Hz
  LL_LPTIM_SetAutoReload(LPTIM1, 62500);
  LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);

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
