/**
 * Independent Watchdog Demo
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

static void APP_GPIO_Config(void);
static void APP_IWDG_Config(void);

int main(void)
{
  uint8_t delay = 0;
  // Set HSI 24MHz as system clock source
  BSP_RCC_HSI_24MConfig();
  // Initialize UART on PA2:TX PA3:RX
  BSP_USART_Config(115200);
  printf("PY32F0 IWDG Demo\r\nClock: %ld\r\n", SystemCoreClock);
  // Set PB5 for LED output
  APP_GPIO_Config();

  APP_IWDG_Config();

  while (1)
  {
    /*
     * Watchdog will be triggered when delay exceeds 1 second
    */
    printf("Delay %d ... ", 900 + delay);
    LL_mDelay(900 + delay);
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
    printf("reload counter\r\n");
    LL_IWDG_ReloadCounter(IWDG);
    delay += 10;
  }
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

static void APP_IWDG_Config(void)
{
  // Enable LSI
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() == 0U) {;}
  // Enable IWDG
  LL_IWDG_Enable(IWDG);

  LL_IWDG_EnableWriteAccess(IWDG);
  // Set waiting period to around 1 ms
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32);
  // Set counter to 1000 -> around 1 seconds
  LL_IWDG_SetReloadCounter(IWDG, 1000);
  // Wait IWDG ready
  while (LL_IWDG_IsReady(IWDG) == 0U);
  // Reset counter
  LL_IWDG_ReloadCounter(IWDG);
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
