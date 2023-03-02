#include "main.h"
#include "py32f0xx_bsp_led.h"
#include "py32f0xx_bsp_printf.h"
#include "SEGGER_RTT.h"


static void APP_SystemClockConfig(void);

int main(void)
{
  APP_SystemClockConfig();

  BSP_USART_Config(115200);

  SEGGER_RTT_printf(0, "SystemCoreClock: %ld\r\n", SystemCoreClock);
  printf("SystemCoreClock: %ld \r\n", SystemCoreClock);

  while (1)
  {
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
    printf("echo\r\n");
    SEGGER_RTT_WriteString(0, "echo\r\n");
    LL_mDelay(1000);
  }
}

static void APP_SystemClockConfig(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSE_Enable();
  LL_RCC_HSE_SetFreqRegion(LL_RCC_HSE_16_32MHz);
  while(LL_RCC_HSE_IsReady() != 1);

  UTILS_ClkInitStruct.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
  UTILS_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_1;
  LL_PLL_ConfigSystemClock_HSE(24000000U, LL_UTILS_HSEBYPASS_OFF, &UTILS_ClkInitStruct);

  /* Re-init frequency of SysTick source, reload = freq/ticks = 48000000/1000 = 48000 */
  LL_InitTick(48000000, 1000U);
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
