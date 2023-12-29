#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "SEGGER_RTT.h"


int main(void)
{
  /* Configure HSI as Systemclock source */
  BSP_RCC_HSI_24MConfig();

  while (1)
  {
    SEGGER_RTT_printf(0, "SystemCoreClock: %ld\r\n", SystemCoreClock);
    LL_mDelay(1000);
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}
