/***
 * Demo: 74HC165 8-Bit Parallel-In-Serial-Out
 * 
 * PY32          74HC165
 * PA0   ------> SH/LD
 * PA1   ------> CLK
 * PA4   ------> QH/Q7
 *               CE/Clock Inhibit ---> GND
 * 
 *               USB2TTL
 * PA2(TX) ----> RX
 * PA3(RX) ----> TX
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "74hc165.h"

static void APP_GPIOConfig(void);

int main(void)
{
  uint8_t data;

  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  APP_GPIOConfig();

  BSP_USART_Config(115200);
  printf("GPIO Demo: 74HC165 8-Bit Parallel-In-Serial-Out\r\nClock: %ld\r\n", SystemCoreClock);

  while (1)
  {
    data = HC165_Read();
    printf("%02X\r\n", data);
    LL_mDelay(500);
  }
}

static void APP_GPIOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* PA0 LD */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA1 CLK */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA4 DATA */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
