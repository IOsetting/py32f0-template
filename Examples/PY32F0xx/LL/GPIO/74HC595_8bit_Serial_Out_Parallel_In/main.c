/***
 * Demo: 74HC595 8-Bit Serial-In-Parallel-Out
 * 
 * PY32          74HC595
 * PA0   ------> RCLK/STCP
 * PA1   ------> SRCLK/SHCP
 * PA4   ------> SER/DS
 *               MR/SRCLR        ---> VCC
 *               OE              ---> GND
 * 
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "74hc595.h"

static void APP_GPIOConfig(void);

int main(void)
{
  uint8_t d1 = 0xAA, d2 = 0x55;

  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();
  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  APP_GPIOConfig();

  while (1)
  {
    HC595_Write(&d1, 1);
    LL_mDelay(200);
    HC595_Write(&d2, 1);
    LL_mDelay(200);
    HC595_WriteByte(d1);
    LL_mDelay(200);
    HC595_WriteByte(d2);
    LL_mDelay(200);
  }
}

static void APP_GPIOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* PA0 RCLK/STCP */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA1 SRCLK/SHCP */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA4 SER/DS */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
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
