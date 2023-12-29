/**
  ******************************************************************************
  * PY32F002B Set SWD port to GPIO
  * 
  * PA2     -->      LED   -->   GND
  * PB6     -->      LED   -->   GND
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"


static void APP_GpioConfig(void);


int main(void)
{
  BSP_RCC_HSI_48MConfig();

  BSP_USART_Config(115200);
  printf("PY32F002B SWD GPIO Demo\r\nClock: %ld\r\n", SystemCoreClock);
  // Wait 2 seconds
  LL_mDelay(2000);
  printf("Set PA2 and PB6 to GPIO\r\n");
  APP_GpioConfig();

  while (1)
  {
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_2);
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_6);
    LL_mDelay(1000);
  }
}

static void APP_GpioConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIOA GPIOB Clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  // PA2 GPIO
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PB6 GPIO
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void APP_ErrorHandler(void)
{
  while (1);
}
