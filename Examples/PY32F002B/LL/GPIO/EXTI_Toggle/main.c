/**
  ******************************************************************************
  * PY32F002B EXTI Toggle Demo
  * 
  * PA0     -->      KEY   -->   GND
  * PB5     -->      LED   -->   GND
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"


static void APP_GpioConfig(void);
static void APP_ConfigureEXTI(void);


int main(void)
{
  BSP_RCC_HSI_48MConfig();

  BSP_USART_Config(115200);
  printf("PY32F002B EXTI Toggle Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GpioConfig();
  APP_ConfigureEXTI();

  while (1);
}

static void APP_GpioConfig(void)
{
  /* Enable GPIOB Clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /* PB5 as output */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

static void APP_ConfigureEXTI(void)
{
  /* Enable GPIOA Clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /* PA0 as input */
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Triggerred by falling edge */
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.Line = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**
   * Enable interrupt
  */
  NVIC_SetPriority(EXTI0_1_IRQn, 1);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void EXTI0_1_IRQHandler(void)
{
  if(LL_EXTI_IsActiveFlag(LL_EXTI_LINE_0))
  {
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
    LL_EXTI_ClearFlag(LL_EXTI_LINE_0);
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}
