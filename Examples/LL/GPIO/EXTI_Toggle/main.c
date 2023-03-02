/***
 * Demo: EXTI Interrupt
 */
#include "main.h"
#include "py32f0xx_bsp_led.h"
#include "py32f0xx_bsp_printf.h"

static void APP_SystemClockConfig(void);
static void APP_GpioConfig(void);
static void APP_ConfigureEXTI(void);


int main(void)
{
  APP_SystemClockConfig();

  APP_GpioConfig();
  APP_ConfigureEXTI();

  while (1);
}

static void APP_SystemClockConfig(void)
{
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
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

  /* PA12 as input */
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Triggerred by falling edge */
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.Line = LL_EXTI_LINE_12;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**
   * Enable interrupt
   * - EXTI0_1_IRQn for PA/PB/PC[0,1]
   * - EXTI2_3_IRQn for PA/PB/PC[2,3]
   * - EXTI4_15_IRQn for PA/PB/PC[4,15]
  */
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void EXTI4_15_IRQHandler(void)
{
  if(LL_EXTI_IsActiveFlag(LL_EXTI_LINE_12))
  {
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
    LL_EXTI_ClearFlag(LL_EXTI_LINE_12);
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
