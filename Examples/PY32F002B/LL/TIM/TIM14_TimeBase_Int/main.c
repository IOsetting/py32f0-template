/**
 ******************************************************************************
 * 
 * PY32F002B TIM14 Update Interrupt Demo
 * 
 ******************************************************************************
 */

#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"

static void APP_GPIO_Config(void);
static void APP_TIM_Config(void);

int main(void)
{
  BSP_RCC_HSI_48MConfig();
  BSP_USART_Config(115200);
  printf("PY32F002B TIM14 Update Interrupt Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIO_Config();
  APP_TIM_Config();

  while (1);
}

static void APP_TIM_Config(void)
{
  LL_TIM_InitTypeDef TIM_InitTypeDef = {0};

  LL_APB1_GRP2_EnableClock(RCC_APBENR2_TIM14EN);
  
  TIM_InitTypeDef.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitTypeDef.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM_InitTypeDef.Prescaler           = 8000-1;
  TIM_InitTypeDef.Autoreload          = 6000-1;
  TIM_InitTypeDef.RepetitionCounter   = 0;
  LL_TIM_Init(TIM14, &TIM_InitTypeDef);

  LL_TIM_EnableIT_UPDATE(TIM14);
  LL_TIM_EnableCounter(TIM14);

  NVIC_EnableIRQ(TIM14_IRQn);
  NVIC_SetPriority(TIM14_IRQn, 1);
}

static void APP_GPIO_Config(void)
{
  /* Set PB5 as GPIO output */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

void TIM14_IRQHandler(void)
{
  if(LL_TIM_IsActiveFlag_UPDATE(TIM14) && LL_TIM_IsEnabledIT_UPDATE(TIM14))
  {
    LL_TIM_ClearFlag_UPDATE(TIM14);
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}
