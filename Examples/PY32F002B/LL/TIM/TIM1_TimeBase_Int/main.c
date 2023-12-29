/**
 ******************************************************************************
 * 
 * PY32F002B TIM1 Update Interrupt Demo
 * 
 ******************************************************************************
 */

#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"

static void APP_GPIOConfig(void);
static void APP_TIM1Config(void);

int main(void)
{
  BSP_RCC_HSI_48MConfig();
  BSP_USART_Config(115200);
  printf("PY32F002B TIM1 Update Interrupt Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIOConfig();
  APP_TIM1Config();

  while (1);
}

static void APP_TIM1Config(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(RCC_APBENR2_TIM1EN);
  
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler           = 8000-1;
  TIM1CountInit.Autoreload          = 6000-1;
  TIM1CountInit.RepetitionCounter   = 0;
  LL_TIM_Init(TIM1, &TIM1CountInit);

  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,0);
}

static void APP_GPIOConfig(void)
{
  /* Set PB5 as GPIO output */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) && LL_TIM_IsEnabledIT_UPDATE(TIM1))
  {
    LL_TIM_ClearFlag_UPDATE(TIM1);
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}
