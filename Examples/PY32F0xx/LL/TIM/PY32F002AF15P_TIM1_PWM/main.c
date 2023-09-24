/***
 * Demo: TIM1 PWM output
 * 
 * Board: PY32F002AF15P
 */
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

static void APP_TIM1Config(void);
static void APP_PWMChannelConfig(void);


int main(void)
{
  BSP_RCC_HSI_24MConfig();
  BSP_USART_Config(115200);
  printf("PY32F002AF15P TIM1 PWM Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_TIM1Config();
  APP_PWMChannelConfig();
  while (1);
}

static void APP_PWMChannelConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitTypeDef;
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct;

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  /* PA0:AF13->TIM1_CH3, PA1:AF13->TIM1_CH4 */
  GPIO_InitTypeDef.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
  GPIO_InitTypeDef.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitTypeDef.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitTypeDef);

  /* PB3:AF1->TIM1_CH2 */
  GPIO_InitTypeDef.Pin = LL_GPIO_PIN_3;
  GPIO_InitTypeDef.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitTypeDef);

  TIM_OC_Initstruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_Initstruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;

  /* Set channel compare values */
  TIM_OC_Initstruct.CompareValue = 250;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_Initstruct);
  TIM_OC_Initstruct.CompareValue = 500;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_Initstruct);
  TIM_OC_Initstruct.CompareValue = 750;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_Initstruct);
}

static void APP_TIM1Config(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
 
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler           = 2400-1;
  /* PWM period = 1000 */
  TIM1CountInit.Autoreload          = 1000-1;
  TIM1CountInit.RepetitionCounter   = 0;
  LL_TIM_Init(TIM1,&TIM1CountInit);

  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);
}

void APP_ErrorHandler(void)
{
  while (1);
}
