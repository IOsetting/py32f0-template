/***
 * Demo: TIM1 PWM Complementary
 * 
 * Board: PY32F002AF15P
 * 
 * PB3:AF1-> TIM1_CH2, PB0:AF2-> TIM1_CH2N
 * PB6:AF1-> TIM1_CH3, PB1:AF2-> TIM1_CH3N
 */
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

static void APP_PWMChannelConfig(void);
static void APP_TIM1BaseConfig(void);

int main(void)
{
  uint16_t comp1 = 1000, comp2 = 0;
  BSP_RCC_HSI_24MConfig();

  BSP_USART_Config(115200);
  printf("TIM1 PWM Complementary Demo\r\nClock: %ld \r\n", SystemCoreClock);

  APP_TIM1BaseConfig();
  APP_PWMChannelConfig();

  while (1)
  {
    /* Change the duty cycle of CH2 and CH3 */
    LL_TIM_OC_SetCompareCH2(TIM1, comp1--);
    LL_TIM_OC_SetCompareCH3(TIM1, comp2++);
    if (comp1 == 0) comp1 = 1000;
    if (comp2 == 1000) comp2 = 0;
    LL_mDelay(1);
  }
}

static void APP_PWMChannelConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitTypeDef;
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct;

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**
   * PB3:AF1-> TIM1_CH2
   * PB6:AF1-> TIM1_CH3
  */
  GPIO_InitTypeDef.Pin        = LL_GPIO_PIN_3|LL_GPIO_PIN_6;
  GPIO_InitTypeDef.Mode       = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitTypeDef.Alternate  = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitTypeDef);
  /**
   * PB0:AF2-> TIM1_CH2N
   * PB1:AF2-> TIM1_CH3N
  */
  GPIO_InitTypeDef.Pin        = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitTypeDef.Alternate  = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitTypeDef);

  TIM_OC_Initstruct.OCMode        = LL_TIM_OCMODE_PWM1;
  TIM_OC_Initstruct.OCState       = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCPolarity    = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCIdleState   = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_Initstruct.OCNState      = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCNPolarity   = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCNIdleState  = LL_TIM_OCIDLESTATE_LOW;

  TIM_OC_Initstruct.CompareValue  = 500;
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH2,&TIM_OC_Initstruct);

  TIM_OC_Initstruct.CompareValue  = 950;
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH3,&TIM_OC_Initstruct);
}

static void APP_TIM1BaseConfig(void)
{
  LL_TIM_InitTypeDef TIM1CountInit;

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
 
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler           = 24-1;
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

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
