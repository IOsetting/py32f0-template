/**
  ******************************************************************************
  * PY32F002B PWM Demo
  * 
  * PA0     -->      LED1
  * PA1     -->      LED2
  * PB5     -->      LED3
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"


uint8_t p1 = 0, p2 = 0x5F, p3 = 0xAF;

static void APP_TIM1Config(void);
static void APP_PWMChannelConfig(void);


int main(void)
{
  BSP_RCC_HSI_48MConfig();

  BSP_USART_Config(115200);
  printf("PY32F002B PWM Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_TIM1Config();
  APP_PWMChannelConfig();

  while (1)
  {
    LL_TIM_OC_SetCompareCH1(TIM1, p1++);
    LL_TIM_OC_SetCompareCH2(TIM1, p2++);
    LL_TIM_OC_SetCompareCH3(TIM1, p3++);

    LL_mDelay(20);
  }
}

static void APP_PWMChannelConfig(void)
{
  LL_GPIO_InitTypeDef GpioInit = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA | LL_IOP_GRP1_PERIPH_GPIOB);

  /* PA0/PA1 -> TIM1_CH1/TIM1_CH2 */
  GpioInit.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
  GpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
  GpioInit.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GpioInit);
  /* PB5 -> TIM1_CH3 */
  GpioInit.Pin = LL_GPIO_PIN_5;
  GpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
  GpioInit.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GpioInit);

  TIM_OC_Initstruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_Initstruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;

  /* Set channel compare values */
  TIM_OC_Initstruct.CompareValue = p1;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_Initstruct);
  TIM_OC_Initstruct.CompareValue = p2;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_Initstruct);
  TIM_OC_Initstruct.CompareValue = p3;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_Initstruct);
}

static void APP_TIM1Config(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
 
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler           = 480-1;
  /* PWM period = 1000 */
  TIM1CountInit.Autoreload          = 256-1;
  TIM1CountInit.RepetitionCounter   = 0;
  LL_TIM_Init(TIM1,&TIM1CountInit);

  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);
}

void APP_ErrorHandler(void)
{
  while (1);
}
