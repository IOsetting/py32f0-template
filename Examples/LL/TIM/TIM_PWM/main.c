/***
 * Demo: TIM1 PWM output
 * 
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"

uint32_t TIM1ReloadCounter[9] = {900-1,800-1,700-1,600-1,500-1,400-1,300-1,200-1,100-1};


static void APP_SystemClockConfig(void);
static void APP_TIM1Config(void);
static void APP_PWMChannelConfig(void);


int main(void)
{
  APP_SystemClockConfig();
  BSP_USART_Config(115200);
  printf("TIM1 PWM Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_TIM1Config();
  APP_PWMChannelConfig();
  while (1);
}

static void APP_PWMChannelConfig(void)
{
  LL_GPIO_InitTypeDef TIM1CH1MapInit = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /* PA8/PA9/PA10 -> TIM1_CH1N/TIM1_CH1/TIM1_CH2/TIM1_CH3 */
  TIM1CH1MapInit.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
  TIM1CH1MapInit.Mode = LL_GPIO_MODE_ALTERNATE;
  TIM1CH1MapInit.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &TIM1CH1MapInit);

  TIM_OC_Initstruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_Initstruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;

  /* Set channel compare values */
  TIM_OC_Initstruct.CompareValue = 250;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_Initstruct);
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


static void APP_SystemClockConfig(void)
{
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while (LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_Init1msTick(24000000);
  LL_SetSystemCoreClock(24000000);
}

void APP_TIM1UpdateCallback(void)
{
  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
}

void APP_ErrorHandler(void)
{
  while (1);
}
