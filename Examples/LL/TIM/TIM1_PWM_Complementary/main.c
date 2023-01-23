/***
 * Demo: TIM1 PWM Complementary
 * 
 * CH1:PA8  -> CH1N:PA7
 * CH2:PA9  -> CH2N:PB0
 * CH3:PA10 -> CH3N:PB1
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"

static void APP_PWMChannelConfig(void);
static void APP_SystemClockConfig(void);
static void APP_TIM1BaseConfig(void);

int main(void)
{
  APP_SystemClockConfig();

  BSP_USART_Config(115200);
  printf("TIM1 PWM Complementary Demo\r\nClock: %ld \r\n", SystemCoreClock);

  APP_TIM1BaseConfig();
  APP_PWMChannelConfig();

  while (1);
}

static void APP_PWMChannelConfig(void)
{
  LL_GPIO_InitTypeDef TIM1CH1MapInit= {0};
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct ={0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA|LL_IOP_GRP1_PERIPH_GPIOB);

  /* PA7/PA8/PA9/PA10 -> TIM1_CH1N/TIM1_CH1/TIM1_CH2/TIM1_CH3 */
  TIM1CH1MapInit.Pin        = LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  TIM1CH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CH1MapInit.Alternate  = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &TIM1CH1MapInit);

  /* PB0/PB1 -> TIM1_CH2N/TIM1_CH3N */
  TIM1CH1MapInit.Pin        = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  TIM1CH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CH1MapInit.Alternate  = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &TIM1CH1MapInit);

  TIM_OC_Initstruct.OCMode        = LL_TIM_OCMODE_PWM1;
  TIM_OC_Initstruct.OCState       = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCPolarity    = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCIdleState   = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_Initstruct.OCNState      = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCNPolarity   = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCNIdleState  = LL_TIM_OCIDLESTATE_LOW;

  TIM_OC_Initstruct.CompareValue  = 50;
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH1,&TIM_OC_Initstruct);

  TIM_OC_Initstruct.CompareValue  = 500;
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH2,&TIM_OC_Initstruct);

  TIM_OC_Initstruct.CompareValue  = 950;
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH3,&TIM_OC_Initstruct);
}

static void APP_TIM1BaseConfig(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
 
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler           = 2400-1;
  TIM1CountInit.Autoreload          = 1000-1;
  TIM1CountInit.RepetitionCounter   = 0;
  LL_TIM_Init(TIM1,&TIM1CountInit);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);
}

static void APP_SystemClockConfig(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSI_Enable();
  /* Change this value to adjust clock frequency, larger is faster */
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz + 15);
  while (LL_RCC_HSI_IsReady() != 1);

  UTILS_ClkInitStruct.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
  UTILS_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_1;
  LL_PLL_ConfigSystemClock_HSI(&UTILS_ClkInitStruct);

  /* Re-init frequency of SysTick source, reload = freq/ticks = 48000000/1000 = 48000 */
  LL_InitTick(48000000, 1000U);
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
