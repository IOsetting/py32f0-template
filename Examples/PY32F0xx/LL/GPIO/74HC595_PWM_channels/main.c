/***
 * Demo: Generate PWM signal over 74HC595
 * 
 *    The PWM frequency is adjustable between 10Hz ~ 100Hz, which is sufficient 
 *    to drive a brushed DC motor. In this way, 3 GPIO pins can generate 16 PWM 
 *    channels by using 2 74HC595.
 * 
 * PY32          74HC595
 * PA0   ------> RCLK/STCP
 * PA1   ------> SRCLK/SHCP
 * PA4   ------> SER/DS
 *               MR/SRCLR        ---> VCC
 *               OE              ---> GND
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "74hc595.h"

/* PWM frequency, the count of PWM interval periods per second */
#define PWM_FREQUENCY 50
/* The resolution of each duty cycle. In this demo PWM_FREQUENCY * PWM_PERIOD should be a divisor of 6000 */
#define PWM_PERIOD    20
/* The count of 74hc595 */
#define HC595_SIZE    2
/* PWM channels, a multiple of 8 */
#define PWM_CH_SIZE   (HC595_SIZE*8)


uint8_t hc595_state[HC595_SIZE], pwm_duty[PWM_CH_SIZE], pwm_duty_pre[PWM_CH_SIZE], pwm_counter = 0;

static void APP_GPIOConfig(void);
static void APP_TIM1Config(void);

int main(void)
{
  uint8_t i;

  /* Set clock = 48MHz */
  BSP_RCC_HSI_PLL48MConfig();

  /* Enable peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  APP_GPIOConfig();

  APP_TIM1Config();

  /* Initialize the comparison value of each channel */
  for (i = 0; i < PWM_CH_SIZE; i++)
  {
    pwm_duty[i] = i;
    pwm_duty_pre[i] = i;
  }

  while (1)
  {
    LL_mDelay(200);
    /*
     * Change comparison value to update duty cycle  
     */
    for (i = 0; i < PWM_CH_SIZE; i++)
    {
      pwm_duty_pre[i] = (pwm_duty_pre[i] + 1) % PWM_PERIOD;
    }
  }
}

void APP_TIM1UpdateCallback(void)
{
  uint8_t i, hc595_idx, mask;

  for (i = 0; i < PWM_CH_SIZE; i++)
  {
    mask = 1 << i;
    hc595_idx = i / 8;

    if (pwm_duty[i] <= pwm_counter)
    {
      hc595_state[hc595_idx] = hc595_state[hc595_idx] & (~mask);
    }
    else
    {
      hc595_state[hc595_idx] = hc595_state[hc595_idx] | mask;
    }
  }
  /* Write to 74hc595 output pins */
  HC595_Write(hc595_state, HC595_SIZE);

  pwm_counter++;
  /* When period ends*/
  if (pwm_counter == PWM_PERIOD)
  {
    /* reset counter */
    pwm_counter = 0;
    /* reload comparison values */
    memcpy(pwm_duty, pwm_duty_pre, PWM_CH_SIZE);
  }
}

static void APP_GPIOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* PA0 RCLK/STCP */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA1 SRCLK/SHCP */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA4 SER/DS */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void APP_TIM1Config(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(RCC_APBENR2_TIM1EN);
  
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler           = 8000-1;
  TIM1CountInit.Autoreload          = (SystemCoreClock / 8000) / (PWM_FREQUENCY * PWM_PERIOD) -1;
  TIM1CountInit.RepetitionCounter   = 0;
  LL_TIM_Init(TIM1,&TIM1CountInit);

  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,0);
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
