/***
 * Demo: Analog Watchdog
 * 
 * - PA4 is set as ADC input pin
 * - Limit the input voltage to between [0v, 1.5v] when start
 * - Increase the input voltage
 * - The LED on PB5 will start blinking when the voltage exceeds around 1.66v 
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"


#define ADC_CALIBRATION_TIMEOUT_MS       ((uint32_t) 1)

__IO uint8_t ubAnalogWatchdog1Status = 0;

static void APP_AdcConfig(void);
static void APP_AwdConfig(uint32_t AWDThresholdHighValue, uint32_t AWDThresholdLowValue);
static void APP_AdcCalibrate(void);
static void APP_GPIOConfig(void);
static void APP_SystemClockConfig(void);
static void APP_TimerInit(void);

int main(void)
{
  APP_SystemClockConfig();
  APP_GPIOConfig();
  BSP_USART_Config(115200);
  printf("ADC Watchdog Demo\r\nClock: %ld\r\n", SystemCoreClock);

  LL_ADC_Reset(ADC1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  APP_AdcCalibrate();
  APP_AdcConfig();
  /* Set threshold range to [0V, 1.66V] */
  APP_AwdConfig(0x0FFF / 2, 0);
  LL_ADC_Enable(ADC1);
  /* Delay 1ms(>= 8 ADC Clock) to ensure ADC is stable */
  LL_mDelay(1);
  LL_ADC_REG_StartConversion(ADC1);

  APP_TimerInit();
  while (1)
  {
    if (ubAnalogWatchdog1Status == 1)
    {
      LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
      LL_mDelay(500);
    }
  }
}

static void APP_AdcCalibrate(void)
{
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; 
#endif 

  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    LL_ADC_StartCalibration(ADC1);

#if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif 

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
#if (USE_TIMEOUT == 1)
      /* 检测校准是否超时 */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {

        }
      }
#endif 
    }

    /* Delay 1ms(>= 4 ADC clocks) before re-enable ADC */
    LL_mDelay(1);
  }
}

static void APP_AdcConfig(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /* PA4 as ADC input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
  /* Set ADC channel and clock source when ADEN=0, set other configurations when ADSTART=0 */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
  /* Low power mode off */
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_41CYCLES_5);
  /* Set TIM1 as trigger source */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
  /* DMA off */
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4);
}

static void APP_AwdConfig(uint32_t AWDThresholdHighValue, uint32_t AWDThresholdLowValue)
{
  /* ADC watchdog on channel4, use LL_ADC_AWD_ALL_CHANNELS_REG to monitor all channels */
  LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_CHANNEL_4_REG);
  /* Set threshold */
  LL_ADC_ConfigAnalogWDThresholds(ADC1, AWDThresholdHighValue, AWDThresholdLowValue);
  /* 
   * Turn on/off AWD interrupt when ADC conversion is stopped.
   * Note: LL_ADC_EnableIT_AWD / LL_ADC_DisableIT_AWD doesn't work when conversion is ongoing
   */
  LL_ADC_EnableIT_AWD(ADC1);

  NVIC_SetPriority(ADC_COMP_IRQn, 0);
  NVIC_EnableIRQ(ADC_COMP_IRQn);
}

void APP_AdcAnalogWatchdogCallback(void)
{
  ubAnalogWatchdog1Status = 1;
}

static void APP_TimerInit(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  /* Set period to 48000000 for 48MHz clock */
  LL_TIM_SetPrescaler(TIM1,8000);
  LL_TIM_SetAutoReload(TIM1, 6000);
  LL_TIM_SetTriggerOutput(TIM1,LL_TIM_TRGO_UPDATE);
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

static void APP_GPIOConfig(void)
{
  /* Set PB5 as GPIO output */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

void APP_ErrorHandler(void)
{
  while (1);
}
