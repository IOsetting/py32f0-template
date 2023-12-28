/**
 * Demo: ADC Triggered by TIM1
 * 
 * PY32          
 * PA7      ------> Input voltage between 0V ~ 3.3V
 * 
 * PA3(TX)  ------> USB2TTL RX
 * PA4(RX)  ------> USB2TTL TX
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"


#define VDDA_APPLI                       ((uint32_t)3300)

static void APP_ADCConfig(void);
static void APP_TimerInit(void);
static void APP_AdcCalibrate(void);

int main(void)
{
  BSP_RCC_HSI_24MConfig();

  BSP_USART_Config(115200);
  printf("PY32F002B ADC Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_ADCConfig();

  APP_AdcCalibrate();

  /* Enable ADC */
  LL_ADC_Enable(ADC1);
  /* The delay between ADC enablement and ADC stabilization is at least 8 ADC clocks */
  LL_mDelay(1);

  // Start ADC regular conversion and wait for next external trigger
  LL_ADC_REG_StartConversion(ADC1);

  APP_TimerInit();

  while (1);
}

static void APP_TimerInit(void)
{
  /* Enable TIM1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  /* Set TIM1 prescale */
  LL_TIM_SetPrescaler(TIM1,6000);

  /* Set TIM1 auto-reload value */
  LL_TIM_SetAutoReload(TIM1, 4000);

  /* TIM1 Update event is used as trigger output */
  LL_TIM_SetTriggerOutput(TIM1,LL_TIM_TRGO_UPDATE);

  /* Enable TIM1 */
  LL_TIM_EnableCounter(TIM1);
}

static void APP_ADCConfig(void)
{
  /* Enable ADC1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /* Configure PA7 pin in analog input mode */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);

  /* Set ADC clock to pclk/8 */
  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV8);
  /* Set ADC resolution to 12 bit */
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  /* ADC conversion data alignment: right aligned */
  LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
  /* No ADC low power mode activated */
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
  /* Sampling time 239.5 ADC clock cycles */
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

  /* ADC regular group conversion trigger from external IP: TIM1 TRGO. */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  /* Set Trigger edge to rising edge */
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  /* Set ADC conversion mode to single mode: one conversion per trigger */
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
  /* ADC regular group behavior in case of overrun: data overwritten */
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  /* Disable ADC regular group sequencer discontinuous mode  */
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
  /*
   * PA3:ADC_IN1, PA4:ADC_IN2, PA6:ADC_IN3, PA7:ADC_IN4, 
   * PC0:ADC_IN5, PB6:ADC_IN6, PB0:ADC_IN7
  */
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4);
  /* Dose not enable internal conversion channel */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE );

  /* Enable EOC IT */
  LL_ADC_EnableIT_EOC(ADC1);

  NVIC_SetPriority(ADC_COMP_IRQn, 0);
  NVIC_EnableIRQ(ADC_COMP_IRQn);
}

static void APP_AdcCalibrate(void)
{
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0;
#endif

  if (LL_ADC_IsEnabled(ADC1) == 0)
  {

    /* Enable ADC calibration */
    LL_ADC_StartCalibration(ADC1);

#if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
#if (USE_TIMEOUT == 1)
      /* Detects if the calibration has timed out */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {

        }
      }
#endif
    }

    /* The delay between the end of ADC calibration and ADC enablement is at least 4 ADC clocks */
    LL_mDelay(1);
  }
}

void ADC_COMP_IRQHandler(void)
{
  uint16_t mVoltage;
  uint32_t adcRead;

  if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
  {
    LL_ADC_ClearFlag_EOC(ADC1);

    /* Read ADC conversion result */
    adcRead = LL_ADC_REG_ReadConversionData12(ADC1);
    /* Convert the adc value to voltage value */
    mVoltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adcRead, LL_ADC_RESOLUTION_12B);
    printf("Read %ld, voltage %d mV\r\n", adcRead, mVoltage);
  }
}


void APP_ErrorHandler(void)
{
  while (1);
}
