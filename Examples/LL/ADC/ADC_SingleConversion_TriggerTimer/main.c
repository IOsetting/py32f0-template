/***
 * Demo: ADC Triggered by TIM1
 * 
 * PY32          
 * PA4      ------> Input voltage between 0V ~ 3.3V
 * 
 * PA2(TX)  ------> RX
 * PA3(RX)  ------> TX
 */
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

#define VDDA_APPLI                       ((uint32_t)3300)

static void APP_ADCConfig(void);
static void APP_TimerInit(void);

int main(void)
{
  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("ADC Timer Trigger Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_ADCConfig();
  // Start ADC regular conversion and wait for next external trigger
  LL_ADC_REG_StartConversion(ADC1);

  APP_TimerInit();

  while (1);
}

static void APP_TimerInit(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  LL_TIM_SetPrescaler(TIM1, (SystemCoreClock / 6000) - 1);
  LL_TIM_SetAutoReload(TIM1, 6000 - 1);
  /* Triggered by update */
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
  LL_TIM_EnableCounter(TIM1);
}

static void APP_ADCConfig(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_ADC_Reset(ADC1);
  // Calibrate start
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
    /* Delay 1ms(>= 4 ADC clocks) before re-enable ADC */
    LL_mDelay(1);
  }
  // Calibrate end

  /* PA4 as analog input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
  /* Set ADC channel and clock source when ADEN=0, set other configurations when ADSTART=0 */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_41CYCLES_5);

  /* Set TIM1 as trigger source */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
  /* Turn off DMA transfer */
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4);

  LL_ADC_Enable(ADC1);
  LL_mDelay(1);

  /* Enable EOC(end of conversion) interrupt */
  LL_ADC_EnableIT_EOC(ADC1);
  NVIC_SetPriority(ADC_COMP_IRQn, 0);
  NVIC_EnableIRQ(ADC_COMP_IRQn);
}

void APP_ADC_EOC_Callback(void)
{
  uint16_t mVoltage;
  uint32_t adcRead;
  /* Read ADC conversion result */
  adcRead = LL_ADC_REG_ReadConversionData12(ADC1);
  /* Convert the adc value to voltage value */
  mVoltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adcRead, LL_ADC_RESOLUTION_12B);
  printf("Channel4 voltage %d mV\r\n", mVoltage);
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
