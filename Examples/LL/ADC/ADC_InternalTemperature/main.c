/**
 * Demo of internal temperature sensor
 * 
 * - If the internal sensor reading is always 4095(0xFFF), disconnect the UART RX(A3) pin
 * 
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

#define VDDA_APPLI                       ((uint32_t)3300)

uint16_t convert_result = 0, temperature = 0;

static void APP_ADCConfig(void);


int main(void)
{
  // Set system clock to 48MHz
  BSP_RCC_HSI_PLL48MConfig();
  // Initialize UART on PA2:TX PA3:RX
  BSP_USART_Config(115200);
  printf("PY32F0 Internal Temperature Sensor Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_ADCConfig();
  // Start ADC regular conversion
  LL_ADC_REG_StartConversion(ADC1);

  while (1)
  {
    if(LL_ADC_IsActiveFlag_EOS(ADC1))
    {
      LL_ADC_ClearFlag_EOS(ADC1);
      // Read regular conversion data, 12-bit
      convert_result = LL_ADC_REG_ReadConversionData12(ADC1);
      // Convert it to temperature
      temperature = __LL_ADC_CALC_TEMPERATURE(VDDA_APPLI, convert_result, LL_ADC_RESOLUTION_12B);
      printf("Temperature:%d  DR:%d \r\n", temperature, convert_result);
      LL_mDelay(1000);
    }
  }
}

static void APP_ADCConfig(void)
{
  __IO uint32_t wait_loop_index = 0;

  LL_ADC_InitTypeDef ADC_Init;
  LL_ADC_REG_InitTypeDef LL_ADC_REG_InitType;

  LL_ADC_Reset(ADC1);

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  // Calibrate start
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
    /* Delay 1ms(>= 4 ADC clocks) before re-enable ADC */
    LL_mDelay(1);
  }
  // Calibrate end

  ADC_Init.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV64;
  ADC_Init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_Init.LowPowerMode = LL_ADC_LP_MODE_NONE;
  ADC_Init.Resolution = LL_ADC_RESOLUTION_12B;
  LL_ADC_Init(ADC1, &ADC_Init);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

  // Regular ADC config
  LL_ADC_REG_InitType.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  LL_ADC_REG_InitType.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_InitType.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_InitType.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  LL_ADC_REG_InitType.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  LL_ADC_REG_Init(ADC1, &LL_ADC_REG_InitType);
  // Set common path and internal channel
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
  // Delay to ensure temperature sensor becomes stable
  wait_loop_index = ((LL_ADC_DELAY_TEMPSENSOR_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  // Select temperature sensor
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
  // Enable ADC
  LL_ADC_Enable(ADC1);
  LL_mDelay(1);
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
