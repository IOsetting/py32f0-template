/***
 * Demo: 4-Channel ADC With DMA Triggered by TIM1
 * 
 * PY32          
 * PA4      ------> Input voltage between 0V ~ 3.3V
 * PA5      ------> Input voltage between 0V ~ 3.3V
 * PA6      ------> Input voltage between 0V ~ 3.3V
 * PA7      ------> Input voltage between 0V ~ 3.3V
 * 
 * PA2(TX)  ------> RX
 * PA3(RX)  ------> TX
 */
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

__IO uint16_t ADCxConvertedDatas[4];


static void APP_ADCConfig(void);
static void APP_TimerInit(void);
static void APP_DMAConfig(void);

int main(void)
{
  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("Timer Trigger 4-Channel ADC DMA Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_DMAConfig();
  APP_ADCConfig();
  // Start ADC regular conversion
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
  __IO uint32_t backup_setting_adc_dma_transfer = 0;

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_ADC_Reset(ADC1);
  // Calibrate start
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Backup current settings */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    /* Turn off DMA when calibrating */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_StartCalibration(ADC1);

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);

    /* Delay 1ms(>= 4 ADC clocks) before re-enable ADC */
    LL_mDelay(1);
    /* Apply saved settings */
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
  }
  // Calibrate end

  /* PA4,PA5,PA6,PA7 as ADC input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);
  
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
  /* Single conversion mode (CONT = 0, DISCEN = 0), performs a single sequence of conversions, converting all the channels once */
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  /* Enable: each conversions in the sequence need to be triggerred separately */
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
  /* Set channel 4/5/6/7 */
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4 | LL_ADC_CHANNEL_5 | LL_ADC_CHANNEL_6 | LL_ADC_CHANNEL_7);

  LL_ADC_Enable(ADC1);
}

static void APP_DMAConfig(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  // Remap ADC to LL_DMA_CHANNEL_1
  LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_ADC);
  // Transfer from peripheral to memory
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  // Set priority
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);
  // Circular mode
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  // Peripheral address no increment
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  // Memory address increment
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  // Peripheral data alignment : Word
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  // Memory data alignment : Word
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
  // Data length
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 4);
  // Sorce and target address
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR, (uint32_t)ADCxConvertedDatas, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
  // Enable DMA channel 1
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  // Enable transfer-complete interrupt
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void APP_TransferCompleteCallback(void)
{
  printf("Read value:");
  for (uint8_t i = 0; i < 4; i++)
  {
    printf(" %d", *(ADCxConvertedDatas + i));
  }
  printf("\r\n");
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
