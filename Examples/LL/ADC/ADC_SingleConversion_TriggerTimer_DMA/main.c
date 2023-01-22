#include "main.h"
#include "py32f0xx_bsp_printf.h"

#define ADC_CALIBRATION_TIMEOUT_MS       ((uint32_t) 1)
#define VDDA_APPLI                       ((uint32_t)3300)
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

/* Private variables ---------------------------------------------------------*/
__IO uint32_t ubUserButtonPressed = 0;
__IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;
__IO uint16_t uhADCxConvertedData_Voltage_mVolt = 0;

static uint32_t ADCxConvertedDatas;


static void APP_SystemClockConfig(void);
static void APP_AdcCalibrate(void);
static void APP_AdcConfig(void);
static void APP_GPIOConfig(void);
static void APP_TimerInit(void);
static void APP_DmaConfig(void);

int main(void)
{
  APP_SystemClockConfig();
  APP_GPIOConfig();
  BSP_USART_Config(115200);
  printf("ADC Timer Trigger DMA Demo\r\nClock: %ld \r\n", SystemCoreClock);

  APP_DmaConfig();

  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  LL_ADC_Reset(ADC1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  APP_AdcCalibrate();
  APP_AdcConfig();
  LL_ADC_Enable(ADC1);
  /* Ensure ADC is stable, >= 8 ADC Clock */
  LL_mDelay(1);
  LL_ADC_REG_StartConversion(ADC1);

  APP_TimerInit();

  while (1)
  {
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
    LL_mDelay(1000);
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
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_41CYCLES_5);

  /* Set TIM1 as trigger source */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4);
}

static void APP_AdcCalibrate(void)
{
  __IO uint32_t backup_setting_adc_dma_transfer = 0;
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0;
#endif

  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Backup current settings */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    /* Turn off DMA when calibrating */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
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

    /* Delay before re-enable ADC */
    LL_mDelay(1);
    /* Apply saved settings */
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
  }
}

static void APP_TimerInit(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  /* Set period to 48000000 */
  LL_TIM_SetPrescaler(TIM1, 6000);
  LL_TIM_SetAutoReload(TIM1, 8000);
  /* Triggered by update */
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);

  LL_TIM_EnableCounter(TIM1);
}

static void APP_DmaConfig(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  /* ADC -> LL_DMA_CHANNEL_1 */
  SET_BIT(SYSCFG->CFGR3, 0x0);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_NOINCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR, (uint32_t)&ADCxConvertedDatas, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
  /* Enable transfer-complete interrupt */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void APP_TransferCompleteCallback(void)
{
  /* Convert the adc value to voltage value */
  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas, LL_ADC_RESOLUTION_12B);
  printf("%s%s%d mV\r\n","Channel4","Voltage:",uhADCxConvertedData_Voltage_mVolt);
}

static void APP_SystemClockConfig(void)
{
  LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct;

  LL_RCC_HSI_Enable();
  /* Change this value to adjust frequency */
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz + 15);
  while(LL_RCC_HSI_IsReady() != 1);

  UTILS_ClkInitStruct.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
  UTILS_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_1;
  LL_PLL_ConfigSystemClock_HSI(&UTILS_ClkInitStruct);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Re-init frequency of SysTick source, reload = freq/ticks = 48000000/1000 = 48000 */
  LL_InitTick(48000000, 1000U);
  /* Update global SystemCoreClock(or through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
}

static void APP_GPIOConfig(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
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
