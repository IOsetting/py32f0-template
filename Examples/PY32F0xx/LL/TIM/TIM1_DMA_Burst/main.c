/***
 * Demo: TIM1 DMA Burst
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"

uint32_t TIM1DataBuff[] = {5,200,5,800,5,100,5,900,5,50,5,950};

static void APP_DMABurstConfig(void);
static void APP_PWMChannelConfig(void);
static void APP_SystemClockConfig(void);
static void APP_TIM1BaseConfig(void);

int main(void)
{
  APP_SystemClockConfig();

  BSP_USART_Config(115200);
  printf("TIM1 DMA Burst Demo\r\nClock: %ld \r\n", SystemCoreClock);

  APP_TIM1BaseConfig();
  APP_PWMChannelConfig();
  APP_DMABurstConfig();

  while (1);
}

static void APP_PWMChannelConfig(void)
{
  LL_GPIO_InitTypeDef TIM1CH1MapInit = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  TIM1CH1MapInit.Pin        = LL_GPIO_PIN_8;
  TIM1CH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CH1MapInit.Alternate  = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &TIM1CH1MapInit);

  TIM_OC_Initstruct.OCMode        = LL_TIM_OCMODE_PWM1;
  TIM_OC_Initstruct.OCState       = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCPolarity    = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCIdleState   = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_Initstruct.CompareValue  = 100;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_Initstruct);
}

static void APP_DMABurstConfig(void)
{
  LL_DMA_InitTypeDef DMA_TIM1DMABurst ={0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  DMA_TIM1DMABurst.PeriphOrM2MSrcAddress  = (uint32_t)&(TIM1->DMAR);
  DMA_TIM1DMABurst.MemoryOrM2MDstAddress  = (uint32_t)TIM1DataBuff;
  DMA_TIM1DMABurst.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_TIM1DMABurst.Mode                   = LL_DMA_MODE_NORMAL;
  DMA_TIM1DMABurst.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
  DMA_TIM1DMABurst.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
  DMA_TIM1DMABurst.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_TIM1DMABurst.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_TIM1DMABurst.Priority               = LL_DMA_PRIORITY_MEDIUM;
  DMA_TIM1DMABurst.NbData                 = 12;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_TIM1DMABurst);

  LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_TIM1_UP);

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
}

static void APP_TIM1BaseConfig(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  TIM1CountInit.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler = 4800 - 1;
  TIM1CountInit.Autoreload = 1000 - 1;
  TIM1CountInit.RepetitionCounter = 1;
  LL_TIM_Init(TIM1, &TIM1CountInit);

  LL_TIM_EnableDMAReq_UPDATE(TIM1);
  /* DMA burst mode, each time 2 registers starting from RCR */
  LL_TIM_ConfigDMABurst(TIM1, LL_TIM_DMABURST_BASEADDR_RCR, LL_TIM_DMABURST_LENGTH_2TRANSFERS);
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
