/***
 * Demo: TIM1 DMA
 * 
 */
#include "main.h"
#include "py32f0xx_bsp_printf.h"

uint32_t TIM1ReloadCounter[9] = {900-1,800-1,700-1,600-1,500-1,400-1,300-1,200-1,100-1};


static void APP_GPIOConfig(void);
static void APP_SystemClockConfig(void);
static void APP_TIM1Config(void);
static void APP_DMAConfig(void);


int main(void)
{
  APP_SystemClockConfig();
  APP_GPIOConfig();
  BSP_USART_Config(115200);
  printf("TIM1 DMA Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_TIM1Config();
  APP_DMAConfig();
  while (1);
}

static void APP_TIM1Config(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(RCC_APBENR2_TIM1EN);
  
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  /* Start with 1 second interval (at 8MHz) */
  TIM1CountInit.Prescaler           = 8000-1;
  TIM1CountInit.Autoreload          = 1000-1;
  TIM1CountInit.RepetitionCounter   = 0;
  LL_TIM_Init(TIM1,&TIM1CountInit);

  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableDMAReq_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,0);
}

static void APP_DMAConfig(void)
{
  LL_DMA_InitTypeDef DMA_TIM1Reload ={0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  DMA_TIM1Reload.PeriphOrM2MSrcAddress  = (uint32_t)&(TIM1->ARR);
  DMA_TIM1Reload.MemoryOrM2MDstAddress  = (uint32_t)TIM1ReloadCounter;
  DMA_TIM1Reload.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_TIM1Reload.Mode                   = LL_DMA_MODE_NORMAL;
  DMA_TIM1Reload.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
  DMA_TIM1Reload.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
  DMA_TIM1Reload.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_TIM1Reload.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_TIM1Reload.NbData                 = 9;
  DMA_TIM1Reload.Priority               = LL_DMA_PRIORITY_MEDIUM;
  LL_DMA_Init(DMA1,LL_DMA_CHANNEL_1,&DMA_TIM1Reload);

  /* Remap TIM1 update to DMA channel 1 */
  LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_TIM1_UP);

  LL_DMA_EnableIT_TC(DMA1,LL_DMA_CHANNEL_1);
  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);

  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  NVIC_SetPriority(DMA1_Channel1_IRQn,0);
}

static void APP_SystemClockConfig(void)
{
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

static void APP_GPIOConfig(void)
{
  /* Set PB5 as GPIO output */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

void APP_TIM1UpdateCallback(void)
{
  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
}

void APP_ErrorHandler(void)
{
  while (1);
}
