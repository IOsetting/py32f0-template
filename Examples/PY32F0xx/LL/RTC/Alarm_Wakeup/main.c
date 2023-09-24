/**
 * Demo of wakeup by RTC alarm
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

// 0x7FFF = 32,768 - 1
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7FFF)

static void APP_GPIO_Config(void);
static void APP_RTC_Init(void);
static void APP_RTC_AlarmConfig(void);
static void APP_RTC_InterruptConfig(void);
static void APP_EnterStopMode(void);

int main(void)
{
  // Set HSI 8MHz as system clock source
  BSP_RCC_HSI_8MConfig();
  // Initialize UART on PA2:TX PA3:RX
  BSP_USART_Config(115200);
  printf("PY32F0 RTC Wakeup Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_GPIO_Config();

  APP_RTC_Init();
  APP_RTC_InterruptConfig();

  while (1)
  {
    LL_RTC_WaitForSynchro(RTC);
    // Update alarm clock
    APP_RTC_AlarmConfig();

    printf("Entering stop mode... ");
    APP_EnterStopMode();
    printf("wakeup\r\n");
  }
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

static void APP_RTC_Init(void)
{
  LL_RTC_InitTypeDef rtc_initstruct;
  LL_RTC_TimeTypeDef  rtc_time_initstruct;

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  LL_PWR_EnableBkUpAccess();

  /* 
   * Enable LSI and set it as RTC clock source
  */
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() != 1);
  if (LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  }

  // Enable RTC clock and alarm
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTC);
  LL_RCC_EnableRTC();

  if (LL_RTC_DeInit(RTC) != SUCCESS)
  {
    printf("error\r\n");
  }
  rtc_initstruct.AsynchPrescaler = RTC_ASYNCH_PREDIV;
  rtc_initstruct.OutPutSource = LL_RTC_CALIB_OUTPUT_NONE;
  if (LL_RTC_Init(RTC, &rtc_initstruct) != SUCCESS)
  {
    printf("error\r\n");
  }

  // Time = 11:59:35
  rtc_time_initstruct.Hours      =11;
  rtc_time_initstruct.Minutes    =59;
  rtc_time_initstruct.Seconds    =35;
  if (LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &rtc_time_initstruct) != SUCCESS)
  {
    printf("error\r\n");
  }
}

static void APP_RTC_AlarmConfig(void)
{
  uint32_t ts;
  LL_RTC_AlarmTypeDef rtc_alarm_initstruct;

  ts = LL_RTC_TIME_Get(RTC) + 2;
  rtc_alarm_initstruct.AlarmTime.Hours      = ts/3600;
  rtc_alarm_initstruct.AlarmTime.Minutes    = (ts % 3600) / 60;
  rtc_alarm_initstruct.AlarmTime.Seconds    = (ts % 3600) % 60;
  if (LL_RTC_ALARM_Init(RTC, LL_RTC_FORMAT_BIN, &rtc_alarm_initstruct) != SUCCESS)   
  {
    printf("error\r\n");
  }

  if (LL_RTC_ExitInitMode(RTC) != SUCCESS)   
  {
    printf("error\r\n");
  }
}

static void APP_RTC_InterruptConfig(void)
{
  LL_RTC_DisableWriteProtection(RTC);
  // Clear alarm interrupt flag
  LL_RTC_ClearFlag_ALR(RTC);
  // Enable Alarm interrupt
  LL_RTC_EnableIT_ALR(RTC);
  LL_RTC_EnableWriteProtection(RTC);

  NVIC_SetPriority(RTC_IRQn, 0x00);
  NVIC_EnableIRQ(RTC_IRQn);

  LL_EXTI_EnableIT(LL_EXTI_LINE_19);
}

static void APP_EnterStopMode(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_LPM_EnableDeepSleep();
  __WFI();
}

void RTC_IRQHandler(void)
{
  if (LL_RTC_IsActiveFlag_ALR(RTC))
  {
    LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_5);
    LL_RTC_ClearFlag_ALR(RTC);
  }
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
