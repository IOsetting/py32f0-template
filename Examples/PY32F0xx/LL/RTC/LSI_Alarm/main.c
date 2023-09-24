/**
 * Demo of RTC and Alarm
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7FFF)

static void APP_RTC_Init(void);
static void APP_RTC_AlarmConfig(void);
static void APP_RTC_InterruptConfig(void);

int main(void)
{
  // Set system clock to 48MHz
  BSP_RCC_HSI_PLL48MConfig();
  // Initialize UART on PA2:TX PA3:RX
  BSP_USART_Config(115200);
  printf("PY32F0 RTC Demo\r\nClock: %ld\r\n", SystemCoreClock);

  APP_RTC_Init();
  APP_RTC_AlarmConfig();
  APP_RTC_InterruptConfig();

  while (1);
}

static void APP_RTC_Init(void)
{
  LL_RTC_InitTypeDef rtc_initstruct;

  // Enable RTC peripheral clock
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  LL_PWR_EnableBkUpAccess();

  /* 
   * Enable LSI and set it as RTC clock source
   * - LSI is not as accurate as HSI, clock error is +/-3% when TA=25°C,VCC=3.3V
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
}

static void APP_RTC_AlarmConfig(void)
{
  LL_RTC_TimeTypeDef  rtc_time_initstruct;
  LL_RTC_AlarmTypeDef rtc_alarm_initstruct;

  // Time = 11:59:35
  rtc_time_initstruct.Hours      =11;
  rtc_time_initstruct.Minutes    =59;
  rtc_time_initstruct.Seconds    =35;
  if (LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &rtc_time_initstruct) != SUCCESS)
  {
    printf("error\r\n");
  }

  // Alarm = 12:00:25
  rtc_alarm_initstruct.AlarmTime.Hours      = 12;
  rtc_alarm_initstruct.AlarmTime.Minutes    = 00;
  rtc_alarm_initstruct.AlarmTime.Seconds    = 15;
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
  LL_RTC_ClearFlag_ALR(RTC);
  // Enable second interrupt
  LL_RTC_EnableIT_SEC(RTC);
  // Enable overflow interrupt (when 32bit counter is overflow)
  LL_RTC_EnableIT_OW(RTC);
  // Enable Alarm interrupt
  LL_RTC_EnableIT_ALR(RTC);
  LL_RTC_EnableWriteProtection(RTC);

  NVIC_SetPriority(RTC_IRQn, 0x00);
  NVIC_EnableIRQ(RTC_IRQn);
}

void RTC_IRQHandler(void)
{
  uint32_t ts = 0;

  if (LL_RTC_IsActiveFlag_SEC(RTC))
  {
    ts = LL_RTC_TIME_Get(RTC);
    printf("%.2ld:%.2ld:%.2ld\r\n", ts/3600, (ts % 3600) / 60, (ts % 3600) % 60);
    LL_RTC_ClearFlag_SEC(RTC);
  }
  if (LL_RTC_IsActiveFlag_OW(RTC))
  {
    printf("overflow\r\n");
    LL_RTC_ClearFlag_OW(RTC);
  }
  if (LL_RTC_IsActiveFlag_ALR(RTC))
  {
    printf("alarm triggered\r\n");
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
