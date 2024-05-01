/**
  ******************************************************************************
  * PY32F002B Deep Stop Wake Up By LPTIM(LSI)
  * 
  * Connections
  *  PB5      --->    - LED +   ---> 3.3V
  *  VCC      ---> 3.3V
  *  GND      ---> GND 
  *  
  * - MCU enters deep stop mode after 3 seconds,
  * - LED blinks every 5 seconds (actually it is a bit longer than 5 seconds)
  * - Current consumption is around 0.6uA in deep stop mode
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"

static void APP_GPIO_Config(void);
static void APP_LPTIM_Config(void);
static void APP_EnterDeepStop(void);

int main(void)
{
  /* Configure HSI as Systemclock source */
  BSP_RCC_HSI_24MConfig();
  /* Hold 3 seconds for flash download */
  LL_mDelay(3000);

  APP_GPIO_Config();

  APP_LPTIM_Config();

  /* Starts LPTIM counter in continuous mode */
  LL_LPTIM_StartCounter(LPTIM, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

  while (1)
  {
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
    LL_mDelay(20);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
    /* Enable DEEP STOP mode */
    APP_EnterDeepStop();
  }
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /* PB5 as output */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  /* Turn LED off */
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
}

static void APP_LPTIM_Config(void)
{
  /* Disable LES otherwise it will consume extra 0.6 uA */
  LL_RCC_LSE_Disable();

  /* Enable LSI */
  LL_RCC_LSI_SetCalibTrimming(LL_RCC_LSICALIBRATION_32768Hz);
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1);

  /* Set LSI as LTPIM clock source */
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  /* prescaler: 128 */
  LL_LPTIM_SetPrescaler(LPTIM, LL_LPTIM_PRESCALER_DIV128);
  LL_LPTIM_SetUpdateMode(LPTIM, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
  /* Enable LPTIM autoreload match interrupt  */
  LL_LPTIM_EnableIT_ARRM(LPTIM);
  /* Enable LPTIM */
  LL_LPTIM_Enable(LPTIM);
  /*
   * Set autoreload value
   * It must be modified only when LPTIM is enabled
   */
  LL_LPTIM_SetAutoReload(LPTIM, 1280 - 1);

  /* Enable LPTIM1 interrupt */
  NVIC_SetPriority(LPTIM1_IRQn, 0);
  NVIC_EnableIRQ(LPTIM1_IRQn);
}

static void APP_EnterDeepStop(void)
{
  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  /* DEEP STOP mode with deep low power regulator ON */
  LL_PWR_SetLprMode(LL_PWR_LPR_MODE_DLPR);
  /* SRAM retention voltage aligned with digital LDO output */
  LL_PWR_SetStopModeSramVoltCtrl(LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO);
  /* Enter STOP mode */
  LL_LPM_EnableDeepSleep();
  /* Wait For interrupt */
   __WFI();

   LL_LPM_EnableSleep();
}

/**
  * LPTIM interrupt callback
  */
void APP_LptimIRQCallback(void)
{
  if((LL_LPTIM_IsActiveFlag_ARRM(LPTIM) == 1) && (LL_LPTIM_IsEnabledIT_ARRM(LPTIM) == 1))
  {
    /* Clear autoreload match flag */
    LL_LPTIM_ClearFLAG_ARRM(LPTIM);
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}
