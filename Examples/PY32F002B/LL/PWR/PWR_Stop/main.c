/**
  ******************************************************************************
  * PY32F002B Deep Sleep / Stop Mode
  * 
  * Connections
  *  VCC      ---> Microammeter --> 3.3V
  *  GND      ---> GND 
  *  
  * - Don't connect any other pins to avoid possible current leakage
  * - MCU enters stop mode after 3 seconds, current drops from 1.2 mA to 1.6 uA
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"

LL_LPTIM_InitTypeDef LPTIM_InitStruct = {0};

static void APP_EnterStopMode(void);

int main(void)
{
  /* Configure HSI as Systemclock source */
  BSP_RCC_HSI_24MConfig();

  /* Hold 3 seconds for flash download */
  LL_mDelay(3000);

  /* Set wake-up mode of the LPTIM(EXTI Line29) to event request */
  LL_EXTI_DisableIT(LL_EXTI_LINE_29);   /* Disable interrupt request for EXTI Line29 */
  LL_EXTI_EnableEvent(LL_EXTI_LINE_29); /* Enable event request for EXTI Line29 */

  /* Enabel LSI */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1);

  /* Select LSI as LTPIM clock source */
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);
  /* Enable LPTIM clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  /* Initialize LPTIM */
  LPTIM_InitStruct.Prescaler = LL_LPTIM_PRESCALER_DIV128;        /* prescaler: 128 */
  LPTIM_InitStruct.UpdateMode = LL_LPTIM_UPDATE_MODE_IMMEDIATE;  /* registers are updated after each APB bus write access */
  if (LL_LPTIM_Init(LPTIM, &LPTIM_InitStruct) != SUCCESS)
  {
    APP_ErrorHandler();
  }

  /* Enable LPTIM1 interrupt */
  NVIC_SetPriority(LPTIM1_IRQn, 0);
  NVIC_EnableIRQ(LPTIM1_IRQn);

  /* Enable LPTIM autoreload match interrupt  */
  LL_LPTIM_EnableIT_ARRM(LPTIM);

  /* Enable LPTIM */
  LL_LPTIM_Enable(LPTIM);

  /* Set autoreload value */
  LL_LPTIM_SetAutoReload(LPTIM, 1280 - 1);

  /* LPTIM starts in continuous mode */
  LL_LPTIM_StartCounter(LPTIM, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

  while (1)
  {
    /* Enable STOP mode */
    APP_EnterStopMode();
  }
}

/**
  * @brief  Enable Stop mode
  * @param  None
  * @retval None
  */
static void APP_EnterStopMode(void)
{
  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* STOP mode with low power regulator ON */
  LL_PWR_SetLprMode(LL_PWR_LPR_MODE_LPR);

  /* SRAM retention voltage aligned with digital LDO output */
  LL_PWR_SetStopModeSramVoltCtrl(LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO);

  /* Enter DeepSleep mode */
  LL_LPM_EnableDeepSleep();

  /* Request Wait For event */
   __SEV();
   __WFE();
   __WFE();

   LL_LPM_EnableSleep();
}

/**
  * @brief  LPTIM interrupt callback program
  * @param  None
  * @retval None
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
