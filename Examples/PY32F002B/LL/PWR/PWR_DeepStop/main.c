/**
  ******************************************************************************
  * PY32F002B Deep Stop Mode
  * 
  * Connections
  *  VCC      ---> Microammeter --> 3.3V
  *  GND      ---> GND 
  *  PA6      ---> Key          --> GND
  *  
  * - Don't connect any other pins to avoid possible current leakage
  * - MCU enters deep stop mode after 3 seconds, current drops from 1.2 mA to 0.6 uA
  * - Press key (short PA6 to GND) will bring MCU back to run mode, current then rises to 1.2 mA
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"

static void APP_ExtiConfig(void);
static void APP_EnterDeepStop(void);


int main(void)
{
  /* Configure HSI as Systemclock source */
  BSP_RCC_HSI_24MConfig();

  /* Hold 3 seconds for flash download */
  LL_mDelay(3000);

  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Initial EXTI Event */
  APP_ExtiConfig();

  /* Enter DEEP STOP mode */
  APP_EnterDeepStop();

  while (1);
}

/**
  * @brief  Configure Exti
  * @param  None
  * @retval None
  */
static void APP_ExtiConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* Configure PA6 as the EXTI6 interrupt input */
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE6);

  /* Enable EXTI6 */
  EXTI_InitStruct.Line = LL_EXTI_LINE_6;
  EXTI_InitStruct.LineCommand = ENABLE;
  /* Event mode */
  EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
  /* Trigger Falling Mode */
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  /* Initialize the EXTI registers according to the specified parameters in EXTI_InitStruct  */
  LL_EXTI_Init(&EXTI_InitStruct);
}

/**
  * @brief  Enter deep stop mode
  * @param  None
  * @retval None
  */
static void APP_EnterDeepStop(void)
{
  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* STOP mode with deep low power regulator ON */
  LL_PWR_SetLprMode(LL_PWR_LPR_MODE_DLPR);

  /* SRAM retention voltage aligned with digital LDO output */
  LL_PWR_SetStopModeSramVoltCtrl(LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO);

  /* Enter DeepSleep mode */
  LL_LPM_EnableDeepSleep();

  /* Request Wait For Event */
   __SEV();
   __WFE();
   __WFE();

   LL_LPM_EnableSleep();
}

void APP_ErrorHandler(void)
{
  while (1);
}
