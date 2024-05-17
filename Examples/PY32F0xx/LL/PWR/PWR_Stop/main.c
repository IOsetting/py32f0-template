/**
  ******************************************************************************
  * PY32F002A/003/030 Deep Sleep Mode
  * 
  * Connections
  *  VCC      ---> Microammeter --> 3.3V
  *  GND      ---> GND 
  *  PA0      ---> Key          --> GND
  *  
  * - Don't connect any other pins to avoid possible current leakage
  * - MCU enters stop mode after 3 seconds, current drops from 1.35 mA to 5.2 uA
  * - Press key (short PA0 to GND) will bring MCU back to run mode, current then rises to 1.47 mA
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

static void APP_ExtiConfig(void);


int main(void)
{
  /* Configure HSI as Systemclock source */
  BSP_RCC_HSI_24MConfig();

  /* Hold 3 seconds for flash download */
  LL_mDelay(3000);

  /* Disable LES to avoid it consume extra power */
  LL_RCC_LSE_Disable();

  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Initial EXTI Event */
  APP_ExtiConfig();
  /*
   * Set STOP voltage to 1.0V (default 1.2V)
   */
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  /*
   * Enable low power mode, SET_BIT(PWR->CR1, PWR_CR1_LPR)
   */
  LL_PWR_EnableLowPowerRunMode();
  /*
   * Enable deep stop, SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
   */
  LL_LPM_EnableDeepSleep();

  /*
   * Wait event 
   * Change the following to __WFI() if waiting for interrupt
   */
  __SEV();
  __WFE();
  __WFE();

  /*
   * Return to sleep mode
   * CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk))
   */
  LL_LPM_EnableSleep();

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

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* Configure PA0 as the EXTI6 interrupt input */
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE0);

  /* Enable EXTI0 */
  EXTI_InitStruct.Line = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  /* Event mode */
  EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
  /* Trigger Falling Mode */
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  /* Initialize the EXTI registers according to the specified parameters in EXTI_InitStruct  */
  LL_EXTI_Init(&EXTI_InitStruct);
}

void APP_ErrorHandler(void)
{
  while (1);
}
