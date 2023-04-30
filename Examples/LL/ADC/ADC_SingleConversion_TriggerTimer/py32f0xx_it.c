#include "main.h"
#include "py32f0xx_it.h"

extern void APP_ADC_EOC_Callback(void);

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
}

/**
  * @brief This function handles ADC_COMP Interrupt .
  */
void ADC_COMP_IRQHandler(void)
{
  if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
  {
    LL_ADC_ClearFlag_EOC(ADC1);
    APP_ADC_EOC_Callback();
  }
}