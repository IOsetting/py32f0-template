/**
 * Power Voltage Detector(PVD) Demo
 * 
 * - when input(PB7) voltage lower than 1.2V, PB5 output high
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

static void APP_GPIO_Config(void);
static void APP_ExtiConfig(void);
static void APP_PVD_Config(void);

int main(void)
{
  // Set HSI 24MHz as system clock source
  BSP_RCC_HSI_24MConfig();
  // Initialize UART on PA2:TX PA3:RX
  BSP_USART_Config(115200);
  printf("PY32F0 PVD Demo\r\nClock: %ld\r\n", SystemCoreClock);
  // Set PB5 for LED output
  APP_GPIO_Config();
  // Set PB7 as analog input
  APP_ExtiConfig();
  APP_PVD_Config();
  // Enable PVD
  LL_PWR_EnablePVD();

  while (1);
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

static void APP_ExtiConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  // PB7
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // Line 16 is for PVD output
  EXTI_InitStruct.Line = LL_EXTI_LINE_16;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);
}

static void APP_PVD_Config(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetPVDLevel(LL_PWR_PVDLEVEL_0);
  LL_PWR_DisablePVDFilter();
  LL_PWR_SetPVDFilter(LL_PWR_PVD_FILTER_1CLOCK);
  // PB7
  LL_PWR_SetPVDSource(LL_PWR_PVD_SOURCE_PB7);
  NVIC_SetPriority(PVD_IRQn, 1);
  NVIC_EnableIRQ(PVD_IRQn);
}

void PVD_IRQHandler(void)
{
  if (LL_PWR_IsActiveFlag_PVDO())
  {
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
  }
  else
  {
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
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
