/**
 ******************************************************************************
 * 
 * Read 128-byte UID from UID_BASE(0x1FFF0E00)
 * 
 ******************************************************************************
 */

#include <inttypes.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"

static uint8_t device_uid[128];

static void APP_SystemClockConfig(void);
static void APP_ReadDeviceUID(uint32_t *pBuf);

int main(void)
{
  uint8_t i;
  APP_SystemClockConfig();

  BSP_USART_Config(115200);
  printf("PY32F0xx Read MCU UID\r\nClock: %ld\r\n", SystemCoreClock);
  APP_ReadDeviceUID((uint32_t *)device_uid);

  printf("UID in UINT32: ");
  for (i = 0; i < 32; i++)
  {
    printf("%08" PRIX32 " ", *((uint32_t *)device_uid + i));
  }
  printf("\r\n");

  printf("UID in  UINT8: ");

  for (i = 0; i < 128; i++)
  {
    printf("%02X", *(device_uid + i));
    if ((i + 1) % 4 == 0)
    {
      printf(" ");
    }
  }
  printf("\r\n");

  while (1);
}

static void APP_SystemClockConfig(void)
{
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

static void APP_ReadDeviceUID(uint32_t *pBuf)
{
  uint8_t i;
  for (i = 0; i < 32; i++)
  {
    *(pBuf + i) = *((uint32_t *)UID_BASE_ADDRESS + i); // or *((uint32_t *)(UID_BASE_ADDRESS + (4U * i)));
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
