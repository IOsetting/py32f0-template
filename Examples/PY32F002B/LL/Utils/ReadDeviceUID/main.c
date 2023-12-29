/**
 ******************************************************************************
 * 
 * Read 128-bit UID from UID_BASE(0x1FFF0000)
 * 
 *  0 Lot Numer
 *  1 Lot Numer
 *  2 Lot Numer
 *  3 Lot Numer
 *  4 Wafer Number
 *  5 Lot Numer
 *  6 Lot Numer
 *  7 Lot Numer
 *  8 internal code
 *  9 Y coordinate low order
 * 10 X coordinate low order
 * 11 X,YCoordinate high address
 * 12 Fixed code 0x78
 * 13 internal code
 * 14 internal code
 * 15 internal code
 * 
 ******************************************************************************
 */

#include <inttypes.h>
#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"

static uint8_t device_uid[16];

static void APP_ReadDeviceUID(uint32_t *pBuf);

int main(void)
{
  uint8_t i;
  
  BSP_RCC_HSI_24MConfig();
  BSP_USART_Config(115200);
  printf("PY32F002B Read UID\r\nClock: %ld\r\n", SystemCoreClock);

  APP_ReadDeviceUID((uint32_t *)device_uid);

  printf("UID in UINT32: ");
  for (i = 0; i < 4; i++)
  {
    printf("%08" PRIX32 " ", *((uint32_t *)device_uid + i));
  }
  printf("\r\n");

  printf("UID in  UINT8: ");

  for (i = 0; i < 16; i++)
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

static void APP_ReadDeviceUID(uint32_t *pBuf)
{
  uint8_t i;
  for (i = 0; i < 4; i++)
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
