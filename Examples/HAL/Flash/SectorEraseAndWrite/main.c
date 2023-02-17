/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_hal.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

void APP_ErrorHandler(void);

#define FLASH_USER_START_ADDR     0x0800F000

const uint32_t DATA[64] =
{
  0x01010101, 0x23456789, 0x3456789A, 0x456789AB, 0x56789ABC, 0x6789ABCD, 0x789ABCDE, 0x89ABCDEF,
  0x9ABCDEF0, 0xABCDEF01, 0xBCDEF012, 0xCDEF0123, 0xDEF01234, 0xEF012345, 0xF0123456, 0x01234567,
  0x01010101, 0x23456789, 0x3456789A, 0x456789AB, 0x56789ABC, 0x6789ABCD, 0x789ABCDE, 0x89ABCDEF,
  0x9ABCDEF0, 0xABCDEF01, 0xBCDEF012, 0xCDEF0123, 0xDEF01234, 0xEF012345, 0xF0123456, 0x01234567,
  0x23456789, 0xAAAAAAAA, 0x55555555, 0x23456789, 0xAAAAAAAA, 0x55555555, 0x23456789, 0xAAAAAAAA,
  0x23456789, 0xAAAAAAAA, 0x55555555, 0x23456789, 0xAAAAAAAA, 0x55555555, 0x23456789, 0xAAAAAAAA,
  0x23456789, 0xAAAAAAAA, 0x55555555, 0x23456789, 0xAAAAAAAA, 0x55555555, 0x23456789, 0xAAAAAAAA,
  0x23456789, 0xAAAAAAAA, 0x55555555, 0x23456789, 0xAAAAAAAA, 0x55555555, 0x23456789, 0xAAAAAAAA,
};

static void APP_FlashErase(void);
static void APP_FlashProgram(void);
static void APP_FlashEraseVerify(void);
static void APP_FlashWriteVerify(void);

int main(void)
{
  HAL_Init();

  BSP_HSI_PLL_48MHzClockConfig();

  BSP_USART_Config();

  printf("Flash Read&Write Demo\r\nClock: %ld\r\n", SystemCoreClock);

  // Unlock flash
  HAL_FLASH_Unlock();
  // Erase flash
  APP_FlashErase();
  APP_FlashEraseVerify();
  printf("Erase verif: succ\r\n");

  // Write to flash
  APP_FlashProgram();
  // Lock
  HAL_FLASH_Lock();
  // Write verification
  APP_FlashWriteVerify();
  printf("Write verif: succ\r\n");

  while (1);
}


static void APP_FlashErase(void)
{
  uint32_t SECTORError = 0;
  FLASH_EraseInitTypeDef EraseInitStruct;
  // Erase type = sector
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORERASE;
  // Erase address start
  EraseInitStruct.SectorAddress = FLASH_USER_START_ADDR;
  // Number of sectors
  EraseInitStruct.NbSectors  = 1;
  // Erase
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

static void APP_FlashProgram(void)
{
  uint32_t flash_program_start = FLASH_USER_START_ADDR ;
  uint32_t flash_program_end = (FLASH_USER_START_ADDR + sizeof(DATA));
  uint32_t *src = (uint32_t *)DATA;

  while (flash_program_start < flash_program_end)
  {
    // Write to flash
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, flash_program_start, src) == HAL_OK)
    {
      // Move flash point to next page
      flash_program_start += FLASH_PAGE_SIZE;
      // Move data point
      src += FLASH_PAGE_SIZE / 4;
    }
  }
}

static void APP_FlashEraseVerify(void)
{
  uint32_t addr = 0;

  while (addr < sizeof(DATA))
  {
    if (0xFFFFFFFF != HW32_REG(FLASH_USER_START_ADDR + addr))
    {
      printf("erase verification error\r\n");
    }
    addr += 4;
  }
}

static void APP_FlashWriteVerify(void)
{
  uint32_t addr = 0;

  while (addr < sizeof(DATA))
  {
    if (DATA[addr / 4] != HW32_REG(FLASH_USER_START_ADDR + addr))
    {
      printf("write verification error\r\n");
    }
    addr += 4;
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Export assert error source and line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
