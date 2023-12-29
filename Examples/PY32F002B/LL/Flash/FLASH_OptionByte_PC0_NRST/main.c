/**
 ******************************************************************************
 * 
 * PY32F002B Option Byte Demo
 *   This demo will switch PC0 back to NRST
 * 
 ******************************************************************************
 */

#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"

static void APP_FlashSetOptionByte(void);

int main(void)
{
  BSP_RCC_HSI_24MConfig();
  BSP_USART_Config(115200);
  printf("PY32F002B OptionByte Example\r\nClock: %ld\r\n", SystemCoreClock);
  LL_mDelay(1000);

  /**
    * Option Byte - Flash User option
    * 
    * [ 0, 7]:Reserved
    * [    8]:BOR enable, 0:disabled, 1:enabled
    * [ 9,11]:BOR Level, 000:1.8V ~ 111:3.2V
    * [   12]:IWDG_SW, 0:hardware, 1:software
    * [13,14]:NRST_MODE,SWD_MODE, 0x:C0=NRST,B6=SWD, 10:CO=GPIO,B6=SWD, 11:CO=SWD,B6=GPIO
    * [   15]:IWDG_STOP, Set timer status when IWDG stop, 0:timer freeze, 1:run
  */

  // check if bit[14] == 1
  if (READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE) != 0)
  {
    printf("PC0/NRST is not NRST\r\n");
    /* Write option byte */
    APP_FlashSetOptionByte();
  }
  else
  {
    printf("PC0/NRST has been configurated as NRST\r\n");
  }

  while (1);
}

static void APP_FlashSetOptionByte(void)
{
  FLASH_OBProgramInitTypeDef OBInitCfg;

  LL_FLASH_Unlock();
  LL_FLASH_OB_Unlock();

  OBInitCfg.OptionType = OPTIONBYTE_USER;

  /* 
   * USERType is a combination of
      OB_USER_BOR_EN, OB_USER_BOR_LEV, OB_USER_IWDG_SW, OB_USER_SWD_NRST_MODE OB_USER_IWDG_STOP
  */
  OBInitCfg.USERType = OB_USER_BOR_EN | OB_USER_BOR_LEV | OB_USER_IWDG_SW | OB_USER_SWD_NRST_MODE | OB_USER_IWDG_STOP;
  /*
   * USERConfig is a combination of 
   *  FLASH_OB_USER_BOR_ENABLE: OB_BOR_DISABLE, OB_BOR_ENABLE
   *  FLASH_OB_USER_BOR_LEVEL:  OB_BOR_LEVEL_1p7_1p8, OB_BOR_LEVEL_1p9_2p0, OB_BOR_LEVEL_2p1_2p2, ... OB_BOR_LEVEL_3p1_3p2
   *  FLASH_OB_USER_IWDG_SW:    OB_IWDG_SW, OB_IWDG_HW 
   *  FLASH_OB_USER_IWDG_STOP:  OB_IWDG_STOP_FREEZE, OB_IWDG_STOP_ACTIVE
   *  FLASH_OB_USER_SWD_NRST:   OB_SWD_PB6_NRST_PC0, OB_SWD_PB6_GPIO_PC0, OB_SWD_PC0_GPIO_PB6
   * Default value
   *  OB_BOR_DISABLE
   *  OB_BOR_LEVEL_3p1_3p2
   *  OB_IWDG_SW
   *  OB_IWDG_STOP_ACTIVE
   *  OB_SWD_PB6_NRST_PC0
  */
  OBInitCfg.USERConfig = OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW | OB_IWDG_STOP_ACTIVE | OB_SWD_PB6_NRST_PC0;
  LL_FLASH_OBProgram(&OBInitCfg);

  LL_FLASH_Lock();
  LL_FLASH_OB_Lock();
  /* Reload option bytes */
  LL_FLASH_OB_Launch();
}

void APP_ErrorHandler(void)
{
  while (1);
}
