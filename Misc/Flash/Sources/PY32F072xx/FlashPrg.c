/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 - 2019 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        2021-7-1
 * $Revision:    V1.0.0
 *
 * Project:      Flash Programming Functions for Puya PY32F030xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "FlashOS.h"        // FlashOS Structures
#include "py32f0xx.h"

typedef volatile unsigned char    vu8;
typedef          unsigned char     u8;
typedef volatile unsigned short   vu16;
typedef          unsigned short    u16;
typedef volatile unsigned long    vu32;
typedef          unsigned long     u32;

#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

//uint32_t gdw_HSI_FS;

//#define USE_HSI_24MHZ

////uiOffset = 0(4MHz), 1(8MHz), 2(16MHz), 3(22.12MHz), 4(24MHz)
//void SetFlashParameter(uint32_t uiOffset)
//{
//    FLASH->TS0 = ((M32(0x1FFF0F1C + uiOffset * 0x14) >> 0) & 0x000000FF);
//    FLASH->TS3 = ((M32(0x1FFF0F1C + uiOffset * 0x14) >> 8) & 0x000000FF);
//    FLASH->TS1 = ((M32(0x1FFF0F1C + uiOffset * 0x14) >> 16) & 0x000001FF);
//    FLASH->TS2P = ((M32(0x1FFF0F20 + uiOffset * 0x14) >> 0) & 0x000000FF);
//    FLASH->TPS3 = ((M32(0x1FFF0F20 + uiOffset * 0x14) >> 16) & 0x000007FF);
//    FLASH->PERTPE = ((M32(0x1FFF0F24 + uiOffset * 0x14) >> 0) & 0x0001FFFF);
//    FLASH->SMERTPE = ((M32(0x1FFF0F28 + uiOffset * 0x14) >> 0) & 0x0001FFFF);
//    FLASH->PRGTPE = ((M32(0x1FFF0F2C + uiOffset * 0x14) >> 0) & 0x0000FFFF);
//    FLASH->PRETPE = ((M32(0x1FFF0F2C + uiOffset * 0x14) >> 16) & 0x0000FFFF);
//}

//void InitRccAndFlashParam(void)
//{
//#ifdef USE_HSI_24MHZ
//    gdw_HSI_FS = READ_BIT(RCC->ICSCR, RCC_ICSCR_HSI_FS);
//    MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_FS, RCC_ICSCR_HSI_FS_2);//HSI_24MHz
//    MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_TRIM, M32(0x1FFF0F10)&RCC_ICSCR_HSI_TRIM);
//    while (RCC_CR_HSIRDY != READ_BIT(RCC->CR, RCC_CR_HSIRDY));
//    SetFlashParameter(4);
//#else
//    uint32_t dwOffset;
//    switch (READ_BIT(RCC->ICSCR, RCC_ICSCR_HSI_FS))
//    {
//    default://4MHz
//        dwOffset = 0x00;
//        break;
//    case RCC_ICSCR_HSI_FS_0://8MHz
//        dwOffset = 0x01;
//        break;
//    case RCC_ICSCR_HSI_FS_1://16MHz
//        dwOffset = 0x02;
//        break;
//    case (RCC_ICSCR_HSI_FS_1|RCC_ICSCR_HSI_FS_0)://22.12MHz
//        dwOffset = 0x03;
//        break;
//    case RCC_ICSCR_HSI_FS_2://24MHz
//        dwOffset = 0x04;
//        break;
//    }
//    MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_TRIM, M32(0x1FFF0F00 + dwOffset * 4)&RCC_ICSCR_HSI_TRIM);
//    WRITE_REG(FLASH->TS0, ((M32(0x1FFF0F1C + dwOffset * 0x14) >> 0) & 0x000000FF));
//    WRITE_REG(FLASH->TS1, ((M32(0x1FFF0F1C + dwOffset * 0x14) >> 8) & 0x000000FF));
//    WRITE_REG(FLASH->TS3, ((M32(0x1FFF0F1C + dwOffset * 0x14) >> 16) & 0x000001FF));
//    WRITE_REG(FLASH->TS2P, ((M32(0x1FFF0F20 + dwOffset * 0x14) >> 0) & 0x000000FF));
//    WRITE_REG(FLASH->TPS3, ((M32(0x1FFF0F20 + dwOffset * 0x14) >> 16) & 0x000007FF));
//    WRITE_REG(FLASH->PERTPE, ((M32(0x1FFF0F24 + dwOffset * 0x14) >> 0) & 0x0001FFFF));
//    WRITE_REG(FLASH->SMERTPE, ((M32(0x1FFF0F28 + dwOffset * 0x14) >> 0) & 0x0001FFFF));
//    WRITE_REG(FLASH->PRGTPE, ((M32(0x1FFF0F2C + dwOffset * 0x14) >> 0) & 0x0000FFFF));
//    WRITE_REG(FLASH->PRETPE, ((M32(0x1FFF0F2C + dwOffset * 0x14) >> 16) & 0x00000FFF));
//#endif
//}

//void UnInitRccAndFlashParam(void)
//{
//#ifdef USE_HSI_24MHZ
//    MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_FS, gdw_HSI_FS);
//    switch (gdw_HSI_FS)
//    {
//    case RCC_ICSCR_HSI_FS_2://24MHz
//        MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_TRIM, M32(0x1FFF0F10)&RCC_ICSCR_HSI_TRIM);
//        break;
//    case (RCC_ICSCR_HSI_FS_1|RCC_ICSCR_HSI_FS_0)://22.12MHz
//        MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_TRIM, M32(0x1FFF0F0C)&RCC_ICSCR_HSI_TRIM);
//        break;
//    case RCC_ICSCR_HSI_FS_1://16MHz
//        MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_TRIM, M32(0x1FFF0F08)&RCC_ICSCR_HSI_TRIM);
//        break;
//    case RCC_ICSCR_HSI_FS_0://8MHz
//        MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_TRIM, M32(0x1FFF0F04)&RCC_ICSCR_HSI_TRIM);
//        break;
//    default://4MHz
//        MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_TRIM, M32(0x1FFF0F00)&RCC_ICSCR_HSI_TRIM);
//        break;
//    }
//    while (RCC_CR_HSIRDY != READ_BIT(RCC->CR, RCC_CR_HSIRDY));
//#endif
//}

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int Init(unsigned long adr, unsigned long clk, unsigned long fnc)
{

//    InitRccAndFlashParam();

    FLASH->KEYR = FLASH_KEY1;                             // Unlock Flash
    FLASH->KEYR = FLASH_KEY2;

    FLASH->ACR  = 0x00000000;                             // Zero Wait State, no Prefetch
    FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

    if ((FLASH->OPTR & FLASH_OPTR_IWDG_SW) == 0x00)                   // Test if IWDG is running (IWDG in HW mode)
    {
        // Set IWDG time out to ~32.768 second
        IWDG->KR  = 0x5555;                                 // Enable write access to IWDG_PR and IWDG_RLR
        IWDG->PR  = 0x06;                                   // Set prescaler to 256
        IWDG->RLR = 0xFFF;                                   // Set reload value to 4095
    }

    return (0);
}
#endif

#ifdef FLASH_OPT
int Init(unsigned long adr, unsigned long clk, unsigned long fnc)
{

    InitRccAndFlashParam();

    FLASH->KEYR = FLASH_KEY1;                             // Unlock Flash
    FLASH->KEYR = FLASH_KEY2;

    FLASH->OPTKEYR = FLASH_OPTKEY1;                       // Unlock Option Bytes
    FLASH->OPTKEYR = FLASH_OPTKEY2;

    FLASH->ACR  = 0x00000000;                             // Zero Wait State, no Prefetch
    FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP


    if ((FLASH->OPTR & 0x1000) == 0x00)                   // Test if IWDG is running (IWDG in HW mode)
    {
        // Set IWDG time out to ~32.768 second
        IWDG->KR  = 0x5555;                                 // Enable write access to IWDG_PR and IWDG_RLR
        IWDG->PR  = 0x06;                                   // Set prescaler to 256
        IWDG->RLR = 0xFFF;                                  // Set reload value to 4095
    }

    return (0);
}
#endif


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int UnInit(unsigned long fnc)
{

//    UnInitRccAndFlashParam();

    FLASH->CR    |=  FLASH_CR_LOCK;                          // Lock Flash

    return (0);
}
#endif

#ifdef FLASH_OPT
int UnInit(unsigned long fnc)
{

    UnInitRccAndFlashParam();

    FLASH->CR    |=  FLASH_CR_LOCK;                          // Lock Flash
    FLASH->CR    |=  FLASH_CR_OPTLOCK;                       // Lock Option Bytes

    return (0);
}
#endif

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int EraseChip(void)
{
    FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

    FLASH->CR |=  FLASH_CR_MER;                              // Mass Erase Enabled
    FLASH->CR |=  FLASH_CR_EOPIE;
    M32(0x08000000) = 0xFF;
    __asm("DSB");

    while (FLASH->SR & FLASH_SR_BSY)
    {
        IWDG->KR = 0xAAAA;                                  // Reload IWDG
    }

    FLASH->CR &= ~FLASH_CR_MER;                              // Mass Erase Disabled
    FLASH->CR &= ~FLASH_CR_EOPIE;                            // Reset FLASH_EOPIE

    if (FLASH_SR_EOP != (FLASH->SR & FLASH_SR_EOP))             // Check for FLASH_SR_EOP
    {
        FLASH->SR |= FLASH_SR_EOP;
        return (1);                                         // Failed
    }

    return (0);                                           // Done
}
#endif

#ifdef FLASH_OPT
int EraseChip(void)
{

    /* erase chip is not needed for
       - Flash Option bytes
       - Flash One Time Programmable bytes
    */
    return (0);                                           // Done
}
#endif

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int EraseSector(unsigned long adr)
{

    FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

    FLASH->CR  |=  FLASH_CR_SER;                             // Sector Erase Enabled
    FLASH->CR  |=  FLASH_CR_EOPIE;
    M32(adr) = 0xFF;                                      // Sector Address
    __asm("DSB");

    while (FLASH->SR  & FLASH_SR_BSY)
    {
        IWDG->KR = 0xAAAA;                                  // Reload IWDG
    }

    FLASH->CR  &= ~FLASH_CR_SER;                             // Sector Erase Disabled
    FLASH->CR  &= ~FLASH_CR_EOPIE;                           // Reset FLASH_EOPIE

//  if (FLASH_EOP != (FLASH->SR & FLASH_EOP)) {           // Check for FLASH_SR_EOP
//    FLASH->SR |= FLASH_EOP;
//    return (1);                                         // Failed
//  }

    return (0);                                           // Done
}
#endif


#ifdef FLASH_OPT
int EraseSector(unsigned long adr)
{
    /* erase sector is not needed for
       - Flash Option bytes
       - Flash One Time Programmable bytes
    */
    return (0);                                           // Done
}
#endif

/*
 *  Blank Check Checks if Memory is Blank
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

int BlankCheck(unsigned long adr, unsigned long sz, unsigned char pat)
{
    return (1);                                            // Always Force Erase
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{

    sz = (sz + 127) & ~127;                               // Adjust size for 32 Words

    FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

    while (sz)
    {
        FLASH->CR  |=  FLASH_CR_PG;                            // Programming Enabled
        FLASH->CR  |=  FLASH_CR_EOPIE;

        for (u8 i = 0; i < 32; i++)
        {

            M32(adr + i * 4) = *((u32 *)(buf + i * 4));       // Program the first word of the Double Word
            if (i == 30)
            {
                FLASH->CR  |= FLASH_CR_PGSTRT;
            }
        }
        __asm("DSB");

        while (FLASH->SR & FLASH_SR_BSY)
        {
            IWDG->KR = 0xAAAA;                                // Reload IWDG
        }

        FLASH->CR  &= ~FLASH_CR_PG;                            // Programming Disabled
        FLASH->CR  &= ~FLASH_CR_EOPIE;                         // Reset FLASH_EOPIE

        //为了能够Program SRAM，屏蔽下面的EOP检查
//      if (FLASH_EOP != (FLASH->SR & FLASH_EOP)) {         // Check for FLASH_SR_EOP
//          FLASH->SR |= FLASH_EOP;
//          return (1);                                       // Failed
//      }

        adr += 128;                                         // Go to next Page
        buf += 128;
        sz  -= 128;
    }

    return (0);                                           // Done
}
#endif // PY32F030xx_64 || FLASH_OTP

#ifdef FLASH_OPT
int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{

    u32 optr;
    u32 sdkr;
    u32 wrpr;

    optr = *((u32 *)(buf + 0x00));
    sdkr = *((u32 *)(buf + 0x04));
    wrpr = *((u32 *)(buf + 0x0C));

    FLASH->SR |= FLASH_SR_EOP;                            // Reset FLASH_EOP

    FLASH->OPTR = (optr & 0x0000FFFF);                 // Write OPTR values
    FLASH->SDKR = (sdkr & 0x0000FFFF);                 // Write SDKR values
    FLASH->WRPR = (wrpr & 0x0000FFFF);                 // Write WRPR values

    FLASH->CR |= FLASH_CR_OPTSTRT;
    FLASH->CR   |= FLASH_CR_EOPIE;
    M32(0x40022080) = 0xFF;
    __asm("DSB");

    FLASH->CR  &= ~FLASH_CR_OPTSTRT;                      // Programming Disabled
    FLASH->CR  &= ~FLASH_CR_EOPIE;                        // Reset FLASH_EOPIE

#ifdef FLASH_OPT_OBL_LAUNCH
    FLASH->CR |= FLASH_CR_OBL_LAUNCH;
#endif

    return (0);                                           // Done
}
#endif //  FLASH_OPT


/*
 *  Verify Flash Contents
 *    Parameter:      adr:  Start Address
 *                    sz:   Size (in bytes)
 *                    buf:  Data
 *    Return Value:   (adr+sz) - OK, Failed Address
 */

#ifdef FLASH_OPT
unsigned long Verify(unsigned long adr, unsigned long sz, unsigned char *buf)
{
    u32 optr;
    u32 sdkr;
    u32 wrpr;

    optr = *((u32 *)(buf +  0x00));
    sdkr = *((u32 *)(buf +  0x04));
    wrpr = *((u32 *)(buf +  0x0C));

    if (M32(adr + 0x00) != optr)
    {
        return (adr + 0x00);
    }
    if (M32(adr + 0x04) != sdkr)
    {
        return (adr + 0x04);
    }
    if (M32(adr + 0x0C) != wrpr)
    {
        return (adr + 0x0C);
    }

    return (adr + sz);
}
#endif // FLASH_OPT
