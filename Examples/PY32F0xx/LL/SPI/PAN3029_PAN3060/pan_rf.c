/**
 * @file      pan_rf.c
 * @brief     PAN3029/PAN3060 driver implementation
 * @version   V1.0.1
 * @date      2025-08-18
 * @copyright Panchip Microelectronics Co., Ltd. All rights reserved.
 * @code
 *             ____              ____ _     _
 *            |  _ \ __ _ _ __  / ___| |__ (_)_ __
 *            | |_) / _` | '_ \| |   | '_ \| | '_ \
 *            |  __/ (_| | | | | |___| | | | | |_) |
 *            |_|   \__,_|_| |_|\____|_| |_|_| .__/
 *                                           |_|
 *            (C)2009-2025 PanChip
 * @endcode
 * @author    PanChip
 * @note      The encoding of this file is utf-8.
 */
#include <stdio.h>
#include <string.h>
#include "pan_param.h"
#include "pan_rf.h"

/**
 * @brief PAN3029/3060 receive packet structure
 * @note This structure is used to store received packets, including data length, data buffer, SNR and RSSI information
 */
volatile RfRxPkt_t g_RfRxPkt = {0};

/**
 * @brief PAN3029/3060 configuration parameter structure
 * @note This structure is used to store PAN3029/3060 configuration parameters, including transmit power, frequency, spreading factor, bandwidth, coding rate, etc.
 */
static volatile RfConfig_t g_RfCfgParams = {0};

/**
 * @brief Save current RF operation state
 */
static volatile RfOpState_t g_RfOperatetate;

/**
 * @brief Microsecond delay function
 * @param us Number of microseconds to delay
 * @note The specific implementation of this function needs to be modified according to the actual hardware platform.
 * @note Must ensure RF_DelayUs(1) is greater than or equal to 1us.
 */
__attribute__((optimize("O0"))) void RF_DelayUs(uint32_t us)
{
    uint32_t i, j;
    for (i = 0; i < us; i++)
    {
        for (j = 0; j < 12; j++)
        {
            __NOP(); /* Each loop takes approximately 1 cycle */
        }
    }
}

/**
 * @brief Millisecond delay function
 * @param ms Number of milliseconds to delay
 * @note The specific implementation of this function needs to be modified according to the actual hardware platform.
 * @note Must ensure RF_DelayMs(1) is greater than or equal to 1ms.
 */
void RF_DelayMs(uint32_t ms)
{
    LL_mDelay(ms); /* Call millisecond delay function */
}

/**
 * @brief SPI write single byte
 * @param Value Single byte data to write
 * @note This interface is a software SPI implementation, and the specific implementation may vary depending on the MCU platform.
 * @note Before calling this function, you must first call SPI_CS_LOW() to pull down the CS pin.
 * @note This function will set the SPI_MOSI pin to output mode and set it to input mode after transmission is complete.
 * @note PAN3029/3060 SPI interface uses 3-line SPI mode.
 * @note PAN3029/3060 SPI configuration:
 *       Clock polarity: Active low
 *       Clock phase: Sample data on first edge
 *       Data transmission order: MSB first
 * @note Taking sending 0xCC data as an example, the timing diagram is as follows:
 *      SPI_CS：  ____________________________________________________
 *      SPI_CLK:  ____|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__
 *      SPI_MOSI: ___|‾‾‾‾‾‾‾‾‾‾|___________|‾‾‾‾‾‾‾‾‾‾‾|_____________
 *      BIT DATA:     1     1     0     0      1     1      0     0
 */
void SPI_WriteByte(uint8_t Value)
{
    /* TODO: According to your hardware implementation, the following is a software SPI implementation example */
#if INTERFACE_MODE == USE_SPI_4LINE
    SPI_TxRxByte(Value);
#elif INTERFACE_MODE == USE_SPI_3LINE
    unsigned char i;

    SPI_MOSI_OUTPUT(); /* Set SPI_MOSI pin to output mode */

    /* Transmit 8-bit data in MSB first order */
    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW();    /* Pull down clock line, prepare to transfer data */
        if (Value & 0x80) /* Check highest bit data */
        {
            SPI_MOSI_HIGH(); /* If highest bit is 1, send high level */
        }
        else
        {
            SPI_MOSI_LOW(); /* If highest bit is 0, send low level */
        }

        Value <<= 1;    /* Shift left one bit, prepare to transfer the next bit */
        SPI_SCK_HIGH(); /* Raise clock line, transfer one bit of data */
    }

    SPI_MOSI_INPUT(); /* Set SPI_MOSI pin to input mode */
    SPI_SCK_LOW();    /* End transmission, pull down clock line, ensure clock line is low level */
#endif
}

/**
 * @brief SPI read single byte
 * @return unsigned char Read data byte
 * @note This interface is a software SPI implementation, and the specific implementation may vary depending on the MCU platform.
 * @note Before calling this function, you must first call SPI_CS_LOW() to pull down the CS pin.
 * @note This function will set the SPI_MOSI pin to output mode and set it to input mode after transmission is complete.
 * @note PAN3029/3060 SPI interface uses 3-line SPI mode.
 * @note PAN3029/3060 SPI configuration:
 *       Clock polarity: Active low
 *       Clock phase: Sample data on first edge
 *       Data transmission order: MSB first
 * @note Taking sending 0x33 data as an example, the timing diagram is as follows:
 *      SPI_CS：  ___________________________________________________
 *      SPI_CLK:  ___|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__
 *      SPI_MISO: _____________|‾‾‾‾‾‾‾‾‾‾‾|___________|‾‾‾‾‾‾‾‾‾‾‾‾‾
 *      BIT DATA:    0     0     1     1     0     0     1     1     
 */
unsigned char SPI_ReadByte(void)
{
    unsigned char readByte = 0;

    /* TODO: According to your hardware implementation, the following is a software SPI implementation example */
#if INTERFACE_MODE == USE_SPI_4LINE
    readByte = SPI_TxRxByte(0xFF);
#elif INTERFACE_MODE == USE_SPI_3LINE
    unsigned char i;
    unsigned char Value = 0;

    /* Receive 8-bit data in MSB first order */
    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW();  /* Pull down clock line, prepare to receive data */
        Value <<= 1;  /* Shift left one bit, prepare to receive the next bit */
        SPI_SCK_HIGH(); /* Raise clock line, trigger PAN3029 SPI to send data */
        if (SPI_MOSI_STATUS())
        {
            Value |= 0x01;
        }
    }
    SPI_SCK_LOW(); /* End transmission, pull down clock line, ensure clock line is low level */
    readByte = Value; /* Store received data in readByte */
#endif
    return readByte;
}

/**
 * @brief Get the number of trailing zeros in a byte
 * @param Value The value to check
 * @return The number of trailing zeros
 */
uint8_t __ctz(uint8_t Value)
{
    int i;

    for (i = 0; i < 8; ++i)
    {
        if ((Value >> i) & 1)
            return (uint8_t)i;
    }
    
    return 0;
}

/**
 * @brief Read single byte from specified register
 * @param Addr Register address to read from
 * @return uint8_t Value read from register
 * @note The SPI_CS_LOW(), SPI_CS_HIGH(), SPI_WriteByte()
 *       and SPI_ReadByte() functions of this function need to be modified according to the actual hardware implementation.
 */
uint8_t RF_ReadReg(uint8_t Addr)
{
    uint8_t Temp;
    SPI_CS_LOW();                      /* Pull down chip select signal, start SPI transmission */
    SPI_WriteByte((Addr << 1) & 0xFE); /* Bit7:0 is address, Bit0 is read/write bit, Bit0=0 indicates read operation */
    Temp = SPI_ReadByte();             /* Read register value */
    SPI_CS_HIGH();                     /* Pull up chip select signal, end SPI transmission */

    return Temp;
}

/**
 * @brief Write single byte to specified register
 * @param Addr Register address to write to
 * @param Value Single byte data to write to register
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation successful
 *         - RF_FAIL: Operation failed
 * @note The SPI_CS_LOW(), SPI_CS_HIGH(), SPI_WriteByte()
 *       and SPI_ReadByte() functions of this function need to be modified according to the actual hardware implementation.
 */
RF_Err_t RF_WriteReg(uint8_t Addr, uint8_t Value)
{
    SPI_CS_LOW();                      /* Pull down chip select signal, start SPI transmission */
    SPI_WriteByte((Addr << 1) | 0x01); /* Bit7:1 is address, Bit0 is read/write bit, Bit0=1 indicates write operation */
    SPI_WriteByte(Value);              /* Write register value */
    SPI_CS_HIGH();                     /* Pull up chip select signal, end SPI transmission */

#if USE_RF_REG_CHECK /* Whether to use register readback confirmation function */
    /*
     * The function of this part of the code is to read the value of the register and compare it with the written value, 
     * and print an error message if they are not equal. This code can be commented out or deleted after debugging is complete.
     */
    {
        uint8_t Temp = RF_ReadReg(Addr);
        if (Temp == Value)
        {
            // printf("Write reg ok: 0x%02x, 0x%02x, 0x%02x\n", Addr, Value, Temp);
            return RF_OK; /* Written value is equal to read value, return operation successful */
        }
        else
        {
            /* Read value is not equal to written value, return error */
            printf("Write reg fail: 0x%02x, 0x%02x, 0x%02x\r\n", Addr, Value, Temp);
            return RF_FAIL;
        }
    }
#else
    return RF_OK;
#endif
}

/**
 * @brief Continuously write multiple bytes to specified register area
 * @param Addr Start address of register area to write to
 * @param Buffer Buffer pointer to write to register
 * @param Size Number of bytes to write
 * @note The SPI_CS_LOW(), SPI_CS_HIGH(), SPI_WriteByte() functions of this function need to be modified according to the actual hardware implementation.
 */
void RF_WriteRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
    unsigned char i;
    SPI_CS_LOW();                      /* Pull down chip select signal, start SPI transmission */
    SPI_WriteByte((Addr << 1) | 0x01); /* Bit7:1 is address, Bit0 is read/write bit, Bit0=1 indicates write operation */
    for (i = 0; i < Size; i++)
    {
        SPI_WriteByte(Buffer[i]); /* Write register value */
    }
    SPI_CS_HIGH();              /* Pull up chip select signal, end SPI transmission */
}

/**
 * @brief Continuously read multiple bytes from specified register
 * @param Addr Register address to read from
 * @param Buffer Buffer pointer to store read data
 * @param Size Number of bytes to read
 * @note The SPI_CS_LOW(), SPI_CS_HIGH(), SPI_WriteByte()
 *       and SPI_ReadByte() functions of this function need to be modified according to the actual hardware implementation.
 */
void RF_ReadRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
    unsigned char i;
    SPI_CS_LOW();                      /* Pull down chip select signal, start SPI transmission */
    SPI_WriteByte((Addr << 1) & 0xFE); /* Bit7:0 is address, Bit0 is read/write bit, Bit0=0 indicates read operation */
    for (i = 0; i < Size; i++)
    {
        Buffer[i] = SPI_ReadByte(); /* Read register value */
    }
    SPI_CS_HIGH();             /* Pull up chip select signal, end SPI transmission */
}

/**
 * @brief Select register page
 * @param Page Register page to select, range 0~3
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation successful
 *         - RF_FAIL: Operation failed
 * @note If the current page is already the required page, no need to configure the register
 */
RF_Err_t RF_SetPage(uint8_t Page)
{
    static uint8_t gCurrPage = 0xFF;
    if(gCurrPage == Page)
    {
        return RF_OK;
    }
    gCurrPage = Page;
    RF_ASSERT(RF_WriteReg(0x00, gCurrPage)); /* Write page value to register 0x00 */
    return RF_OK;
}

/**
 * @brief Write single byte to specified register of selected page
 * @param Page Register page to write to, range 0~3
 * @param Addr Register address to write to
 * @param Value Single byte data to write to register
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation successful
 *         - RF_FAIL: Operation failed
 */
RF_Err_t RF_WritePageReg(uint8_t Page, uint8_t Addr, uint8_t Value)
{
    RF_SetPage(Page);
    RF_WriteReg(Addr, Value);

    return RF_OK;
}

/**
 * @brief Write multiple bytes to specified register area of selected page
 * @param Page Register page to write to, range 0~3
 * @param Addr Register address to write to
 * @param Buffer Buffer pointer to write to register
 * @param Size Number of bytes to write
 */
void RF_WritePageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
    RF_SetPage(Page);                 /* Select register page */
    RF_WriteRegs(Addr, Buffer, Size); /* Write register value */
}

/**
 * @brief Read single byte from specified register of selected page
 * @param Page Register page to read from, range 0~3
 * @param Addr Register address to read from
 * @return uint8_t Value read from register
 */
uint8_t RF_ReadPageReg(uint8_t Page, uint8_t Addr)
{
    RF_SetPage(Page);
    return RF_ReadReg(Addr);
}

/**
 * @brief Read multiple bytes from specified register area of selected page
 * @param Page Register page to read from, range 0~3
 * @param Addr Register address to read from
 * @param Buffer Buffer pointer to store read data
 * @param Size Number of bytes to read
 */
void RF_ReadPageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
    RF_SetPage(Page);                /* Select register page */
    RF_ReadRegs(Addr, Buffer, Size); /* Read register value */
}

/**
 * @brief Set specified bits of register in selected page
 * @param Page Register page to set bits, range 0~3
 * @param Addr Register address to set bits
 * @param Mask Bit mask to set, each bit set to 1 means corresponding bit will be set
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation successful
 *         - RF_FAIL: Operation failed
 */
RF_Err_t RF_SetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask)
{
    uint8_t Temp;

    RF_SetPage(Page);
    Temp = RF_ReadReg(Addr);
    RF_WriteReg(Addr, Temp | Mask);

    return RF_OK;
}

/**
 * @brief Reset specified bits of register in selected page
 * @param Page Register page to reset bits, range 0~3
 * @param Addr Register address to reset bits
 * @param Mask Bit mask to reset, each bit set to 1 means corresponding bit will be reset
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation successful
 *         - RF_FAIL: Operation failed
 */
RF_Err_t RF_ResetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask)
{
    uint8_t Temp;

    RF_SetPage(Page);                  /* Select register page */
    Temp = RF_ReadReg(Addr);           /* Read register value */
    RF_WriteReg(Addr, Temp & (~Mask)); /* Clear bits in register corresponding to mask */

    return RF_OK;
}

/**
 * @brief Write specified bits of register in selected page
 * @param Page Register page to write bits, range 0~3
 * @param Addr Register address to write bits
 * @param Value Value to write, each bit set to 1 means corresponding bit will be set
 * @param Mask Bit mask to write, each bit set to 1 means corresponding bit will be written
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation successful
 *         - RF_FAIL: Operation failed
 * @note This function will first clear bits in register corresponding to mask, then set new value
 * @note For example, to set bit 2 and bit 3 of page 1 register 0x08 to 0b10, other bits remain unchanged,
 *       call RF_WritePageRegBits(1, 0x08, 0x02, 0x0C);
 *       Where Value = 0x02, Mask = 0x0C, Value does not need to be left-shifted,
 *       because the mask already specifies the bits to be set.
 */
RF_Err_t RF_WritePageRegBits(uint8_t Page, uint8_t Addr, uint8_t Value, uint8_t Mask)
{
    uint8_t Temp;
    uint8_t shift = __ctz(Mask); /* Get shift value of mask */

    Value <<= shift; /* Shift value to align with mask */
    Value &= Mask;   /* Mask value to ensure only set bits are written */

    RF_SetPage(Page);                            /* Select register page */
    Temp = RF_ReadReg(Addr);                     /* Read register value */
    RF_WriteReg(Addr, (Temp & (~Mask)) | Value); /* Clear bits in register corresponding to mask, then set new value */

    return RF_OK;
}

/**
 * @brief Configure GPIO mode
 * @param[in] <GpioPin> GPIO pin number
 * @param[in] <GpioMode> GPIO mode
 *         - GPIO_MODE_INPUT: Input mode
 *         - GPIO_MODE_OUTPUT: Output mode
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation successful
 *         - RF_FAIL: Operation failed
 */
RF_Err_t RF_ConfigGpio(uint8_t GpioPin, uint8_t GpioMode)
{
    if(GpioMode == GPIO_MODE_INPUT)
    {
        if(GpioPin < 8)
        {
            RF_ASSERT(RF_SetPageRegBits(0, 0x63, (1 << GpioPin)));
        }
        else
        {
            RF_ASSERT(RF_SetPageRegBits(0, 0x64, (1 << (GpioPin - 8))));
        }
    }
    else if(GpioMode == GPIO_MODE_OUTPUT)
    {
        if(GpioPin < 8)
        {
            RF_SetPageRegBits(0, 0x65, (1 << GpioPin));
        }
        else
        {
            RF_SetPageRegBits(0, 0x66, (1 << (GpioPin - 8)));
        }
    }
    else
    {
        return RF_FAIL;
    }

    return RF_OK;
}

/**
 * @brief Write GPIO output level
 * @param[in] <GpioPin> GPIO pin number
 * @param[in] <Level> GPIO output level
 *         - 0: Low level
 *         - 1: High level
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation successful
 *         - RF_FAIL: Operation failed
 */
RF_Err_t RF_WriteGpioLevel(uint8_t GpioPin, uint8_t Level)
{
    if(GpioPin < 8)
    {
        RF_WritePageRegBits(0, 0x67, Level, (1 << GpioPin));
    }
    else
    {
        RF_WritePageRegBits(0, 0x68, Level, (1 << (GpioPin - 8)));
    }

    return RF_OK;
}

/**
 * @brief Read GPIO input level
 * @param[in] <GpioPin> GPIO pin number
 * @return GPIO input level
 *         - 0: Low level
 *         - 1: High level
 */
uint8_t RF_ReadGpioLevel(uint8_t GpioPin)
{
    uint8_t Temp;

    if(GpioPin < 6)
    {
        Temp = RF_ReadPageReg(0, 0x74);
    }
    else
    {
        Temp = RF_ReadPageReg(0, 0x75);
        GpioPin -= 6;
    }

    return (bool)((Temp >> GpioPin) & 0x01);
}

/**
 * @brief Initialize PAN3029/3060 antenna control GPIO
 * @note This function is used to initialize the GPIO used to control the antenna of PAN3029/3060.
 *       It configures the GPIO as output mode and sets the initial level to low.
 * @note If using MCU's GPIO to control the antenna switch, this function needs to be re-adapted.
 */
void RF_InitAntGpio(void)
{
    RF_ConfigGpio(MODULE_GPIO_RX, GPIO_MODE_OUTPUT);
    RF_ConfigGpio(MODULE_GPIO_TX, GPIO_MODE_OUTPUT);

    RF_WriteGpioLevel(MODULE_GPIO_RX, 0);
    RF_WriteGpioLevel(MODULE_GPIO_TX, 0);
}

/**
 * @brief Turn on PAN3029/3060's transmit antenna
 * @note This function is used to turn on the transmit antenna of PAN3029/3060.
 *       It sets the TX pin to high level and the RX pin to low level.
 * @note If using MCU's GPIO to control the antenna switch, this function needs to be re-adapted.
 */
void RF_TurnonTxAnt(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_RX, 0);
    RF_WriteGpioLevel(MODULE_GPIO_TX, 1);
}

/**
 * @brief Turn on PAN3029/3060's receive antenna
 * @note This function is used to turn on the receive antenna of PAN3029/3060.
 *       It sets the RX pin to high level and the TX pin to low level.
 * @note If using MCU's GPIO to control the antenna switch, this function needs to be re-adapted.
 */
void RF_TurnonRxAnt(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_TX, 0);
    RF_WriteGpioLevel(MODULE_GPIO_RX, 1);
}

/**
 * @brief Shutdown PAN3029/3060's antennas
 * @note This function is used to shutdown the antennas of PAN3029/3060.
 *       It sets both RX and TX pins to low level.
 * @note If using MCU's GPIO to control the antenna switch, this function needs to be re-adapted.
 */
void RF_ShutdownAnt(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_RX, 0);
    RF_WriteGpioLevel(MODULE_GPIO_TX, 0);
}

/**
 * @brief Initialize PAN3029/3060's TCXO control GPIO
 * @note This function is used to initialize the GPIO used to control the TCXO of PAN3029/3060.
 *       It configures the GPIO as output mode and sets the initial level to high.
 * @note If using MCU's GPIO to control the TCXO switch, this function needs to be re-adapted.
 */
void RF_InitTcxoGpio(void)
{
    RF_ConfigGpio(MODULE_GPIO_TCXO, GPIO_MODE_OUTPUT);
    RF_WriteGpioLevel(MODULE_GPIO_TCXO, 1);
}

/**
 * @brief Turn on PAN3029/3060's TCXO
 * @note This function is used to turn on the TCXO of PAN3029/3060.
 *       It sets the TCXO pin to high level.
 * @note If using MCU's GPIO to control the TCXO switch, this function needs to be re-adapted.
 */
void RF_TurnonTcxo(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_TCXO, 1);
}

/**
 * @brief Shutdown PAN3029/3060's TCXO
 * @note This function is used to shutdown the TCXO of PAN3029/3060.
 *       It sets the TCXO pin to low level.
 * @note If using MCU's GPIO to control the TCXO switch, this function needs to be re-adapted.
 */
void RF_TurnoffTcxo(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_TCXO, 0);
}

/**
 * @brief Enable LDO PA
 */
void RF_TurnonLdoPA(void)
{    
    RF_SetPageRegBits(0, 0x4F, 0x08);
}

/*
 * @brief Disable LDO PA
 */
void RF_TurnoffLdoPA(void)
{
    RF_ResetPageRegBits(0, 0x4F, 0x08);
}

/**
 * @brief Shutdown PAN3029/3060's internal and external PA
 */
void RF_TurnoffPA(void)
{
    RF_TurnoffLdoPA(); /* Shutdown internal PA */
    RF_ShutdownAnt();  /* Shutdown external PA */
    /* After transmission, if configured for DCDC power mode, then switch back to DCDC power mode */
    if(g_RfCfgParams.RegulatorMode == USE_DCDC)
    {
        RF_WritePageReg(3, 0x24, 0x08);
    }
}

/**
 * @brief Turn on PAN3029/3060's internal and external PA
 */
void RF_TurnonPA(void)
{
    /* If current power mode is DCDC, then switch to LDO mode before transmission */
    if(g_RfCfgParams.RegulatorMode == USE_DCDC)
    {
        RF_WritePageReg(3, 0x24, 0x00);
    }
    RF_TurnonLdoPA(); /* Turn on internal PA */
    RF_TurnonTxAnt(); /* Turn on external PA */
}

/**
 * @brief Set PAN3029/3060's chip mode
 * @param <ChipMode> Chip mode
 *        - CHIPMODE_MODE0
 *        - CHIPMODE_MODE1
 */
void RF_SetChipMode(RfChipMode_t ChipMode)
{
    if(ChipMode == CHIPMODE_MODE0)
    {
        /* Mode0 config */
        RF_WritePageRegBits(1, 0x25, 0, 0xF0);
        RF_WritePageRegBits(1, 0x25, 0, 0x08);
        RF_WritePageRegBits(3, 0x12, 1, 0x04);
        RF_WritePageRegBits(3, 0x12, 1, 0x10);
        RF_WritePageRegBits(0, 0x58, 1, 0x04); /* Enable crc interrupt */
    }
    else
    {
        /* Mode1 config */
        RF_WritePageRegBits(1, 0x25, 4, 0xF0);
        RF_WritePageRegBits(1, 0x25, 1, 0x08);
        RF_WritePageRegBits(3, 0x12, 0, 0x04);
        RF_WritePageRegBits(3, 0x12, 0, 0x10);
        RF_WritePageRegBits(0, 0x58, 0, 0x04); /* Disable crc interrupt */
    }
    g_RfCfgParams.ChipMode = ChipMode;
}

/**
 * @brief Get PAN3029/3060's chip mode
 * @param -
 */
RfChipMode_t RF_GetChipMode(void)
{
    return g_RfCfgParams.ChipMode;
}

/**
 * @brief Read a byte from PAN3029/3060's info area
 * @param <Addr> Register address
 *        <Pattern> Pattern match value
 *        <InfoAddr> Info area address
 * @return Byte value read from info area
 */
uint8_t RF_ReadInfoByte(uint8_t Addr, uint16_t Pattern, uint8_t InfoAddr)
{
    uint8_t Value;
    uint8_t Buffer[3];
    uint16_t Timeout = 10000;

    Buffer[0] = Pattern >> 8;
    Buffer[1] = Pattern & 0xFF;
    Buffer[2] = InfoAddr << 1;

    RF_WritePageRegs(2, Addr, Buffer, sizeof(Buffer));
    do
    {
        if (RF_ReadPageReg(0, 0x6C) & 0x80)
        {
            break;
        }
    } while (Timeout--);

    Value = RF_ReadPageReg(2, Addr);

    return Value;
}

/**
 * @brief Calibrate PAN3029/3060's RF related parameters
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation success
 *         - RF_FAIL: Operation fail
 */
RF_Err_t RF_Calibrate(void)
{
    int i;
    uint8_t Temp[3] = {0};

    /* Temp[0]: efuse[0x1E] - DCDCIMAX
       Temp[1]: efuse[0x1F] - DCDCREF
       Temp[2]: efuse[0x20] - PABIAS */
    RF_ResetPageRegBits(2, 0x3E, 0x08); // Unlock info
    for (i = 0; i < sizeof(Temp); i++)
    {
        Temp[i] = RF_ReadInfoByte(0x3B, 0x5AA5, 0x1E + i);
    }

    if (RF_ReadInfoByte(0x3B, 0x5AA5, 0x1C) == 0x5A)
    {
        RF_WritePageReg(2, 0x3D, 0xFD);

        if (Temp[2] != 0)
        {
            /* Write PABIAS */
            RF_WritePageReg(0, 0x45, Temp[2]);
        }

        RF_WritePageReg(3, 0x1C, (0xC0 | (Temp[0] & 0x1F))); /* Write DCDCIMAX */
        RF_WritePageReg(3, 0x1D, Temp[1]);                   /* Write DCDCREF */
    }
    RF_ASSERT(RF_SetPageRegBits(2, 0x3E, 0x08)); // Lock info

    return RF_OK;
}

/**
 * @brief Configure PAN3029/3060's AGC function
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation success
 *         - RF_FAIL: Operation fail
 */
RF_Err_t RF_ConfigAgc(void)
{
    /* Enable AGC function
       - [Page2][0x06][Bit0] equal to 0 means enable AGC function
       - [Page2][0x06][Bit1] equal to 1 means disable AGC function */
    RF_ASSERT(RF_ResetPageRegBits(2, 0x06, 0x01));

#if REGION_DEFAULT == REGION_CN470_510
    RF_WritePageRegs(2, 0x0A, (uint8_t *)g_LowFreqAgcCfg, 40);
#elif REGION_DEFAULT == REGION_EU_863_870 || REGION_DEFAULT == REGION_US_902_928 
    RF_WritePageRegs(2, 0x0A, (uint8_t *)g_HighFreqAgcCfg, 40);
#endif

    RF_ASSERT(RF_WritePageReg(2, 0x34, 0xEF));

    return RF_OK;
}

/**
 * @brief Configure PAN3029/3060's default RF register parameters
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation success
 *         - RF_FAIL: Operation fail
 */
RF_Err_t RF_ConfigDefaultParams(void)
{
    int i;
    for(i = 0; i < sizeof(g_RfDefaultConfig)/sizeof(PAN_RegCfg_t); i++)
    {
        RF_WritePageReg(g_RfDefaultConfig[i].Page, g_RfDefaultConfig[i].Addr, g_RfDefaultConfig[i].Value);
    }
    return RF_OK;
}

/**
 * @brief Initialize PAN3029/3060's RF transceiver to STB3 state
 * @return RF_Err_t Return operation result
 *         - RF_OK: Operation success
 *         - RF_FAIL: Operation fail
 * @note Call this function before configuring MCU's SPI and related GPIO pins
 */
RF_Err_t RF_Init(void)
{
#if USE_RF_RST_GPIO == 1
    RF_RESET_PIN_LOW();  /* Pull down chip reset pin, start reset */
    RF_DelayUs(100);     /* Ensure actual delay is at least 100us */
    RF_RESET_PIN_High(); /* Pull up chip reset pin, release reset */
    RF_DelayUs(100);     /* Ensure actual delay is at least 100us */
#endif
    /* [Pagex][0x04][BIT4] is reset control bit, 0 means reset chip, 1 means release reset */
    RF_WriteReg(0x04, 0x06); /* Start POR reset chip */
    RF_DelayUs(100);         /* Ensure actual delay is at least 100us */

#if INTERFACE_MODE == USE_SPI_4LINE
    RF_WriteReg(0x00, 0x03); /* Select register page 3 */
    RF_WriteReg(0x1A, 0x03); /* Enable 4line SPI */
#elif INTERFACE_MODE == USE_SPI_3LINE
    RF_WriteReg(0x00, 0x03); /* Select register page 3 */
    RF_WriteReg(0x1A, 0x83); /* Enable 3line SPI */
#endif

    RF_SetPage(0);                                    /* Select register page 0 */
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_DEEPSLEEP)); /* Enter deepsleep state */
    RF_DelayUs(10);                                   /* Ensure actual delay is at least 10us */
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_SLEEP));     /* Enter sleep state */
    RF_DelayUs(10);                                   /* Ensure actual delay is at least 10us */
    RF_ASSERT(RF_WritePageReg(3, 0x06, 0x20));        /* Enable ISO */
    RF_DelayUs(10);                                   /* Ensure actual delay is at least 10us */
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_STB1));      /* Enter stb1 state */
    RF_DelayUs(10);                                   /* Ensure actual delay is at least 10us */
#if USE_ACTIVE_CRYSTAL == 1                           /* If using active crystal, need to configure TCXO GPIO pin */
    RF_ASSERT(RF_WritePageReg(3, 0x26, 0xA0));        /* Enable kernel power and open active crystal channel */
    RF_DelayUs(100);                                  /* Ensure actual delay is at least 100us */
    RF_ASSERT(RF_WriteReg(0x04, 0x36));               /* Enable LFT and release POR reset */
    RF_DelayMs(1);                                    /* Ensure actual delay is at least 1ms */
    RF_InitTcxoGpio();                                /* Initialize TCXO GPIO pin */
#else
    RF_ASSERT(RF_WritePageReg(3, 0x26, 0x20));        /* Enable kernel power */
    RF_DelayUs(100);                                  /* Ensure actual delay is at least 100us */
    RF_ASSERT(RF_WriteReg(0x04, 0x36));               /* Enable LFT and release POR reset */
    RF_DelayMs(1);                                    /* Ensure actual delay is at least 1ms */
#endif
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_STB2));      /* Enter stb2 state */
    RF_DelayMs(1);                                    /* Ensure actual delay is at least 1ms */
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_STB3));      /* Enter stb3 state */
    RF_DelayUs(100);                                  /* Ensure actual delay is at least 100us */
    RF_ASSERT(RF_ConfigDefaultParams());              /* Configure default RF register parameters */
    RF_ASSERT(RF_Calibrate());                        /* Calibrate RF related parameters */
    RF_ASSERT(RF_ConfigAgc());                        /* Configure AGC function */
    RF_InitAntGpio();                                 /* Initialize antenna GPIO pins */
    g_RfOperatetate = RF_STATE_STB3;                  /* Set current operate state to STB3 */

    return RF_OK;
}

/**
 * @brief Configure PAN3029/3060's user parameters
 */
void RF_ConfigUserParams(void)
{
    RF_SetTxPower(22);                    /* Set power level */
    RF_SetFreq(RF_FREQ_DEFAULT);          /* Set frequency */
    RF_SetBW(RF_BW_DEFAULT);              /* Set bandwidth */
    RF_SetSF(RF_SF_DEFAULT);              /* Set spreading factor */
    RF_SetCR(RF_CR_DEFAULT);              /* Set channel coding rate */
    RF_SetCRC(RF_CRC_DEFAULT);            /* Set CRC check */
    RF_SetLDR(RF_LDR_DEFAULT);            /* Set low data rate mode */
    RF_SetPreamLen(RF_PREAMBLE_DEFAULT);  /* Set preamble length */
    RF_SetInvertIQ(RF_IQ_INVERT_DEFAULT); /* Set IQ invert */
    RF_SetRegulatorMode(USE_LDO);         /* Set regulator mode to LDO */
    RF_SetChipMode(CHIPMODE_MODE0);       /* Set chip mode to MODE0 */
}

/**
 * @brief Reset PAN3029/3060's logic
 */
void RF_ResetLogic(void)
{
    RF_WriteReg(0x00, 0x80);
    RF_WriteReg(0x00, 0x00);

    (void)RF_ReadReg(0x00); /* Need to read register 0x00 once to make reset effective */ 
}

/**
 * @brief Get PAN3029/3060's working state
 * @return RfOpState_t Current working state
 *         - RF_STATE_SLEEP:  Chip is in sleep mode
 *         - RF_STATE_STB3:   Chip is in standby mode
 *         - RF_STATE_TX:     Chip is in transmit mode
 *         - RF_STATE_RX:     Chip is in receive mode
 */
RfOpState_t RF_GetOperateState(void)
{
    return g_RfOperatetate;
}

/**
 * @brief Set PAN3029/3060's working state
 * @param <RfState> Working state
 *         - RF_STATE_SLEEP:  Chip is in sleep mode
 *         - RF_STATE_STB3:   Chip is in standby mode
 *         - RF_STATE_TX:     Chip is in transmit mode
 *         - RF_STATE_RX:     Chip is in receive mode
 */
void RF_SetOperateState(RfOpState_t RfState)
{
    g_RfOperatetate = RfState;
}

/**
 * @brief Set PAN3029/3060's working state
 * @param <RfState> Working state
 *        - RF_STATE_DEEPSLEEP
 *        - RF_STATE_SLEEP
 *        - RF_STATE_STB3
 *        - RF_STATE_TX
 *        - RF_STATE_RX
 */
void RF_SetRfState(uint8_t RfState)
{
    RF_WriteReg(0x02, RfState);
    g_RfOperatetate = (RfOpState_t)RfState;
}

/**
 * @brief Enter PAN3029/3060's deep sleep mode
 * @note This function is used to put the RF chip into deep sleep mode, shutting down the antenna power and TCXO power
 * @note This function will set the chip's working state to MODE_DEEPSLEEP
 * @note After executing this function, if you need to wake up the RF chip, you need to call the RF_Init() function to wake up the chip
 */
void RF_EnterDeepsleepState(void)
{
    RF_ShutdownAnt();                      /* Shutdown antenna */
    RF_WriteReg(0x02, RF_STATE_STB3);      /* Enter STB3 state */
    RF_DelayUs(150);                       /* Ensure actual delay is at least 150us */
    RF_WriteReg(0x02, RF_STATE_STB2);      /* Enter STB2 state */
    RF_DelayUs(10);                        /* Ensure actual delay is at least 10us */
    RF_WriteReg(0x02, RF_STATE_STB1);      /* Enter STB1 state */
    RF_DelayUs(10);                        /* Ensure actual delay is at least 10us */
#if USE_ACTIVE_CRYSTAL == 1                /* If using active crystal, need to shutdown TCXO power */
    RF_TurnoffTcxo();                      /* Shutdown TCXO power */
#endif
    RF_WriteReg(0x04, 0x06);               /* Shutdown LFT */
    RF_DelayUs(10);                        /* Ensure actual delay is at least 10us */
    RF_WriteReg(0x02, RF_STATE_SLEEP);     /* Enter SLEEP state */
    RF_DelayUs(10);                        /* Ensure actual delay is at least 10us */
    RF_WritePageReg(3, 0x06, 0x00);        /* Shutdown ISO */
    RF_DelayUs(10);                        /* Ensure actual delay is at least 10us */
    RF_WritePageReg(3, 0x26, 0x00);        /* Shutdown internal power */
    RF_DelayUs(10);                        /* Ensure actual delay is at least 10us */
    RF_WriteReg(0x02, RF_STATE_DEEPSLEEP); /* Enter DEEPSLEEP state */

    g_RfOperatetate = RF_STATE_DEEPSLEEP;
}

/**
 * @brief Enter PAN3029/3060's sleep mode
 * @note This function is used to put the RF chip into sleep mode, shutting down the antenna power and TCXO power
 * @note This function will set the chip's working state to MODE_SLEEP
 * @note After executing this function, if you need to wake up the RF chip, you need to call the RF_ExitSleepState() function
 */
void RF_EnterSleepState(void)
{
    RF_ShutdownAnt();                   /* Shutdown antenna */
    RF_WriteReg(0x02, RF_STATE_STB3);   /* Enter STB3 state */
    RF_DelayUs(150);                    /* Ensure actual delay is at least 150us */
    RF_WriteReg(0x02, RF_STATE_STB2);   /* Enter STB2 state */
    RF_DelayUs(10);                     /* Ensure actual delay is at least 10us */
    RF_WriteReg(0x02, RF_STATE_STB1);   /* Enter STB1 state */
    RF_DelayUs(10);                     /* Ensure actual delay is at least 10us */
#if USE_ACTIVE_CRYSTAL == 1             /* If using active crystal, need to shutdown TCXO power */
    RF_TurnoffTcxo();                   /* 关闭TCXO电源 */
#endif
    RF_WriteReg(0x04, 0x16);            /* Shutdown LFT */
    RF_DelayUs(10);                     /* Ensure actual delay is at least 10us */
    RF_WriteReg(0x02, RF_STATE_SLEEP);  /* Enter SLEEP state */
    RF_DelayUs(10);                     /* Ensure actual delay is at least 10us */
    RF_ResetPageRegBits(3, 0x06, 0x20); /* Shutdown ISO */
    RF_DelayUs(10);                     /* Ensure actual delay is at least 10us */
    RF_WritePageReg(3, 0x26, 0x00);     /* Shutdown internal power */

    g_RfOperatetate = RF_STATE_SLEEP;
}

/**
 * @brief Exit sleep mode
 * @note This function is used to exit sleep mode, powering on the antenna and TCXO
 * @note This function will set the chip's working state to MODE_STDBY
 */
void RF_ExitSleepState(void)
{
    RF_SetPageRegBits(3, 0x06, 0x20); /* Enable ISO */
    RF_DelayUs(10);                   /* Ensure actual delay is at least 10us */
    RF_WriteReg(0x02, RF_STATE_STB1); /* Enter STB1 state */
    RF_DelayUs(10);                   /* Ensure actual delay is at least 10us */
#if USE_ACTIVE_CRYSTAL == 1
    RF_WritePageReg(3, 0x26, 0xA0);   /* Enable internal power and active crystal channel */
    RF_DelayUs(100);                  /* Ensure actual delay is at least 100us */
    RF_WriteReg(0x04, 0x36);          /* Enable LFT */
    RF_DelayUs(100);                  /* Ensure actual delay is at least 100us */
    RF_TurnonTcxo();                  /* Power on TCXO */
#else
    RF_WritePageReg(3, 0x26, 0x20);   /* Enable internal power */
    RF_DelayUs(100);                  /* Ensure actual delay is at least 100us */
    RF_WriteReg(0x04, 0x36);          /* Enable LFT */
    RF_DelayUs(100);                  /* Ensure actual delay is at least 100us */
#endif
    RF_WriteReg(0x02, RF_STATE_STB2); /* Enter STB2 state */
    RF_DelayMs(1);                    /* Ensure actual delay is at least 1ms */
    RF_WriteReg(0x02, RF_STATE_STB3); /* Enter STB3 state */
    RF_DelayUs(100);                  /* Ensure actual delay is at least 100us */

    g_RfOperatetate = RF_STATE_STB3;
}

/**
 * @brief Enter standby mode
 * @note This function will set the chip's working state to MODE_STDBY
 */
void RF_EnterStandbyState(void)
{
    RF_SetRfState(RF_STATE_STB3);
    RF_SetOperateState(RF_STATE_STB3);
}

/**
 * @brief Check if the RF chip is in sleep state
 * @note This function is used to check if the RF chip is in sleep state,
 * @note If it is in sleep state, it will exit sleep state and enter standby state
 * @note This function will set the chip's working state to MODE_STDBY
 */
void RF_CheckDeviceReady(void)
{
    if (RF_GetOperateState() == RF_STATE_SLEEP)
    {
        RF_ExitSleepState();
    }
}

/**
 * @brief Set the power supply mode of the RF chip
 * @param <RegulatorMode> Power supply mode
 *         - USE_LDO: Use LDO power supply
 *         - USE_DCDC: Use DCDC power supply
 * @note In transmit state, RF must use LDO power supply mode,
 *       other states can choose any power supply mode.
 */
void RF_SetRegulatorMode(RfRegulatorMode_t RegulatorMode)
{
    RF_WritePageReg(3, 0x24, (RegulatorMode == USE_DCDC) ? 0x08 : 0x00);
    g_RfCfgParams.RegulatorMode = RegulatorMode;
}

/**
 * @brief Set the frequency of the RF chip
 * @param <Frequency> Communication frequency (Hz)
 * @note  The supported frequency range is:
 *        Low frequency band:
 *         - 138.33MHz ~ 282.5MHz
 *         - 405.00MHz ~ 565.00MHz
 *        High frequency band:
 *         - 810.00MHz ~ 1080.00MHz
 */
RF_Err_t RF_SetFreq(uint32_t Frequency)
{
    int i;
    uint32_t Fa, Fb;
    uint32_t Temp = 0;
    uint32_t IntegerPart;
    uint8_t FreqReg[4], Fab[3];
    uint32_t FreqTableNum = (sizeof(g_RfFreqTable) / sizeof(RadioFreqTable_t));
   
    if (Frequency < g_RfFreqTable[0].StartFreq || Frequency > g_RfFreqTable[FreqTableNum - 1].StopFreq)
    {
        return RF_FAIL;
    }
    
    /* Find the matching frequency range in the frequency table */
    for (i = 0; i < FreqTableNum; i++)
    {
        if (Frequency > g_RfFreqTable[i].StartFreq && Frequency <= g_RfFreqTable[i].StopFreq)
        {
            uint8_t LoMux = (g_RfFreqTable[i].LoParam & 0x70) >> 4;
            Temp = Frequency * g_VcoDivTable[LoMux];
            RF_WritePageRegs(0, 0x40, (uint8_t *)&g_RfFreqTable[i].VcoParam, 2);
            RF_WriteReg(0x3D, g_RfFreqTable[i].LoParam);
            break;
        }
    }

    /* No frequency range matched */
    if (i >= FreqTableNum)
    {
        return RF_FAIL;
    }
    
    IntegerPart = Temp / 32000000;
    Fa = IntegerPart - 20;
    Fb = (Temp % 32000000) / 40000;

    FreqReg[0] = (uint8_t)Frequency;
    FreqReg[1] = (uint8_t)(Frequency >> 8);
    FreqReg[2] = (uint8_t)(Frequency >> 16);
    FreqReg[3] = (uint8_t)(Frequency >> 24);
    RF_WritePageRegs(3, 0x09, FreqReg, 4);

    Fab[0] = (uint8_t)(Fa);
    Fab[1] = (uint8_t)(Fb);
    Fab[2] = (uint8_t)((Fb >> 8) & 0x0F);
    RF_WritePageRegs(3, 0x15, Fab, 3);

    g_RfCfgParams.Frequency = Frequency;

    return RF_OK;
}

/**
 * @brief Set IQ inversion
 * @param <NewState> Enable or disable IQ inversion
 *         - true: Enable IQ inversion
 *         - false: Disable IQ inversion
 */
void RF_SetInvertIQ(bool NewState)
{
    if (NewState)
    {
        /*
         * BIT6 = 0: invert rx IQ
         * BIT5 = 1: invert tx IQ
         */
        RF_WritePageRegBits(1, 0x0E, 0x01, 0x40 | 0x20);
        g_RfCfgParams.InvertIQ = RF_IQ_INVERTED;
    }
    else
    {
        /*
         * BIT6 = 1: non-invert rx IQ
         * BIT5 = 0: non-invert tx IQ
         */
        RF_WritePageRegBits(1, 0x0E, 0x02, 0x40 | 0x20);
        g_RfCfgParams.InvertIQ = RF_IQ_NORMAL;
    }
}

/**
 * @brief Set the preamble length
 * @param <PreamLen> Preamble length value
 *         Range: 4 - 65535
 */
void RF_SetPreamLen(uint16_t PreamLen)
{
    uint8_t Temp[2] = {(uint8_t)(PreamLen), (uint8_t)((PreamLen >> 8))};
    RF_WritePageRegs(3, 0x13, Temp, 2);
    g_RfCfgParams.PreambleLen = PreamLen;
}

/**
 * @brief Set the sync word
 * @param <SyncWord> Sync word value
 * @note PAN3029/3060 supports sync word size of 1 byte
 * @note Sync word is used for synchronization detection when receiving packets,
 *       typically needs to be set the same as the sync word when sending and receiving packets
 *       For example, if the sync word is set to 0x12 when sending packets,
 *       it must also be set to 0x12 when receiving packets.
 */
void RF_SetSyncWord(uint8_t SyncWord)
{
    RF_WritePageReg(3, 0x0F, SyncWord);
    g_RfCfgParams.SyncWord = SyncWord;
}

/**
 * @brief Set the transmit power
 * @param <TxPower> Transmit power level, range: 1-22
 * @note Power level corresponding power values are shown in the table below:
 *|------|------------------|------------------|------------------|------------------|
 *|Level | 410MHz 功率(dBm) | 430MHz 功率(dBm)  | 450MHz 功率(dBm) | 460MHz 功率(dBm) |
 *|------|------------------|------------------|------------------|------------------|
 *|  1   |       -18.7      |      -18.2       |       -18.8      |       -19.7      |
 *|  2   |       -8.3       |      -7.9        |       -8.6       |       -9.5       |
 *|  3   |        1.6       |       1.7        |        0.9       |       -0.1       |
 *|  4   |        3.9       |       4.5        |        4.1       |        3.3       |
 *|  5   |        4.9       |       5.3        |        4.8       |        3.9       |
 *|  6   |        5.4       |       5.5        |        4.8       |        3.8       |
 *|  7   |        7.1       |       7.6        |        7.3       |        6.8       |
 *|  8   |        7.7       |       8.2        |        8.0       |        7.5       |
 *|  9   |        8.6       |       9.2        |        8.8       |        8.1       |
 *|  10  |        9.0       |       9.3        |        9.1       |        8.9       |
 *|  11  |        10.3      |       10.7       |        10.6      |        10.3      |
 *|  12  |        10.8      |       11.0       |        10.8      |        10.6      |
 *|  13  |        11.6      |       11.8       |        11.6      |        11.5      |
 *|  14  |        12.8      |       13.2       |        13.1      |        12.9      |
 *|  15  |        13.8      |       14.3       |        14.2      |        14.0      |
 *|  16  |        14.9      |       15.4       |        15.4      |        15.0      |
 *|  17  |        15.4      |       16.0       |        15.8      |        15.3      |
 *|  18  |        16.1      |       16.6       |        16.5      |        16.3      |
 *|  19  |        16.5      |       17.0       |        16.9      |        16.7      |
 *|  20  |        17.4      |       17.9       |        17.8      |        17.6      |
 *|  21  |        18.2      |       18.3       |        18.0      |        17.7      |
 *|  22  |        19.4      |       19.4       |        19.1      |        18.7      |
 *|------|------------------|------------------|------------------|------------------|
 */
void RF_SetTxPower(uint8_t TxPower)
{
    int Index;
    uint8_t Temp1, Temp2;
    static bool PaBiasReadFlag = false; // for read efuse only once
    static uint8_t PaBiasVal = 0;
    
    TxPower = (TxPower > RF_MAX_RAMP ? RF_MAX_RAMP : TxPower);
    TxPower = (TxPower < RF_MIN_RAMP ? RF_MIN_RAMP : TxPower);
    
    Index = TxPower - 1;

    /* Modulate wave ramp mode */
    RF_WritePageReg(3, 0x22, g_RfPowerRampCfg[Index].Ldo & 0x01);
    RF_WritePageReg(0, 0x1E, g_RfPowerRampCfg[Index].Ramp);
    RF_WritePageReg(0, 0x4B, g_RfPowerRampCfg[Index].Ldo >> 4);
    
    if (g_RfPowerRampCfg[Index].PAbias != 0x70)
    {
        RF_SetPageRegBits(0, 0x46, 0x04); // page0, reg0x46, bit2=1
    }
    else
    {
        RF_ResetPageRegBits(0, 0x46, 0x04); // page0, reg0x46, bit2=0
    }
    
    if(!PaBiasReadFlag)
    {
        PaBiasReadFlag = true;

        RF_ResetPageRegBits(2, 0x3E, 0x08); /* RF unlock info */
        PaBiasVal = RF_ReadInfoByte(0x3B, 0x5AA5, 0x20);
        RF_SetPageRegBits(2, 0x3E, 0x08);  /* RF lock info */

        if (PaBiasVal == 0)
        {
            PaBiasVal = 8;
        }
    }

    Temp1 = PaBiasVal - (g_RfPowerRampCfg[Index].PAbias & 0x0F);
    Temp2 = (g_RfPowerRampCfg[Index].PAbias & 0xF0) | Temp1;
    RF_WritePageReg(0, 0x45, Temp2);

    g_RfCfgParams.TxPower = TxPower; /* save current TxPower value */
}

/**
 * @brief Set the modulation bandwidth
 * @param <BandWidth> Modulation bandwidth value
 *         - RF_BW_062K / RF_BW_125K / RF_BW_250K / RF_BW_500K
 * @note The larger the modulation bandwidth, the higher the data rate, but the shorter the transmission distance
 * @note PAN3029 chip supports modulation bandwidth range from RF_BW_062K to RF_BW_500K
 * @note PAN3060 chip supports modulation bandwidth range from RF_BW_125K to RF_BW_500K
 */
void RF_SetBW(uint8_t BandWidth)
{
    /* Page 3, Reg 0x0D, Bit[7:4] = BandWidth */
    RF_WritePageRegBits(3, 0x0D, BandWidth, 0xF0);

    if (BandWidth != RF_BW_500K)
    {
        RF_SetPageRegBits(2, 0x3F, 0x02);
    }
    else
    {
        RF_ResetPageRegBits(2, 0x3F, 0x02);
    }

    g_RfCfgParams.Bandwidth = (RfBandwidths_t)BandWidth; // save current BW value
}

/**
 * @brief Set the spreading factor
 * @param <SpreadFactor> Spreading factor value
 *         - RF_SF5 / RF_SF6 / RF_SF7 / RF_SF8 / RF_SF9 / RF_SF10 / RF_SF11 / RF_SF12
 * @note The larger the spreading factor, the farther the transmission distance, but the lower the data rate
 * @note PAN3029 chip supports spreading factor range from RF_SF5 to RF_SF12
 * @note PAN3060 chip supports spreading factor range from RF_SF5 to RF_SF9
 */
void RF_SetSF(uint8_t SpreadFactor)
{
    /* Page 3, Reg 0x0E, Bit[7:4] = SpreadFactor */
    RF_WritePageRegBits(3, 0x0E, SpreadFactor, 0xF0);

    g_RfCfgParams.SpreadingFactor = (RfSpreadFactor_t)SpreadFactor; // save current SF value
}

/**
 * @brief Set the coding rate
 * @param <CodingRate> Coding rate value
 *         - RF_CR_4_5 / RF_CR_4_6 / RF_CR_4_7 / RF_CR_4_8
 */
void RF_SetCR(uint8_t CodingRate)
{
    /* Page 3, Reg 0x0D, Bit[3:1] = CodingRate */
    RF_WritePageRegBits(3, 0x0D, CodingRate, 0x0E);

    g_RfCfgParams.CodingRate = (RfCodingRates_t)CodingRate; // save current CR value
}

/**
 * @brief Set the CRC mode
 * @param <CrcMode> CRC mode value
 *        - RF_CRC_ON: Enable CRC check
 *        - RF_CRC_OFF: Disable CRC check
 */
void RF_SetCRC(uint8_t CrcMode)
{
    /* Page 3, Reg 0x0D, Bit[0] = CRC */
    RF_WritePageRegBits(3, 0x0E, CrcMode, 0x08);

    g_RfCfgParams.CrcMode = (RfCrcModes_t)CrcMode; // save current CRC value
}

/**
 * @brief Set the low datarate optimize mode
 * @param <LdrMode> Low datarate optimize mode value
 *         - RF_LDR_ON: Enable low datarate optimize mode
 *         - RF_LDR_OFF: Disable low datarate optimize mode
 */
void RF_SetLDR(uint8_t LdrMode)
{
    /* Page 3, Reg 0x12, Bit[3] = LDR */
    RF_WritePageRegBits(3, 0x12, LdrMode, 0x08);

    g_RfCfgParams.LowDatarateOptimize = LdrMode; // save current LDR value
}

/**
 * @brief Set modem mode
 * @param <modem_mode>
 *         - MODEM_MODE_NORMAL
 *         - MODEM_MODE_MULTI_SECTOR
 * @note This function should be called after RF_SetSF(uint8_t SpreadFactor)
 */
void RF_SetModemMode(uint8_t ModemMode)
{
    if (ModemMode == MODEM_MODE_NORMAL)
    {
        RF_WritePageReg(1, 0x0B, 0x08);
    }
    else if (ModemMode == MODEM_MODE_MULTI_SECTOR)
    {
        RF_WritePageReg(1, 0x0B, 0x18);
        if( g_RfCfgParams.SpreadingFactor <= RF_SF6 )
        {
            RF_WritePageReg(1, 0x2F, 0x74);
            RF_WritePageReg(1, 0x30, 0x01);
        }
        else
        {
            RF_WritePageReg(1, 0x2F, 0x54);
            RF_WritePageReg(1, 0x30, 0x40);
        }
    }
}

/**
 * @brief Set the transmit mode
 * @param Buffer Data buffer to be sent
 * @param Size Number of bytes to be sent
 * @note Ensure that the RF is in standby (STB3) mode before calling this function
 * @note This function is a single transmission mode, and will enter standby (STB3) mode after transmission is complete
 * @note After transmission is complete, a TX_DONE interrupt will be triggered
 */
void RF_SetTx(uint8_t *Buffer, uint8_t Size)
{
    RF_TxSinglePkt(Buffer, Size);
    g_RfOperatetate = RF_STATE_TX;
}

/**
 * @brief Set the receive mode
 * @param TimeoutMs Receive timeout time, unit is millisecond
 *        - 0: Continuous receive mode
 *        - >0: Single receive mode, timeout will trigger timeout interrupt and enter standby (STB3) mode automatically
 * @note Ensure that the RF is in standby (STB3) mode before calling this function
 */
void RF_SetRx(uint32_t TimeoutMs)
{
    if (TimeoutMs == 0)
    {
        RF_EnterContinousRxState();
    }
    else
    {
        RF_EnterSingleRxWithTimeout(TimeoutMs);
    }

    g_RfOperatetate = RF_STATE_RX;
}

/**
 * @brief Start CAD detection
 * @param <Threshold> CAD detection threshold
 *         - RF_CAD_THRESHOLD_0A
 *         - RF_CAD_THRESHOLD_10
 *         - RF_CAD_THRESHOLD_15
 *         - RF_CAD_THRESHOLD_20
 * @param <Chirps> CAD detection symbol number
 *         - RF_CAD_01_SYMBOL
 *         - RF_CAD_02_SYMBOL
 *         - RF_CAD_03_SYMBOL
 *         - RF_CAD_04_SYMBOL
 */
void RF_StartCad(uint8_t Threshold, uint8_t Chirps)
{
    /* Configure GPIO11 as output for CAD indication */
    RF_ConfigGpio(MODULE_GPIO_CAD_IRQ, GPIO_MODE_OUTPUT);

    /* [Page0][Reg0x5E][Bit6] = 0, enable GPIO11 CAD indication */
    RF_ResetPageRegBits(0, 0x5E, 0x40);

    RF_WritePageReg(1, 0x0F, Threshold);            /* [Page1][Reg0x0F] = Threshold */
    RF_WritePageRegBits(1, 0x25, Chirps - 1, 0x03); /* [Page1][Reg0x25][Bit[1:0]] = Chirps - 1 */
    RF_WritePageReg(1, 0x35, 0xFE);                 /* [Page1][Reg0x35] payload cad config */
    RF_EnterContinousRxState();                     /* Enter continous RX state */
}

/**
 * @brief Set the CAD detection threshold
 * @param <Threshold> CAD detection threshold
 *         - RF_CAD_THRESHOLD_0A
 *         - RF_CAD_THRESHOLD_10
 *         - RF_CAD_THRESHOLD_15
 *         - RF_CAD_THRESHOLD_20
 */
void RF_SetCadThreshold(uint8_t Threshold)
{
    RF_WritePageReg(1, 0x0F, Threshold);
}

/**
 * @brief Set the CAD detection symbol number
 * @param <Chirps> CAD detection symbol number
 *         - RF_CAD_01_SYMBOL
 *         - RF_CAD_02_SYMBOL
 *         - RF_CAD_03_SYMBOL
 *         - RF_CAD_04_SYMBOL
 */
void RF_SetCadChirps(uint8_t Chirps)
{
    RF_WritePageRegBits(1, 0x25, Chirps - 1, 0x03); /* [Page1][Reg0x25][Bit[1:0]] = Chirps */
}

/**
 * @brief Stop CAD detection
*/
void RF_StopCad(void)
{
    RF_SetPageRegBits(0, 0x5E, 0x40); /* [Page0][Reg0x5E][Bit6] = 1, disable GPIO11 CAD indication */
    RF_WritePageReg(1, 0x0F, 0x0A);   /* Reset CAD threshold */
    RF_WritePageReg(1, 0x35, 0xF4);   /* Reset payload cad config */
    RF_SetRfState(RF_STATE_STB3);     /* Enter standby state */
    RF_ResetLogic();                  /* Soft reset the RF chip, clear cad state */
    g_RfOperatetate = RF_STATE_STB3;
}

/**
 * @brief Set the transmit mode
 * @param <TxMode>
 *         - RF_TX_SINGLE：Single transmission mode
 *         - RF_TX_CONTINOUS：Continuous transmission mode
 * @note This function only sets the transmission mode, and does not change the working state of the chip
 */
void RF_SetTxMode(uint8_t TxMode)
{
    RF_WritePageRegBits(3, 0x06, TxMode, 0x04); /* [Page3][Reg0x06][Bit[2]] = TxMode */
}

/**
 * @brief Send a single packet
 * @param <Buffer> Data buffer to be sent
 * @param <Size> Number of bytes to be sent
 * @note After the transmission is complete, a TX_DONE interrupt will be triggered
 * @note In the TX_DONE interrupt, you need to call the RF_TurnoffPA() function to turn off the internal and external PA of the chip
 */
void RF_TxSinglePkt(uint8_t *Buffer, uint8_t Size)
{
    RF_WriteReg(0x02, RF_STATE_STB3); /* Enter standby state */
    RF_SetTxMode(RF_TX_MODE_SINGLE);  /* Set single transmission mode */
    RF_WritePageReg(1, 0x0C, Size);   /* Set payload length */
    RF_TurnonPA();                    /* PA must be turned on before transmitting data */
    RF_SetRfState(RF_STATE_TX);       /* Set the chip to transmit state */
    RF_WriteRegs(0x01, Buffer, Size); /* Write data to FIFO, after writing, the chip will start transmitting data, where 0x01 is the FIFO register address */
}

/**
 * @brief Set the receive mode
 * @param <RxMode>
 *         - RF_RX_SINGLE: Single receive mode, automatically enter standby mode after receiving a packet
 *         - RF_RX_SINGLE_TIMEOUT：Single receive mode with timeout, automatically enter standby mode after timeout
 *         - RF_RX_CONTINOUS：Continuous receive mode, continue receiving after receiving a packet
 * @note This function only sets the receive mode, and does not change the working state of the chip
 */
void RF_SetRxMode(uint8_t RxMode)
{
    RF_WritePageRegBits(3, 0x06, RxMode, 0x03); /* [Page3][Reg0x06][Bit[1:0]] = RxMode */
}

/**
 * @brief Enter continuous receive state
 * @note Calling this function will put the chip into continuous receive state
 * @note This function will set the working state of the chip to MODE_RX
 */
void RF_EnterContinousRxState(void)
{
    RF_SetRfState(RF_STATE_STB3);       /* Enter standby state */
    RF_TurnonRxAnt();                   /* Turn on receive antenna switch */
    RF_TurnoffLdoPA();                  /* Turn off internal PA */
    RF_SetRxMode(RF_RX_MODE_CONTINOUS); /* Set receive mode to continuous receive */
    RF_SetRfState(RF_STATE_RX);         /* Enter receive state */
}

/**
 * @brief Set the receive timeout time
 * @param <TimeoutMs> Timeout time, unit is ms
 *         Timeout time range: 0~65535ms
 * @note This function only sets the receive timeout time, and does not change the working state of the chip
 */
void RF_SetRxTimeout(uint16_t TimeoutMs)
{
    uint8_t Temp[2] = {(uint8_t)TimeoutMs, (uint8_t)(TimeoutMs >> 8)};
    RF_WritePageRegs(3, 0x07, Temp, 2);
}

/**
 * @brief Enter single receive state with timeout
 * @param <TimeoutMs> Timeout time, unit is ms
 *            Timeout time range: 1~65535ms
 * @note Calling this function will put the chip into single receive state with timeout
 * @note This function will set the working state of the chip to MODE_RX
 */
void RF_EnterSingleRxWithTimeout(uint16_t TimeoutMs)
{
    RF_SetRfState(RF_STATE_STB3);            /* Enter standby state */
    RF_TurnonRxAnt();                        /* Turn on receive antenna switch */
    RF_TurnoffLdoPA();                       /* Turn off internal PA */
    RF_SetRxTimeout(TimeoutMs);              /* Set receive timeout time */
    RF_SetRxMode(RF_RX_MODE_SINGLE_TIMEOUT); /* Set receive mode to single receive mode with timeout */
    RF_SetRfState(RF_STATE_RX);              /* Enter receive state */
}

/**
 * @brief Get the receive data length
 * @return Receive data length
 * @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the receive length register
 */
uint8_t RF_GetRxPayloadLen(void)
{
    return RF_ReadPageReg(1, 0x7D);
}

/**
 * @brief Get the receive data length and content
 * @param *Buffer Pointer to the receive data buffer
 * @return Received data length
 * @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the receive length register
 */
uint8_t RF_GetRecvPayload(uint8_t *Buffer)
{
    uint8_t Size;
    Size = RF_GetRxPayloadLen();     /* Get receive data length */
    RF_ReadRegs(0x01, Buffer, Size); /* Read received data from FIFO, where 0x01 is the FIFO register address */
    return Size;
}

/**
 * @brief Get the RSSI value of the received packet
 * @return RSSI value
 * @note RSSI value range: -125~-10, unit: dBm, RSSI value is less than or equal to sensitivity value
 * @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the signal strength register
 */
int8_t RF_GetPktRssi(void)
{
    return RF_ReadPageReg(1, 0x7F);
}

/**
 * @brief Get the real-time RSSI value
 * @return RSSI value
 * @note RSSI value range: -125~-10, unit: dBm
 * @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the signal strength register
 */
int8_t RF_GetRealTimeRssi(void)
{
    /* Set Bit[2] of register 0x06 to 0, then set it to 1, to update the value of [Page1][Reg0x7E] */
    RF_ResetPageRegBits(2, 0x06, 0x04);
    RF_SetPageRegBits(2, 0x06, 0x04);

    return (int8_t)RF_ReadPageReg(1, 0x7E);
}

/**
 * @brief Get the SNR value of the received packet
 * @return SNR value
 * @note This function must be called before the receive interrupt is cleared, because clearing the receive interrupt will clear the SNR register
 * @note SNR value range: -20~10, unit: dB
 */
int32_t RF_GetPktSnr(void)
{
    int32_t PktSnr = 0, SnrVal;
    uint8_t i, Temp[6];
    uint32_t NoiseStrength;
    uint32_t SingalStrength;

    RF_ReadPageRegs(2, 0x71, &Temp[0], 3); // Noise strength
    RF_ReadPageRegs(1, 0x74, &Temp[3], 3); // Singal strength

    SingalStrength = (((uint32_t)Temp[5] << 16) | ((uint32_t)Temp[4] << 8) | Temp[3]);
    NoiseStrength = (((uint32_t)Temp[2] << 16) | ((uint32_t)Temp[1] << 8) | Temp[0]);

    if (NoiseStrength == 0)
    {
        NoiseStrength = 1;
    }

    if(g_RfCfgParams.SpreadingFactor <= 9)
    {
        SnrVal = (SingalStrength << (9 - g_RfCfgParams.SpreadingFactor)) / NoiseStrength;
    }
    else
    {
        SnrVal = (SingalStrength >> (g_RfCfgParams.SpreadingFactor - 9)) / NoiseStrength;
    }

    for (i = 0; i < 31; i++)
    {
        if (SnrVal <= g_SnrLog10Talbe[i])
        {
            PktSnr = (int32_t)i - (int32_t)20;
            break;
        }
    }

    return PktSnr;
}

/**
 * @brief Get the interrupt flag bits
 * @return IRQ flag bits
 *         - 0x00: No interrupt
 *         - 0x01: RF_IRQ_TX_DONE
 *         - 0x02: RF_IRQ_RX_TIMEOUT
 *         - 0x04: RF_IRQ_CRC_ERR
 *         - 0x08: RF_IRQ_RX_DONE
 *         - 0x40: RF_IRQ_MAPM_DONE
 */
uint8_t RF_GetIRQFlag(void)
{
    return (RF_ReadPageReg(0, 0x6C) & 0x7F);
}

/**
 * @brief Clear the interrupt flag bits
 * @param <IRQFlag> Interrupt flag bits
 */
void RF_ClrIRQFlag(uint8_t IRQFlag)
{
    RF_WritePageReg(0, 0x6C, IRQFlag);
}

/**
 * @brief Get the current frequency setting
 */
uint32_t RF_GetFreq(void)
{
    return g_RfCfgParams.Frequency;
}

/**
 * @brief Get the current IQ inversion setting
 */
RfIQModes_t RF_GetInvertIQ(void)
{
    return g_RfCfgParams.InvertIQ;
}

/**
 * @brief Get the current preamble length setting
 */
uint16_t RF_GetPreamLen(void)
{
    return g_RfCfgParams.PreambleLen;
}

/**
 * @brief Get the current transmit power setting
 */
uint8_t RF_GetTxPower(void)
{
    return g_RfCfgParams.TxPower;
}

/**
 * @brief Get the current bandwidth setting
 */
uint8_t RF_GetBandWidth(void)
{
    return g_RfCfgParams.Bandwidth;
}

/**
 * @brief Get the current spreading factor setting
 */
uint8_t RF_GetSF(void)
{
    return g_RfCfgParams.SpreadingFactor;
}

/**
 * @brief Get the current CRC mode setting
 */
uint8_t RF_GetCRC(void)
{
    return g_RfCfgParams.CrcMode;
}

/**
 * @brief Get the current coding rate setting
 */
uint8_t RF_GetCR(void)
{
    return g_RfCfgParams.CodingRate;
}

/**
 * @brief Get the current sync word setting
 */
uint8_t RF_GetSyncWord(void)
{
    return g_RfCfgParams.SyncWord;
}

/**
 * @brief Get the current low data rate optimize setting
 */
uint8_t RF_GetLDR(void)
{
    return g_RfCfgParams.LowDatarateOptimize;
}

/**
 * @brief Get the current symbol time setting
 * @param <bw> Bandwidth
 *           - RF_BW_062K / RF_BW_125K / RF_BW_250K / RF_BW_500K
 * @param <sf> Spreading factor
 *           - RF_SF5 / RF_SF6 / RF_SF7 / RF_SF8 / RF_SF9 / RF_SF10 / RF_SF11 / RF_SF12
 * @return Symbol time, unit: us
 * @note This function is used to calculate the symbol time
 */
uint32_t RF_GetOneSymbolTime(uint8_t bw, uint8_t sf)
{
    const uint32_t BwTable[4] = {62500, 125000, 250000, 500000};

    if(bw < RF_BW_062K || bw > RF_BW_500K)
    {
        return 0;
    }

    return (1000000 * (1 << sf) / BwTable[bw - RF_BW_062K]);
}

/**
 * @brief Get the current transmit time setting
 * @param <Size> Transmit payload size, unit: byte
 * @return Transmit time, unit: ms
 */
uint32_t RF_GetTxTimeMs(uint8_t Size)
{
    uint8_t sf, cr, bw, ldr;
    uint16_t PreambleLen; /* Preamble length */
    float SymbolTime;     /* Symbol time:ms */
    float PreambleTime;   /* Preamble time:ms */
    float PayloadTime;    /* Payload time:ms */
    float TotalTime;      /* Total time:ms */
    const float BwTable[4] = {62.5, 125, 250, 500};

    sf = RF_GetSF();
    cr = RF_GetCR();
    bw = RF_GetBandWidth();
    ldr = RF_GetLDR();
    PreambleLen = RF_GetPreamLen();
    
    SymbolTime = (float)(1 << sf) / BwTable[bw - RF_BW_062K]; /* Symbol time: ms */

    if (sf < 7)
    {
        PreambleTime = (PreambleLen + 6.25f) * SymbolTime;
        PayloadTime = ceil((float)(Size * 8 - sf * 4 + 36) / ((sf - ldr * 2) * 4));
    }
    else
    {
        PreambleTime = (PreambleLen + 4.25f) * SymbolTime;
        PayloadTime = ceil((float)(Size * 8 - sf * 4 + 44) / ((sf - ldr * 2) * 4));
    }

    TotalTime = PreambleTime + (PayloadTime * (cr + 4) + 8) * SymbolTime;

    if(TotalTime < 1)
    {
        TotalTime = 1; /* 小于1ms时，按1ms处理 */
    }

    return (uint32_t)TotalTime;
}

/**
 * @brief Enable mapm mode
 */
void RF_EnableMapm(void)
{
    RF_SetPageRegBits(1, 0x38, 0x01);      /* Enable mapm mode */
    RF_WritePageRegBits(0, 0x58, 0, 0x40); /* Enable mapm interrupt */
}

/**
 * @brief Disable mapm mode
 */
void RF_DisableMapm(void)
{
    RF_ResetPageRegBits(1, 0x38, 0x01);    /* Disable mapm mode */
    RF_WritePageRegBits(0, 0x58, 1, 0x40); /* Disable mapm interrupt */
}

/**
 * @brief Set the mapm configuration
 * @param <pMapmCfg>
 *         fn: Number of fields in mapm, total number of fields sent is fn*(2^fnm) (usually fnm=0)
 *         fnm: Number of times the same field is repeated (default fnm=0)
 *              - fnm=0时，表示同一个field发送1次，总共发送fn*1个fields
 *              - fnm=1时，表示同一个field发送2次，总共发送fn*2个fields
 *              - fnm=2时，表示同一个field发送4次，总共发送fn*4个fields
 *              - fnm=3时，表示同一个field发送8次，总共发送fn*8个fields
 *         gfs: Last group function selection
 *              - gfs=0时，表示最后一个group的载荷功能为地址
 *              - gfs=1时，表示最后一个group的载荷功能为剩余field的个数(包含当前field)
 *         gn: 一个field中group的个数
 *         pg1: field中第一个group的前导码个数,范围为8~255
 *              - pg1=8时，表示第一个group中有8个前导码
 *              - pg1=200时，表示第一个group中有200个前导码
 *         pgn: field中除第一个group外的其它group的前导码个数,范围为0~255
 *              - pgn=8时，表示第一个group中有8个前导码
 *              - pgn=200时，表示第一个group中有200个前导码
 *         pn: 数据包的前导码个数,范围为1~65535
 * @note group中的地址或计数值占用2个chirp.
 */
void RF_ConfigMapm(RF_MapmCfg_t *pMapmCfg)
{
    uint8_t reg_fn, fn_h, fn_l;

    fn_h = pMapmCfg->fn / 15 + 1;
    fn_l = pMapmCfg->fn % 15 + 1;
    reg_fn = (fn_h << 4) + fn_l;
    RF_WritePageReg(1, 0x3D, reg_fn);                            /* set the number of fields */
    RF_WritePageRegBits(1, 0x37, pMapmCfg->fnm, 0x80 | 0x40);    /* set the unit code word of the field counter represents several fields */
    RF_WritePageRegBits(1, 0x38, pMapmCfg->gfs, 0x02);           /* set the last group function selection */
    RF_WritePageRegBits(1, 0x38, pMapmCfg->gn - 1, 0x08 | 0x04); /* set the number of groups in Field */
    RF_WritePageReg(1, 0x3B, pMapmCfg->pg1);                     /* set the number of Preambles in first groups */

    /* set the number of preambles for groups other than the first group */
    RF_WritePageReg(1, 0x3C, pMapmCfg->pgn);

    /* set the number of preamble between the last group and the sync word */
    RF_WritePageRegBits(1, 0x39, (uint8_t)(pMapmCfg->pn >> 8), 0x0F);
    RF_WritePageReg(1, 0x3A, (uint8_t)(pMapmCfg->pn));
}

/**
 * @brief Set the mapm address
 * @param <MapmAddr> mapm group address
 *        <AddrWidth> Address width, range: 1~4
 * @note The MapmAddr[0] of the receiver must be the same as that of the sender,
 *       otherwise the receiver will not trigger the mapm interrupt.
 * @note Mapm address register description:
 *       [Page1][Reg0x3E] is MapmAddr[0]
 *       [Page1][Reg0x3F] is MapmAddr[1]
 *       [Page1][Reg0x40] is MapmAddr[2]
 *       [Page1][Reg0x41] is MapmAddr[3]
 */
void RF_SetMapmAddr(uint8_t *MapmAddr, uint8_t AddrWidth)
{
    RF_WritePageRegs(1, 0x3E, MapmAddr, AddrWidth);
}

/**
 * @brief Calculate the time it takes to send one field (ms)
 * @param <pMapmCfg> mapm configuration parameters
 *        <SymbolTime> Time of one symbol (chirp)
 * @note Group1 in chirp number is (pg1 + 2), where pg1 is the number of preambles in Group1, and 2 is the number of chirps occupied by the address in Group1.
 * @note The number of chirps in other groups is (pgn + 2) * (gn - 1), where pgn is the number of preambles in other single groups,
 *       and 2 is the number of chirps occupied by the address (or count value) in other single groups.
 *       (gn-1) is the number of groups remaining after removing Group1.
 */
uint32_t RF_GetMapmOneFieldTime(RF_MapmCfg_t *pMapmCfg, uint32_t SymbolTime)
{
    uint8_t pgn = pMapmCfg->pgn;
    uint8_t pg1 = pMapmCfg->pg1;
    uint8_t gn = pMapmCfg->gn;

    uint16_t ChirpNumInOneField = (pg1 + 2) + (pgn + 2) * (gn - 1);
    return ChirpNumInOneField * SymbolTime / 1000;
}

/**
 * @brief Calculate the remaining Mapm time
 * @param <pMapmCfg> mapm configuration parameters
 *        <SymbolTime> Time of one symbol (chirp)
 * @return Remaining Mapm time, unit: ms
 * @note The remaining Mapm time is the time from the current moment to the completion of the remaining fields.
 * @note The remaining field count includes the current field.
 */
uint32_t RF_GetLeftMapmTime(RF_MapmCfg_t *pMapmCfg, uint32_t SymbolTime)
{
    uint8_t fnm, gn, pgn, pg1, fn;
    uint16_t ChirpNumInOneField;
    uint16_t NumberOfLeftChirps;
    uint32_t LeftMapmTime;

    pgn = pMapmCfg->pgn;
    pg1 = pMapmCfg->pg1;
    gn = pMapmCfg->gn;
    fnm = pMapmCfg->fnm;
    fn = pMapmCfg->fn;
    
    /**
     * @brief Calculate the number of chirps in one field
     * @note Group1 in chirp number is (pg1 + 2), where pg1 is the number of preambles in Group1, and 2 is the number of chirps occupied by the address in Group1.
     * @note The number of chirps in other groups is (pgn + 2) * (gn - 1), where pgn is the number of preambles in other single groups,
     *       and 2 is the number of chirps occupied by the address (or count value) in other single groups.
     *       (gn-1) is the number of groups remaining after removing Group1.
     */
    ChirpNumInOneField = (pg1 + 2) + (pgn + 2) * (gn - 1);
    
    /**
     * @brief Calculate the number of remaining chirps
     * @note fn is the number of fields in mapm, if fnm > 0, the actual number of fields sent is fn*(2^fnm).
     * @note pn is the number of preambles between the last group and the sync word, and there are pn chirps in the air.
     * @note Subtracting the number of chirps in one field is to subtract the time spent on the current field.
     */
    NumberOfLeftChirps = (1 << fnm) * fn * ChirpNumInOneField - ChirpNumInOneField;
    
    /* Calculate the remaining Mapm time */
    LeftMapmTime = SymbolTime * NumberOfLeftChirps;

    return LeftMapmTime / 1000; /* 微秒转毫秒 */
}

/**
 * @brief Start sending continuous carrier
 * @note Before calling this function, you need to set the transmission power and frequency.
 * @note After calling this function, the chip will continue to transmit until RF_StopTxContinuousWave() is called to stop transmission.
 */
void RF_StartTxContinuousWave(void)
{
    RF_WriteReg(0x02, RF_STATE_STB3);   /* Set chip to idle state */
    RF_WritePageReg(0, 0x58, 0x00);     /* Disable all RF interrupts */
    RF_SetTxMode(RF_TX_MODE_CONTINOUS); /* Set continuous transmission mode */
    RF_WritePageReg(1, 0x0C, 1);        /* Set the length of transmitted data to 1 byte */
    RF_TurnonPA();                      /* Turn on PA before transmitting data */
    RF_WriteReg(0x02, RF_STATE_TX);     /* Set chip to transmit state */
    RF_WriteReg(0x01, 0xFF);            /* Write 0xFF to FIFO register, then start transmitting continuous carrier */
    g_RfOperatetate = RF_STATE_TX;
}

/**
 * @brief Stop sending continuous carrier
 * @note After calling this function, the chip will stop transmitting and enter idle state.
 */
void RF_StopTxContinuousWave(void)
{
    RF_WriteReg(0x02, RF_STATE_STB3); /* Set chip to idle state */
    RF_TurnoffPA();                   /* Turn off PA after transmitting */
    RF_WritePageReg(0, 0x58, 0x0F);   /* Restore RF default interrupts */
    g_RfOperatetate = RF_STATE_STB3;
}

/**
 * @brief Process RF interrupt events
 * @note This function can be called in interrupt service function or polling mode.
 */
void RF_IRQ_Process(void)
{
    if (CHECK_RF_IRQ()) /* Detect RF interrupt, high level indicates an interrupt */
    {
        uint8_t IRQFlag;

        IRQFlag = RF_GetIRQFlag();    /* Get interrupt flags */
        if (IRQFlag & RF_IRQ_TX_DONE) /* Transmit done interrupt */
        {
            RF_TurnoffPA();                /* Turn off PA after transmitting */
            RF_ClrIRQFlag(RF_IRQ_TX_DONE); /* Clear transmit done interrupt flag */
            IRQFlag &= ~RF_IRQ_TX_DONE;
        }
        if (IRQFlag & RF_IRQ_RX_DONE) /* Receive done */
        {
            g_RfRxPkt.Snr = RF_GetPktSnr();   /* Get the SNR value of the received packet */
            g_RfRxPkt.Rssi = RF_GetPktRssi(); /* Get the RSSI value of the received packet */
            
            /* Get received data and length */
            g_RfRxPkt.RxLen = RF_GetRecvPayload((uint8_t *)g_RfRxPkt.RxBuf);

            RF_ClrIRQFlag(RF_IRQ_RX_DONE); /* Clear receive done interrupt flag */
            IRQFlag &= ~RF_IRQ_RX_DONE;
        }
        if (IRQFlag & RF_IRQ_MAPM_DONE) /* mapm receive done interrupt */
        {
            uint8_t MapmAddr = RF_ReadPageReg(0, 0x6E);
            g_RfRxPkt.MapmRxBuf[g_RfRxPkt.MapmRxIndex++] = MapmAddr;

            RF_ClrIRQFlag(RF_IRQ_MAPM_DONE); /* Clear mapm receive done interrupt flag */
            IRQFlag &= ~RF_IRQ_MAPM_DONE;
        }
        if (IRQFlag & RF_IRQ_CRC_ERR) /* CRC error */
        {
            RF_ClrIRQFlag(RF_IRQ_CRC_ERR); /* Clear CRC error interrupt flag */
            IRQFlag &= ~RF_IRQ_CRC_ERR;
        }
        if (IRQFlag & RF_IRQ_RX_TIMEOUT) /* Receive timeout */
        {
            /* rf_refresh(); */
            IRQFlag &= ~RF_IRQ_RX_TIMEOUT;
            RF_ClrIRQFlag(RF_IRQ_RX_TIMEOUT); /* Clear receive timeout interrupt flag */
        }

        if (IRQFlag)
        {
            RF_ClrIRQFlag(IRQFlag); /* Clear unprocessed interrupt flags */
        }
    }
}
