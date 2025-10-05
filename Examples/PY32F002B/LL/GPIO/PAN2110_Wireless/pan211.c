/**************************************************************************
 * @file      pan211.c
 * @version   V2.2.3
 * $Revision: 1 $
 * $Date:     2025/06/14 $
 * @brief     pan211 driver API definition
 * @code        ___               ___  _     _
 *             |  _ \ __ _ _ __  / ___| |__ (_)_ __
 *             | |_) / _` | '_ \| |   | '_ \| | '_ \
 *             |  __/ (_| | | | | |___| | | | | |_) |
 *             |_|   \__,_|_| |_|\____|_| |_|_| .__/
 *                                            |_|
 *              (C)2009-2025 PanChip
 * @endcode
 * @author    PanChip
 * @note
 * Copyright (C) 2025 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/

/*-----------------------------------------------------------------------------------------------
 *                                   Configuration Parameters
 *-----------------------------------------------------------------------------------------------
 *   Name                 | Value                                   | Description
 * -----------------------|-----------------------------------------|----------------------------
 *   XTAL_FREQ            | XTAL_FREQ_16M                           | XTAL frequency defined by PAN211_XTAL_FREQ
 *   EN_AGC               | 1                                       | Whether to enable AGC
 *   INTERFACE_MODE       | USE_SPI_3LINE                           | Interface mode
 *   ChipMode             | PAN211_CHIPMODE_XN297                   | Chip operating mode
 *   WorkMode             | PAN211_WORKMODE_NORMAL                  | Operating mode
 *   TxMode               | PAN211_TX_MODE_SINGLE                   | Transmission mode
 *   RxMode               | PAN211_RX_MODE_CONTINOUS                | Reception mode
 *   TxPower              | PAN211_TXPWR_9dBm                       | Transmission power
 *   Channel              | 78                                      | RF channel
 *   DataRate             | PAN211_DR_1Mbps                         | Data rate
 *   EnWhite              | 1                                       | Whether to enable whitening
 *   Endian               | PAN211_ENDIAN_BIG                       | Endian mode
 *   Crc                  | PAN211_CRC_2byte                        | CRC check method
 *   crcSkipAddr          | 0                                       | Whether CRC skips address
 *   TxLen                | 32                                      | Transmission data length
 *   RxLen                | 32                                      | Reception data length
 *   RxTimeoutUs          | 2000                                    | Reception timeout (microseconds)
 *   TRxDelayTimeUs       | 0                                       | Transmit/receive delay time (microseconds), valid only in enhanced mode
 *   AutoDelayUs          | 0                                       | Auto retransmission delay time (microseconds), valid only in enhanced mode
 *   AutoMaxCnt           | 0                                       | Maximum auto retransmission count, valid only in enhanced mode
 *   EnDPL                | 0                                       | Whether to enable dynamic payload length, valid only in enhanced mode
 *   EnManuPid            | 0                                       | Whether to use manual packet ID, valid only in enhanced mode
 *   EnRxPlLenLimit       | 0                                       | Whether to limit reception payload length, valid only in enhanced mode
 *   EnTxNoAck            | 1                                       | Whether transmission requires no ACK, valid only in enhanced mode
 *   TxAddrWidth          | 5                                       | Transmission address width
 *   TxAddr               | [0xCC, 0xCC, 0xCC, 0xCC, 0xCC]          | Transmission address
 *   RxAddrWidth          | 5                                       | Reception address width
 *   RxAddr               |                                         | Reception channel enable status and addresses
 *                        | True, [0xcc, 0xcc, 0xcc, 0xcc, 0xcc]    | Pipe0 reception channel enable status and address
 *                        | False, [0xc1, 0xcc, 0xcc, 0xcc, 0xcc]   | Pipe1 reception channel enable status and address
 *                        | False, [0xc2, 0xcc, 0xcc, 0xcc, 0xcc]   | Pipe2 reception channel enable status and address
 *                        | False, [0xc3, 0xcc, 0xcc, 0xcc, 0xcc]   | Pipe3 reception channel enable status and address
 *                        | False, [0xc4, 0xcc, 0xcc, 0xcc, 0xcc]   | Pipe4 reception channel enable status and address
 *                        | False, [0xc5, 0xcc, 0xcc, 0xcc, 0xcc]   | Pipe5 reception channel enable status and address
 *   IOMUX_EN             | 1                                       | Multiplexed interrupt pin function enable
 *   InterruptMask        | 0xff                                    | Interrupt pin event mask (0: mask, 1: unmask)
 *   PowerTable           |                                         | Configurable power level list
 *                        | PAN211_TXPWR_n40dBm                     |                               
 *                        | PAN211_TXPWR_n23dBm                     |                               
 *                        | PAN211_TXPWR_n10dBm                     |                               
 *                        | PAN211_TXPWR_0dBm                       |                               
 *                        | PAN211_TXPWR_6dBm                       |                               
 *                        | PAN211_TXPWR_9dBm                       |                               
 *   RxGain               | 1                                       | Reception gain
 *   TxDevSelect          | 1                                       | Transmission frequency deviation selection
 *   BLEHead0             | 0x42                                    | BLE header identifier 0
 *   BLEHead1             | 0x00                                    | BLE header identifier 1
 *   BLEHeadNum           | 2                                       | BLE header byte count
 *   LengthFilterMode     | PAN211_BLE_LEN_FILTER_DISABLE           | BLE length filter mode
 *   S2S8Mode             | PAN211_PRIMODE_DIS                      | S2S8 operating mode
 *   WhiteInit            | PAN211_BLE_WH_INIPHA_CH37               | Whitening initialization value
 *   WhiteList            | []                                      | Whitelist addresses
 *   WhiteListOffset      | 0                                       | Whitelist offset
 *   WhiteListLen         | 0                                       | Whitelist length
 *-----------------------------------------------------------------------------------------------*/

#include "main.h"
#include "pan211.h"


/* 
 *   - SPI_CS: push-pull output
 *   - SPI_SCK push-pull output
 *   - SPI_DATA: input or push-pull output, set input pull-up resistor in low power mode
 */
#define SPI_CS_HIGH      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5)
#define SPI_CS_LOW       LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5)
#define SPI_SCK_HIGH     LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0)
#define SPI_SCK_LOW      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0)
#define SPI_DATA_HIGH    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define SPI_DATA_LOW     LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define SPI_DATA_STATUS  LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_1)
#define SPI_DATA_OUTPUT  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT)
#define SPI_DATA_INPUT   LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT)


/**
 * @brief SPI write single byte
 * @param Value Single byte data to write
 * @note This interface is implemented with software SPI, specific implementation may vary by MCU platform.
 * @note Before calling this function, must call SPI_CS_LOW function to pull CS pin low.
 * @note This function will set SPI_DATA pin to output mode, and set it to input mode after transmission completes.
 * @note PAN211 SPI interface uses 3-wire SPI mode.
 * @note PAN211 SPI configuration:
 *       Clock polarity: active low
 *       Clock phase: sample data on first edge
 *       Data transfer order: MSB first
 * @note Taking sending 0xCC data as example, timing diagram as follows:
 *      SPI_CS:  ____________________________________________________
 *      SPI_CLK:  ____|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__
 *      SPI_DATA: ___|‾‾‾‾‾‾‾‾‾‾|___________|‾‾‾‾‾‾‾‾‾‾‾|_____________
 *      BIT DATA:     1     1     0     0      1     1      0     0
 */
void SPI_WriteByte(uint8_t Value)
{
    uint8_t i;

    SPI_DATA_OUTPUT;
    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW;
        __NOP();
        if (Value & 0x80)
        {
            SPI_DATA_HIGH;
        }
        else
        {
            SPI_DATA_LOW;
        }
        Value <<= 1;
        __NOP();
        SPI_SCK_HIGH;
        __NOP();
    }
    SPI_DATA_INPUT;
    SPI_SCK_LOW;
}

/**
 * @brief SPI read single byte
 * @return uint8_t Read data byte
 * @note This interface is implemented with software SPI, specific implementation may vary by MCU platform.
 * @note Before calling this function, must call SPI_CS_LOW function to pull CS pin low.
 * @note This function will set SPI_DATA pin to output mode, and set it to input mode after transmission completes.
 * @note PAN211 SPI interface uses 3-wire SPI mode.
 * @note PAN211 SPI configuration:
 *       Clock polarity: active low
 *       Clock phase: sample data on first edge
 *       Data transfer order: MSB first
 * @note Taking sending 0x33 data as example, timing diagram as follows:
 *      SPI_CS:  ___________________________________________________
 *      SPI_CLK:  ___|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__
 *      SPI_DATA: _____________|‾‾‾‾‾‾‾‾‾‾‾|___________|‾‾‾‾‾‾‾‾‾‾‾‾‾
 *      BIT DATA:    0     0     1     1     0     0     1     1
 */
uint8_t SPI_ReadByte(void)
{
    uint8_t i;
    uint8_t Value = 0;

    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW;
        Value <<= 1;
        __NOP();
        SPI_SCK_HIGH;
        __NOP();
        if (SPI_DATA_STATUS)
        {
            Value |= 0x01;
        }
    }
    SPI_SCK_LOW;
    return Value;
}

/**
 * @brief When DATA pin multiplexed interrupt function is enabled, detect if DATA pin has interrupt event during SPI idle period, active low.
 * @note Method to enable DATA pin multiplexed interrupt function is to set IRQ_DATA_MUX_EN (Page0 0x03[2]) to 1.
 * @return 1: Interrupt triggered
 * @return 0: No interrupt triggered
 */
uint8_t IRQDetected(void)
{
    return SPI_DATA_STATUS == 0; /* SPI_DATA引脚状态为低电平表示IRQ触发 */
}

/**
 * @brief Write single byte to specified register
 * @param Addr Register address to write
 * @param Value Single byte data to write to register
 * @return None
 */
void PAN211_WriteReg(uint8_t Addr, uint8_t Value)
{
    SPI_CS_LOW;
    SPI_WriteByte(((Addr << 1) | 0x01)); /* BIT7~1 register address, BIT0=1 indicates write operation */
    SPI_WriteByte(Value);
    SPI_CS_HIGH;
}

/**
 * @brief Read single byte from specified register
 * @param Addr Register address to read
 * @return uint8_t Value read from register
 */
uint8_t PAN211_ReadReg(uint8_t Addr)
{
    uint8_t Value;
    SPI_CS_LOW;
    SPI_WriteByte((Addr << 1)&0xFE);   /* BIT7~1 register address, BIT0=0 indicates read operation */
    Value = SPI_ReadByte();
    SPI_CS_HIGH;
    return Value;
}

/**
 * @brief Continuously write multiple PAN211 register values
 * @param Addr Starting register address
 * @param Buffer Buffer containing data to write
 * @param Len Number of bytes to write
 */
void PAN211_WriteRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Len)
{
    SPI_CS_LOW;
    SPI_WriteByte(((Addr << 1) | 0x01));
    for (int i = 0; i < Len; i++)
    {
        SPI_WriteByte(Buffer[i]);
    }
    SPI_CS_HIGH;
}

/**
 * @brief Continuously read multiple PAN211 register values
 * @param Addr Starting register address
 * @param Buffer Buffer to store read data
 * @param Len Number of bytes to read
 */
void PAN211_ReadRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Len)
{
    SPI_CS_LOW;                        /* 拉低CS引脚，开始SPI传输 */
    SPI_WriteByte((Addr << 1) & 0xFE); /* BIT7~1寄存器地址, BIT0=0表示读操作 */
    for (int i = 0; i < Len; i++)
    {
        Buffer[i] = SPI_ReadByte();    /* 循环读取寄存器数据到Buffer中 */
    }
    SPI_CS_HIGH;                       /* 拉高CS引脚，结束SPI传输 */
}

/**
 * @brief Write data to PAN211 TX FIFO
 * @param Buffer Data buffer to write to FIFO
 * @param Size Number of bytes of data to write
 */
void PAN211_WriteFIFO(uint8_t *Buffer, uint8_t Size)
{
    PAN211_WriteRegs(PAN211_P0_TRX_FIFO, Buffer, Size);
}

/**
 * @brief Read data from data FIFO
 * @param Buffer Buffer to store data read from FIFO
 * @param Size Number of bytes to read
 */
void PAN211_ReadFIFO(uint8_t *Buffer, uint8_t Size)
{
    PAN211_ReadRegs(PAN211_P0_TRX_FIFO, Buffer, Size);
}

/* ========================================================================== */
/*                PAN211 Initialization Function (Auto-generated by tool)     */
/* ========================================================================== */

/**
 * @brief Initialize PAN211 transceiver to STB3 state after power-on
 * @return Whether initialization successful, 1 indicates success, 0 indicates failure
 * @note The following code is auto-generated by tool
 */
uint8_t PAN211_Init(uint8_t rf_channel)
{
    uint8_t Value_2, Value_4;

    /* 1. SPI or IIC initialization */
    PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    PAN211_WriteReg(PAN211_P0_SPI_CFG, 0x83);

    /* 2. STB3 mode */
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x04);
    LL_mDelay(1);
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x74);
    LL_mDelay(1);
    PAN211_WriteReg(PAN211_P0_SYS_CFG, 0x00); /* soft reset */
    LL_mDelay(1);
    PAN211_WriteReg(PAN211_P0_SYS_CFG, 0x02); /* release soft reset */
#if (PAN211_XTAL_FREQ == 0)
    PAN211_WriteReg(0x37, 0xE0); // 16MHz XTAL
#endif

    /* 3. read factory configurations */
    PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
    PAN211_WriteReg(0x05, 0x00);
    PAN211_WriteReg(0x04, 0x04);
    Value_2 = PAN211_ReadReg(0x04);
    PAN211_WriteReg(0x04, 0x08);
    Value_4 = PAN211_ReadReg(0x04);
    PAN211_WriteReg(0x05, 0x01);
    printf("%02X %02X\r\n", Value_2, Value_4);
    if ((Value_2 & 0x0F) != 1)
    {
        return 0; // failed
    }
    PAN211_WriteReg(0x47, 0x83 | ((Value_2 >> 1) & 0x70));
    PAN211_WriteReg(0x43, 0x10 | (!(Value_2 & 0x10)));

    /* 4. write pre-defined configurations to page1 registers */
#if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x27, 0x8A);
        PAN211_WriteReg(0x32, 0x1E);
        PAN211_WriteReg(0x33, 0x19);
        PAN211_WriteReg(0x37, 0x15);
        PAN211_WriteReg(0x3A, 0x14);
        PAN211_WriteReg(0x3E, 0xF1);
    #if (PAN211_XTAL_FREQ == 0)
        PAN211_WriteReg(0x3F, 0xD2);
        PAN211_WriteReg(0x40, 0x20);
        PAN211_WriteReg(0x41, 0xA6);
    #else
        PAN211_WriteReg(0x41, 0xA2);
    #endif
        PAN211_WriteReg(0x46, 0xB0);
        PAN211_WriteReg(0x4C, 0x48);
#elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x0A, 0x48);
        PAN211_WriteReg(0x27, 0x8A);
        PAN211_WriteReg(0x32, 0x10);
        PAN211_WriteReg(0x33, 0x1C);
        PAN211_WriteReg(0x37, 0x15);
        PAN211_WriteReg(0x3A, 0x54);
        PAN211_WriteReg(0x3E, 0xF1);
    #if (PAN211_XTAL_FREQ == 0)
        PAN211_WriteReg(0x3F, 0xD2);
        PAN211_WriteReg(0x40, 0x20);
        PAN211_WriteReg(0x41, 0xA6);
    #else
        PAN211_WriteReg(0x41, 0xA2);
    #endif
        PAN211_WriteReg(0x46, 0xB0);
        PAN211_WriteReg(0x49, 0x44);
        PAN211_WriteReg(0x4C, 0x48);
#else // 2Mbps
    // TODO
#endif
    
    /* 5. write pre-defined configurations to page0 registers */
    PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    PAN211_WriteReg(0x05, (Value_4 >> 4) | 0xC0);
    PAN211_WriteReg(PAN211_P0_SYS_CFG, 0x06); 
    PAN211_WriteReg(PAN211_P0_WMODE_CFG0, 0x89); /* [7:6]Crc [5:4]WORK_MODE [3]EnWhiten [2]crcSkipAddr [1]EnTxNoAck [0]Endian */
    PAN211_WriteReg(PAN211_P0_WMODE_CFG1, 0xa3); /* [4]EnDPL [3]ENHANCE [1:0]AddrWidth */
    PAN211_WriteReg(PAN211_P0_RX_PLLEN_CFG, 0x20); /* [7:0]RxLen */
    PAN211_WriteReg(PAN211_P0_TX_PLLEN_CFG, 0x20); /* [7:0]TxLen */
    // PAN211_WriteReg(PAN211_P0_RFIRQ_CFG, 0x00); /* [7:0]InterruptMask */
    // PAN211_WriteReg(PAN211_P0_TRXTWTL_CFG, 0x00); /* [7:0]TRxDelayTimeUs[7:0] */
    // PAN211_WriteReg(PAN211_P0_TRXTWTH_CFG, 0x00); /* [6:0]TRxDelayTimeUs[14:8] */
    // PAN211_WriteReg(PAN211_P0_PIPE0_RXADDR0_CFG, 0xcc); /* Pipe0-RxAddr[7:0] */
    // PAN211_WriteReg(PAN211_P0_PIPE0_RXADDR1_CFG, 0xcc); /* Pipe0-RxAddr[15:8] */
    // PAN211_WriteReg(PAN211_P0_PIPE0_RXADDR2_CFG, 0xcc); /* Pipe0-RxAddr[23:16] */
    // PAN211_WriteReg(PAN211_P0_PIPE0_RXADDR3_CFG, 0xcc); /* Pipe0-RxAddr[31:24] */
    // PAN211_WriteReg(PAN211_P0_PIPE0_RXADDR4_CFG, 0xcc); /* Pipe0-RxAddr[39:32] */
    // PAN211_WriteReg(PAN211_P0_TXADDR0_CFG, 0xcc); /* TxAddr[7:0] */
    // PAN211_WriteReg(PAN211_P0_TXADDR1_CFG, 0xcc); /* TxAddr[15:8] */
    // PAN211_WriteReg(PAN211_P0_TXADDR2_CFG, 0xcc); /* TxAddr[23:16] */
    // PAN211_WriteReg(PAN211_P0_TXADDR3_CFG, 0xcc); /* TxAddr[31:24] */
    // PAN211_WriteReg(PAN211_P0_TXADDR4_CFG, 0xcc); /* TxAddr[39:32] */
    // PAN211_WriteReg(PAN211_P0_PKT_EXT_CFG, 0x00); /* [6]HDR_LEN_EXIST[5:4]BLEHeadNum */
    // PAN211_WriteReg(PAN211_P0_WHITEN_CFG, 0x7f); /* [6:0]WhiteInitVal */
    // PAN211_WriteReg(PAN211_P0_TXHDR0_CFG, 0x00); /* [7:0]BLE Header */
    // PAN211_WriteReg(PAN211_P0_RXPIPE_CFG, 0x01); /* [5]EnRxPipe5 [4]EnRxPipe4 [3]EnRxPipe3 [2]EnRxPipe2 [1]EnRxPipe1 [0]EnRxPipe0 */
    // PAN211_WriteReg(PAN211_P0_PIPE1_RXADDR0_CFG, 0xcc); /* Pipe1-RxAddr[7:0] */
    // PAN211_WriteReg(PAN211_P0_PIPE1_RXADDR1_CFG, 0xcc); /* Pipe1-RxAddr[15:8] */
    // PAN211_WriteReg(PAN211_P0_PIPE1_RXADDR2_CFG, 0xcc); /* Pipe1-RxAddr[23:16] */
    // PAN211_WriteReg(PAN211_P0_PIPE1_RXADDR3_CFG, 0xcc); /* Pipe1-RxAddr[31:24] */
    // PAN211_WriteReg(PAN211_P0_PIPE1_RXADDR4_CFG, 0xcc); /* Pipe1-RxAddr[39:32] */
    // PAN211_WriteReg(PAN211_P0_PIPE2_RXADDR0_CFG, 0xcc); /* Pipe2-RxAddr[7:0] */
    // PAN211_WriteReg(PAN211_P0_PIPE3_RXADDR0_CFG, 0xcc); /* Pipe3-RxAddr[7:0] */
    // PAN211_WriteReg(PAN211_P0_PIPE4_RXADDR0_CFG, 0xcc); /* Pipe4-RxAddr[7:0] */
    // PAN211_WriteReg(PAN211_P0_PIPE5_RXADDR0_CFG, 0xcc); /* Pipe5-RxAddr[7:0] */
    PAN211_WriteReg(PAN211_P0_TXAUTO_CFG, 0x00); /* [7:4]AutoDelayUs [3:0]AutoMaxCnt */
    PAN211_WriteReg(PAN211_P0_TRXMODE_CFG, 0x41); /* [7]TxMode [6:5]RxMode */
    // PAN211_WriteReg(PAN211_P0_RXTIMEOUTL_CFG, 0xd0); /* [7:0]RxTimeoutUs[7:0] */
    // PAN211_WriteReg(PAN211_P0_RXTIMEOUTH_CFG, 0x07); /* [7:0]RxTimeoutUs[15:8] */
    // PAN211_WriteReg(PAN211_P0_BLEMATCH_CFG0, 0x00); /* [6:4]BLEWhiteListMatchMode [3:2]BLELengthFilterMode */
    // PAN211_WriteReg(PAN211_P0_WLIST0_CFG, 0x00); /* BLEWhiteList[7:0] */
    // PAN211_WriteReg(PAN211_P0_WLIST1_CFG, 0x00); /* BLEWhiteList[15:8] */
    // PAN211_WriteReg(PAN211_P0_WLIST2_CFG, 0x00); /* BLEWhiteList[23:16] */
    // PAN211_WriteReg(PAN211_P0_WLIST3_CFG, 0x00); /* BLEWhiteList[31:24] */
    // PAN211_WriteReg(PAN211_P0_WLIST4_CFG, 0x00); /* BLEWhiteList[39:32] */
    // PAN211_WriteReg(PAN211_P0_WLIST5_CFG, 0x00); /* BLEWhiteList[47:40] */
    // PAN211_WriteReg(PAN211_P0_BLEMATCHSTART_CFG, 0x07); /* [5:0]BLEWhiteListOffset */
#if (PAN211_DATA_RATE == 0) // 1Mbps
    PAN211_WriteReg(PAN211_P0_RF_DATARATE_CFG, 0x45); /* [5:4]DataRate */
#elif (PAN211_DATA_RATE == 3) // 250Kbps
    PAN211_WriteReg(PAN211_P0_RF_DATARATE_CFG, 0xb0); /* [5:4]DataRate */
    #if (PAN211_XTAL_FREQ == 0) // 16M XTAL
    PAN211_WriteReg(0x37, 0xeb);
    #else                       // 32M XTAL
    PAN211_WriteReg(0x37, 0x6b);
    #endif
    PAN211_WriteReg(0x38, 0x4b);
#else
    // TODO
#endif
    PAN211_WriteReg(PAN211_P0_RF_CHANNEL_CFG, 0x55); // RF recalibration
#if (PAN211_DATA_RATE == 0) // 1Mbps
    PAN211_WriteReg(0x43, 0x3a);
#elif (PAN211_DATA_RATE == 3) // 250Kbps
    PAN211_WriteReg(0x43, 0x3b);
#else
#endif
    PAN211_WriteReg(0x44, 0x8c); 
    PAN211_WriteReg(0x55, 0xdd); 
    PAN211_WriteReg(0x56, 0xc9); 
    PAN211_WriteReg(0x57, 0xb7); 
    PAN211_WriteReg(0x5a, 0x10); 
    PAN211_WriteReg(0x5b, 0xfd); 
    PAN211_WriteReg(0x5c, 0xe9); 
    PAN211_WriteReg(0x5d, 0xd4); // rx gain, 0xd4:1, 0xdc:0(low gain)
    PAN211_WriteReg(0x5e, 0x02); 
    PAN211_WriteReg(0x5f, 0x06); 
    PAN211_WriteReg(0x60, 0x0e); 
    PAN211_WriteReg(0x61, 0x3e); // rx gain, 0x3e:1, 0x23:0(lw gain)
    PAN211_WriteReg(0x66, 0x34); 
    PAN211_WriteReg(0x68, 0x0d); 
    PAN211_WriteReg(0x6e, 0x20); 

    /* 6. RF recalibration */
    PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
    PAN211_WriteReg(0x1B, 0x08);
    while (((PAN211_ReadReg(0x70) & 0x40) != 0x40));// or delay 1ms+
    PAN211_WriteReg(0x1B, 0x10);
    LL_mDelay(55); // delay 55 ms+
    PAN211_WriteReg(0x02, 0x76);
    LL_mDelay(0);  // delay 200us+
    PAN211_WriteReg(0x1B, 0x20);
    while (((PAN211_ReadReg(0x7F) & 0x80) != 0x80)); // or delay 2ms+
    PAN211_WriteReg(0x1B, 0x40);
    while (((PAN211_ReadReg(0x6D) & 0x80) != 0x80)); // or delay 1ms+
    PAN211_WriteReg(0x1B, 0x80);
    while (((PAN211_ReadReg(0x7F) & 0x80) != 0x80)); // or delay 2ms+
    PAN211_WriteReg(0x1B, 0x00);
    PAN211_WriteReg(0x02, 0x74);
    PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    PAN211_WriteReg(PAN211_P0_RFIRQFLG, 0xFF);
    PAN211_WriteReg(PAN211_P0_RF_CHANNEL_CFG, rf_channel); // set RF channel 
    return 1;
}

/**
 * @brief PAN211 transceiver soft reset (SOFT_RSTL)
 * @note This function must be called in STB3 state.
 * @note This function will reset PAN211 transceiver internal logic, must call PAN211_Init function to reinitialize after reset.
 * @note POR_RSTL is responsible for low voltage area register reset, SOFT_RSTL is responsible for low voltage area logic reset.
 */
void PAN211_SoftRst(void)
{
    PAN211_WriteReg(PAN211_P0_SYS_CFG, 0x00);
    LL_mDelay(1); // wait 1ms+
    PAN211_WriteReg(PAN211_P0_SYS_CFG, 0x02); // release
}

/**
 * @brief Enter enhanced mode
 */
void PAN211_SetEnhancedMode(uint8_t enabled)
{
    if (enabled == 0)
    {
        PAN211_WriteReg(PAN211_P0_WMODE_CFG1, 0xa3);    // 1010 0011
                                                        //   ^[5]FIFO_128_EN, set 1 in BLE mode and XN297L normal mode, set 0 in XN297L enhanced mode
                                                        //    ^[4]DPY_EN
                                                        //      ^[3]ENHANCE
        PAN211_WriteReg(PAN211_P0_TXAUTO_CFG, 0x00);    // 0000 0000 [4:7]ARD, auto retry interval, [0:3]ARC, retry times
        PAN211_WriteReg(PAN211_P0_TRXMODE_CFG, 0x41);   // 0100 0001
                                                        //  ^[5:6]REG_RX_CFG_MODE, in normal mode: 0:single, 1:single with timeout, 2:continuous
        PAN211_WriteReg(PAN211_P0_RXTIMEOUTL_CFG, 0xd0);// rx timeout LSB
        PAN211_WriteReg(PAN211_P0_RXTIMEOUTH_CFG, 0x07);// rx timeout MSB
    }
    else
    {
        //PAN211_WriteReg(PAN211_P0_WMODE_CFG0, 0x89);  // 1000 1001
                                                        //        ^[1]TX_NOACK, in enhanced mode, 0:reply ack, 1:don't reply ack
        PAN211_WriteReg(PAN211_P0_WMODE_CFG1, 0x9b);    // 1001 1011
        PAN211_WriteReg(PAN211_P0_TXAUTO_CFG, 0x03);    // 0000:250µs, 0011:retry twice
        PAN211_WriteReg(PAN211_P0_TRXMODE_CFG, 0x01);   // 0000 0001
                                                        //  ^[5:6]REG_RX_CFG_MODE, in enhanced mode: 0: continuous, 1:continuous with timeout
        PAN211_WriteReg(PAN211_P0_RXTIMEOUTL_CFG, 0x00);// rx timeout LSB
        PAN211_WriteReg(PAN211_P0_RXTIMEOUTH_CFG, 0x7d);// rx timeout MSB, 0x7d00 = 32000 us = 32 ms
    }
}

/**
 * @brief Enter sleep state from standby state
 */
void PAN211_EnterSleep(void)
{
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x74);
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x21);
}

/**
 * @brief Exit sleep state and enter standby state
 */
void PAN211_ExitSleep(void)
{
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x22);
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x74);
    LL_mDelay(1);
}

/**
 * @brief Enter standby state
 * @note Before calling this function, please ensure to clear IRQ
 */
void PAN211_EnterStandby(void)
{
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x74);
}

/**
 * @brief Exit transmission state and enter standby state. Before calling this function, please ensure to clear IRQ
 */
void PAN211_ExitTxMode(void)
{
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x74);
}

/**
 * @brief Exit reception state and enter standby state
 * @note Before calling this function, please ensure to clear IRQ
 */
void PAN211_ExitRxMode(void)
{
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x74);
}

/**
 * @brief Enter transmission state
 * @note Will return to standby state first before entering transmission state
 */
void PAN211_TxStart(void)
{
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x74);
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x75);
}

/**
 * @brief Exit reception state and enter standby state, before calling this function, please ensure to clear IRQ
 */
void PAN211_RxStart(void)
{
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x74);
    PAN211_WriteReg(PAN211_P0_STATE_CFG, 0x76);
}

/**
 * @brief Set PAN211 transceiver frequency channel
 * @param Channel Desired RF channel, range from 0 to 83
 * @note Actual frequency will be (2400 + Channel)MHz
 */
void PAN211_SetChannel(uint8_t Channel)
{
    PAN211_WriteReg(PAN211_P0_RF_CHANNEL_CFG, Channel);
}

/**
 * @brief Set PAN211 transceiver address width
 * @param AddrWidth Desired address width, 0x01:3 bytes, 0x10:4 bytes, 0x11:5 bytes
 */
void PAN211_SetAddrWidth(uint8_t AddrWidth)
{
    uint8_t Value;
    Value = PAN211_ReadReg(PAN211_P0_WMODE_CFG1) & ~(0x03);
    Value = Value | (AddrWidth & 0x03);
    PAN211_WriteReg(PAN211_P0_WMODE_CFG1, Value);
}

/**
 * @brief Set static reception address for specified channel
 * @param Pipe Channel to configure address, range from 0 to 5
 * @param Addr Buffer pointer containing address
 * @param Len Address length
 * @note Buffer length must equal transceiver current address width
 * @note For channels [2..5], only write first byte of address, because channels 1-5 share four most significant address bytes
 */
void PAN211_SetRxAddr(uint8_t Pipe, uint8_t *Addr, uint8_t Len)
{
    switch (Pipe)
    {
    case 0:
        PAN211_WriteRegs(PAN211_P0_PIPE0_RXADDR0_CFG, Addr, Len); /* pipe 0 address */
        break;
    case 1:
        PAN211_WriteRegs(PAN211_P0_PIPE1_RXADDR0_CFG, Addr, Len); /* pipe 1 address */
        break;
    case 2:
        PAN211_WriteReg(PAN211_P0_PIPE2_RXADDR0_CFG, Addr[0]); /* pipe 2 address, Addr[1]~Addr[5]与通道1相同 */
        break;
    case 3:
        PAN211_WriteReg(PAN211_P0_PIPE3_RXADDR0_CFG, Addr[0]); /* pipe 3 address, Addr[1]~Addr[5]与通道1相同 */
        break;
    case 4:
        PAN211_WriteReg(PAN211_P0_PIPE4_RXADDR0_CFG, Addr[0]); /* pipe 4 address, Addr[1]~Addr[5]与通道1相同 */
        break;
    case 5:
        PAN211_WriteReg(PAN211_P0_PIPE5_RXADDR0_CFG, Addr[0]); /* pipe 5 address, Addr[1]~Addr[5]与通道1相同 */
        break;
    }
}

/**
 * @brief Set transceiver static transmission address
 * @param Addr Buffer pointer containing address
 * @param Len Address length
 */
void PAN211_SetTxAddr(uint8_t *Addr, uint8_t Len)
{
    PAN211_WriteRegs(PAN211_P0_TXADDR0_CFG, Addr, Len);
}

/**
 * @brief Get channel number from which payload can be read from RX FIFO
 * @return Channel number (0-5), returns 0xFF if RX FIFO is empty
 */
uint8_t PAN211_GetRxPipeNum(void)
{
    return (PAN211_ReadReg(PAN211_P0_STATUS0) >> 4) & 0x07; /* 0x74[6:4]为通道号 */
}

/**
 * @brief Set ACK response channel number
 * @param AckPipeNum ACK channel number to set, range from 0 to 5
 * @note This function is used to set ACK response channel number when receiving data packet
 *       In enhanced mode, receiving end will automatically send ACK response after receiving data packet,
 *       Before sending ACK response, must first get receiving data packet channel number through PAN211_GetRxPipeNum() function,
 *       Then call this function to set ACK response channel number.
 */
void PAN211_SetAckPipeNum(uint8_t AckPipeNum)
{
    PAN211_WriteReg(PAN211_P0_MISC_CFG, AckPipeNum);
}

/**
 * @brief Get pending IRQ flags
 * @return Current status of STATUS register RX_DONE, TX_DONE, RX_TIMEOUT and MAX_RT bits
 */
uint8_t PAN211_GetIRQFlags(void)
{
    return PAN211_ReadReg(PAN211_P0_RFIRQFLG);
}

/**
 * @brief Clear PAN211 transceiver pending IRQ flags
 * @param Flags IRQ flags to clear, can be combination of following values:
 *         - RF_IT_TX_IRQ
 *         - RF_IT_MAX_RT_IRQ
 *         - RF_IT_ADDR_ERR_IRQ
 *         - RF_IT_CRC_ERR_IRQ
 *         - RF_IT_LEN_ERR_IRQ
 *         - RF_IT_PID_ERR_IRQ
 *         - RF_IT_RX_TIMEOUT_IRQ
 *         - RF_IT_RX_IRQ
 */
void PAN211_ClearIRQFlags(uint8_t Flags)
{
    PAN211_WriteReg(PAN211_P0_RFIRQFLG, Flags);
}

/**
 * @brief Set BLE whitening initial value
 * @param Value Whitening initial value
 */
void PAN211_SetWhiteInitVal(uint8_t Value)
{
    uint8_t Temp = PAN211_ReadReg(0x1A);
    PAN211_WriteReg(PAN211_P0_WHITEN_CFG, (Temp & 0x80) | (Value & 0x7F));
}

/**
 * @brief Get received valid data length
 * @return Valid data length
 */
uint8_t PAN211_GetRecvLen(void)
{
    return PAN211_ReadReg(PAN211_P0_STATUS3);
}

/**
 * @brief Set PAN211 transmission power (auto-generated by tool)
 * @param TxPower Transmission power value, use PAN211_TXPWR_* constants
 */
void PAN211_SetTxPower(int8_t TxPower)
{
    switch(TxPower)
    {
    case -40: /* PAN211_TXPWR_n40dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x10);
        PAN211_WriteReg(0x46, 0xb1);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x1a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x1b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x00);
        break;

    case -23: /* PAN211_TXPWR_n23dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x13);
        PAN211_WriteReg(0x46, 0xb1);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x1a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x1b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x10);
        break;

    case -16: /* PAN211_TXPWR_n16dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x13);
        PAN211_WriteReg(0x46, 0xb1);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x1a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x1b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x30);
        break;

    case -14: /* PAN211_TXPWR_n14dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x13);
        PAN211_WriteReg(0x46, 0xb1);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x1a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x1b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x40);
        break;

    case -12: /* PAN211_TXPWR_n12dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x13);
        PAN211_WriteReg(0x46, 0xb1);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x1a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x1b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x50);
        break;

    case -10: /* PAN211_TXPWR_n10dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x13);
        PAN211_WriteReg(0x46, 0xb1);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x1a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x1b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x80);
        break;

    case -5: /* PAN211_TXPWR_n5dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x17);
        PAN211_WriteReg(0x46, 0xbd);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x1a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x1b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0xff);
        break;

    case 0: /* PAN211_TXPWR_0dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x13);
        PAN211_WriteReg(0x46, 0xbd);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x3a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x3b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x84);
        break;

    case 3: /* PAN211_TXPWR_3dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x17);
        PAN211_WriteReg(0x46, 0xb8);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x3a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x3b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x80);
        break;

    case 6: /* PAN211_TXPWR_6dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0xca);
        PAN211_WriteReg(0x3c, 0x17);
        PAN211_WriteReg(0x46, 0xb4);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x3a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x3b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x84);
        break;

    case 9: /* PAN211_TXPWR_9dBm */
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x01);
        PAN211_WriteReg(0x27, 0x8a);
        PAN211_WriteReg(0x3c, 0x17);
        PAN211_WriteReg(0x46, 0xb0);
        PAN211_WriteReg(0x48, 0x88);
        PAN211_WriteReg(PAN211_PAGE_CFG, 0x00);
    #if (PAN211_DATA_RATE == 0) // 1Mbps
        PAN211_WriteReg(0x43, 0x3a);
    #elif (PAN211_DATA_RATE == 3) // 250Kbps
        PAN211_WriteReg(0x43, 0x3b);
    #else
    #endif
        PAN211_WriteReg(0x44, 0x8c);
        break;

    }
}

/**
 * @brief Enter carrier wave transmission mode
 * @note Can set frequency point and transmission power before entering carrier mode
 *          - PAN211_SetChannel(12)
 *          - PAN211_SetTxPower(PAN211_TXPWR_9dBm)
 */
void PAN211_StartCarrierWave(void)
{
    PAN211_WriteReg(0x02, 0x74);
    PAN211_WriteReg(0x03, 0x46);
    PAN211_WriteReg(0x06, 0x85);
    PAN211_WriteReg(0x06, 0xa5);
    PAN211_WriteReg(0x06, 0xe5);
    PAN211_WriteReg(0x06, 0xf5);
    PAN211_WriteReg(0x6A, 0x02);
    PAN211_WriteReg(0x6A, 0x03);
    PAN211_WriteReg(0x6A, 0x23);
    PAN211_WriteReg(0x6A, 0x2b);
    PAN211_WriteReg(0x6B, 0x80);
    PAN211_WriteReg(0x6B, 0xc0);
    PAN211_WriteReg(0x03, 0xc6);
}

/**
 * @brief Exit carrier wave transmission mode
 */
void PAN211_ExitCarrierWave(void)
{
    PAN211_WriteReg(0x6A, 0x00);
    PAN211_WriteReg(0x6B, 0x00);
    PAN211_WriteReg(0x06, 0x05);
    PAN211_WriteReg(0x03, 0x46);
    PAN211_WriteReg(0x03, 0x06);
}
