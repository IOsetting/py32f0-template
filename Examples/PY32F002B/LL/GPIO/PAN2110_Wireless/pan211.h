/**************************************************************************
 * @file      pan211.c
 * @version   V2.2
 * $Revision: 1 $
 * $Date:     2025/03/31 $
 * @brief     The header file of pan211 driver
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
#ifndef _PAN211_H_
#define _PAN211_H_

// XTAL
#define PAN211_XTAL_FREQ                0       // 0:16MHz, 1:32MHz
// Data Rate
#define PAN211_DATA_RATE                3       // 0:1Mbps, 1:2Mbps, 3:250Kbps

// PAN211 Registers

#define PAN211_PAGE_CFG                 0x00    // default: 0x00
                                                // [0]PAGE_SEL, 0:page0, 1:page1
#define PAN211_P0_TRX_FIFO              0x01    // default: 0x00, FIFO Read/Write Acess Point
#define PAN211_P0_STATE_CFG             0x02    // default 0x00
                                                // [7]TX_FIFO_READY
                                                // [6]EN_LS_3V
                                                // [5]POR_RSTL
                                                // [4]ISO_TO_0
                                                // [0:2]OPERATE_MODE, 0:deep sleep, 1:sleep, 2:STB1, 3:STB2, 4:STB3, 5:TX, 6:RX
#define PAN211_P0_SYS_CFG               0x03    // default 0x02
                                                // [2]IRQ_DATA_MUX_EN 0:disabled, 1:enabled(SPI mode)
                                                // [1]SOFT_RSTL 0:reset, 1:no reset
#define PAN211_P0_SPI_CFG               0x04    // default 0x73
                                                // [7]REG_SPI3_REN, 3-wire spi read mode, 0:off, 1:on
                                                // [6]REG_DATA_PUEN, data pin pull-up, when REG_IN_PAD_MODE = 0, 0:off, 1:on
                                                // [5]REG_CSN_PUEN, csn pin pull-up, when REG_IN_PAD_MODE = 0, 0:off, 1:on
                                                // [4]REG_SCK_PUEN, sck pin pull-up, 0:off, 1:on
                                                // [3]REG_IN_PAD_MODE, 0:CSN_PUEN use REG_CSN_PUEN, 1:CSN_PUEN=1, MOSI_DIEN=1, MOSI_OUT/MOSI_OE=0
#define PAN211_P0_I2C_CFG               0x06    // default 0x05
                                                // [3]IRQ_I2C_MUX_EN, data and irq complex, 0:off, 1:en
#define PAN211_P0_WMODE_CFG0            0x07    // default 0x49
                                                // [6:7]CRC_MODE, 00：CRC DISABLE, 01：CRC-1BYTE, 10：CRC-2BYTE, 11：CRC-3BYTE
                                                // [4:5]WORK_MODE, 00:XN297L, 11:BLE
                                                // [3]WHITEN_ENABLE, 0:off, 1:on
                                                // [2]CRC_SKIP_ADDR, 0:don't skip, 1:skip
                                                // [1]TX_NOACK, in enhanced mode, 0:reply ack, 1:don't reply ack
                                                // [0]ENDIAN, 0:le, BLE, 1:be, XN297L
#define PAN211_P0_WMODE_CFG1            0x08    // default 0x83
                                                // [7]RX_GOON, exit rx when error, 0:exit, 1:no exit
                                                // [6]PRI_EXIT_RX, force exit rx
                                                // [5]FIFO_128_EN, 0:64-byte FIFO, 1:128-byte FIFO; set 1 in BLE mode and XN297L normal mode, set 0 in XN297L enhanced mode
                                                // [4]DPY_EN, dynamic rx length in enhanced mode
                                                // [3]ENHANCE, 0:normal mode, 1:enhanced mode
                                                // [0:1]ADDR_BYTE_LENGTH, address length, 00:2-byte, 01:3-byte, 10:4-byte, 11:5-byte
#define PAN211_P0_RX_PLLEN_CFG          0x09    // default 0x00
                                                // [0:7]RX_PAYLOAD_LENGTH
#define PAN211_P0_TX_PLLEN_CFG          0x0A    // default 0x00
                                                // [0:7]TX_PAYLOAD_LENGTH
#define PAN211_P0_RFIRQ_CFG             0x0B    // default 0x00
                                                // [7]TX_IRQ_MSK 
                                                // [6]TX_MAX_RT_IRQ_MSK
                                                // [5]RX_ADDR_ERR_MSK
                                                // [4]RX_CRC_ERR_IRQ_MSK
                                                // [3]RX_LEN_ERR_IRQ_MSK
                                                // [2]RX_PID_ERR_IRQ_MSK
                                                // [1]RX_TIMEOUT_IRQ_MSK
                                                // [0]RX_IRQ_MSK
#define PAN211_P0_PID_CFG               0x0C    // default 0x00
                                                // [7]PID_MANUAL_EN manual pid, 0:off, 1:on
                                                // [4:6]ADDR_ERR_THR, address error threshold, 0:0, 1:1bit, ~ 7:1bit
                                                // [2:3]RX_PID_MANUAL, when PID_MANUAL_EN=1
                                                // [0:1]TX_PID_MANUAL, when PID_MANUAL_EN=1
#define PAN211_P0_TRXTWTL_CFG           0x0D    // default 0x00
                                                // [0:7]TRX_TRANS_WAIT_TIME LSB, delay between tx rx switch, unit:us
#define PAN211_P0_TRXTWTH_CFG           0x0E    // default 0x00
                                                // [0:7]TRX_TRANS_WAIT_TIME MSB, delay between tx rx switch, unit:us
#define PAN211_P0_PIPE0_RXADDR0_CFG     0x0F    // default 0xCC, pipe0 RX address 0
#define PAN211_P0_PIPE0_RXADDR1_CFG     0x10    // default 0xCC, pipe0 RX address 1
#define PAN211_P0_PIPE0_RXADDR2_CFG     0x11    // default 0xCC, pipe0 RX address 2
#define PAN211_P0_PIPE0_RXADDR3_CFG     0x12    // default 0xCC, pipe0 RX address 3
#define PAN211_P0_PIPE0_RXADDR4_CFG     0x13    // default 0xCC, pipe0 RX address 4
#define PAN211_P0_TXADDR0_CFG           0x14    // default 0xCC, TX address 0
#define PAN211_P0_TXADDR1_CFG           0x15    // default 0xCC, TX address 1
#define PAN211_P0_TXADDR2_CFG           0x16    // default 0xCC, TX address 2
#define PAN211_P0_TXADDR3_CFG           0x17    // default 0xCC, TX address 3
#define PAN211_P0_TXADDR4_CFG           0x18    // default 0xCC, TX address 4
#define PAN211_P0_PKT_EXT_CFG           0x19    // default 0x00
                                                // [7]W_RX_MAX_CTRL_EN, enable max package length in rx, 0:enabled, 1:disabled
                                                // [6]HDR_LEN_EXIST, 0:disable HEADER and LENGTH, 1:enable HEADER and LENGTH
                                                // [4:5]HDR_LEN_NUMB, take effect when HDR_LEN_EXIST=1, 
                                                //          00:no HEADER and LENGTH, 
                                                //          01:LENGTH for first byte after address, no HEADER
                                                //          10:LENGTH for second byte after address, HEADER for first byte after address
                                                //          11:LENGTH for third byte after address, HEADER for first 2 bytes after address
                                                // [3]PRI_TX_FEC, 0:off, 1:on
                                                // [2]PRI_RX_FEC ,0:off, 1:on
                                                // [0:1]PRI_CI_MODE, 00:S8, 01:S2
#define PAN211_P0_WHITEN_CFG            0x1A    // default 0x7F
                                                // [7]WHITEN_SKIP_ADDR, 0:no skip, 1:skip
                                                // [0:6]WHITEN_SEED
#define PAN211_P0_TXHDR0_CFG            0x1B    // default 0x00
                                                // [0:7]TX_HEADER0, take effect when HDR_LEN_EXIST=1 and HDR_LEN_NUMB = 10 or 11
#define PAN211_P0_TXHDR1_CFG            0x1C    // default 0x00
                                                // [0:7]TX_HEADER1, take effect when HDR_LEN_EXIST=1 and HDR_LEN_NUMB = 11
#define PAN211_P0_TXRAMADDR_CFG         0x1D    // default 0x00
                                                // [0:7]TX_RAM_START_ADDR
#define PAN211_P0_RXRAMADDR_CFG         0x1E    // default 0x00
                                                // [0:7]RX_RAM_START_ADDR
#define PAN211_P0_RXPIPE_CFG            0x1F    // default 0x01
                                                // [5]PIPE5_EN
                                                // [4]PIPE4_EN
                                                // [3]PIPE3_EN
                                                // [2]PIPE2_EN
                                                // [1]PIPE1_EN
                                                // [0]PIPE0_EN
#define PAN211_P0_PIPE1_RXADDR0_CFG     0x20    // default 0xCC, pipe1 RX address LSB 8bit
#define PAN211_P0_PIPE1_RXADDR1_CFG     0x21    // default 0xCC
#define PAN211_P0_PIPE1_RXADDR2_CFG     0x22    // default 0xCC
#define PAN211_P0_PIPE1_RXADDR3_CFG     0x23    // default 0xCC
#define PAN211_P0_PIPE1_RXADDR4_CFG     0x24    // default 0xCC
#define PAN211_P0_PIPE2_RXADDR0_CFG     0x25    // default 0xCC, pipe2 RX address
#define PAN211_P0_PIPE3_RXADDR0_CFG     0x26    // default 0xCC, pipe3
#define PAN211_P0_PIPE4_RXADDR0_CFG     0x27    // default 0xCC, pipe4
#define PAN211_P0_PIPE5_RXADDR0_CFG     0x28    // default 0xCC, pipe5
#define PAN211_P0_TXAUTO_CFG            0x29    // default 0x03
                                                // [4:7]ARD, auto retry interval, 0000：250µs, 0001：500µs, 0010：750µs, ……, 1111：4000µs
                                                // [0:3]ARC, retry times, 0000:no ack, 0001：ack, no retry, 0002: ack, retry once, ……, 1111:ack, retry 14 times
#define PAN211_P0_TRXMODE_CFG           0x2A    // default 0x01
                                                // [7]REG_TX_CFG_MODE, 0:single, 1:continuous
                                                // [5:6]REG_RX_CFG_MODE, 
                                                //      in normal mode: 0:single, 1:single with timeout, 2:continuous
                                                //      in enhanced mode: 0: continuous, 1:continuous with timeout
                                                // [4]PRE_2BYTE_MODE, in XN297L mode with 2mbps rate, 0: one PREAMBLE, 1: double PREAMBLE
                                                // [3]W_PRE_SYNC_12B_EN, enable 12bit presync, 0:off, 1:enable
                                                // [2]W_PRE_SYNC_8B_EN, enable 8bit presync, 0:off, 1:enable 
                                                // [1]W_PRE_SYNC_4B_EN, enable 4bit presync, 0:off, 1:enable 
                                                // [0]W_PRE_SYNC_EN, enable presync
                                                //      if W_PRE_SYNC_EN=1 and all 12B_EN, 8B_EN, 4B_EN = 0, enable 16bit presync
#define PAN211_P0_RXTIMEOUTL_CFG        0x2B    // default 0xD0
                                                // [0:7]REG_RX_TIMEOUT, rx timeout LSB
#define PAN211_P0_RXTIMEOUTH_CFG        0x2C    // default 0x07
                                                // [0:7]REG_RX_TIMEOUT, rx timeout MSB
#define PAN211_P0_BLEMATCH_CFG0         0x2D    // default 0x00
                                                // [7]SNIF_EN, enable SNIFFER, 0:off, 1:on
                                                // [4:6]WL_MATCH_MODE, RX whitelist filter in BLE mode
                                                //      000:no filter
                                                //      001:match with WL_ADVA[47:40]
                                                //      010:match with WL_ADVA[47:32]
                                                //      011:match with WL_ADVA[47:24]
                                                //      100:match with WL_ADVA[47:16]
                                                //      101:match with WL_ADVA[47:8]
                                                //      110:match with WL_ADVA[47:0]
                                                //      111:no filter, the same as 000
                                                // [2:3]BLELEN_MATCH_MODE, RX length filter in BLE mode
                                                //      00: off
                                                //      01: rx length = RXPLLEN_CFG
                                                //      10: rx length > RXPLLEN_CFG
                                                //      11: rx length < RXPLLEN_CFG
#define PAN211_P0_BLEMATCH_CFG1         0x2E    // default 0x28
                                                // 
#define PAN211_P0_WLIST0_CFG            0x2F    // default 0x00
                                                // [0:7]WL_ADVA [0:7]
#define PAN211_P0_WLIST1_CFG            0x30    // default 0x00
                                                // [0:7]WL_ADVA [8:15]
#define PAN211_P0_WLIST2_CFG            0x31    // default 0x00
                                                // [0:7]WL_ADVA [16:23]
#define PAN211_P0_WLIST3_CFG            0x32    // default 0x00
                                                // [0:7]WL_ADVA [24:31]
#define PAN211_P0_WLIST4_CFG            0x33    // default 0x00
                                                // [0:7]WL_ADVA [32:39]
#define PAN211_P0_WLIST5_CFG            0x34    // default 0x00
                                                // [0:7]WL_ADVA [40:47]
#define PAN211_P0_BLEMATCHSTART_CFG     0x35    // default 0x07
                                                // [0:5]PLD_START_BYTE, whitelist start byte in BLE mode, 1~6: start from AdvA, 7~39: start from PAYLOAD
#define PAN211_P0_RF_DATARATE_CFG       0x36    // default 0x45
                                                // [4:5]DATARATE, 00:1Mbps, 01:2Mbps, 11:250Kbps
#define PAN211_P0_RF_CHANNEL_CFG        0x39    // default 0x00
                                                // [0:7]RF_CH, F0= 2400 + RF_CH [MHz]
#define PAN211_P0_IRQ_MUX_CFG           0x45    // default 0x00
                                                // [2:3]OCLK_SEL, clock output, 00:1kHz, 01:4kHz, 10:8MHz, 11:16MHz
                                                // [0:1]IRQ_MUX, IRQ pin function, 00:IRQ, 01:clock output, 10:PA control
#define PAN211_P0_MISC_CFG              0x6F    // default 0x00
                                                // [6]ENH_NOACK_RX_CONT_DIS
                                                // [5]I_NDC_PREAMBLE_SEL, 0:XN297L mode, 1:BLE mode
                                                // [4]PID_LOW_SEL, pid location, 0:middle, 1:lowest 2-bit
                                                // [3]IRQ_HIGH_EN, 0:en low, 1:en high
                                                // [0:2]ACK_PIPE
#define PAN211_P0_RFIRQFLG              0x73    // default 0x00
                                                // [7]TX_IRQ, set after sending finish, write 1 to clear
                                                // [6]TX_MAX_RT_IRQ
                                                // [5]RX_ADDR_ERR_IRQ
                                                // [4]RX_CRC_ERR_IRQ
                                                // [3]RX_LENGTH_ERR_IRQ
                                                // [2]RX_PID_ERR_IRQ
                                                // [1]RX_TIMEOUT_IRQ
                                                // [0]RX_IRQ
#define PAN211_P0_STATUS0               0x74    // default 0x0C
                                                // [7]RX_CI_ERR
                                                // [4:6]RX_SYNC_ADDR
                                                // [2:3]RX_PID
                                                // [0:1]TX_PID
#define PAN211_P0_STATUS1               0x75    // default 0x00
                                                // [0:7]RX_HEADER[7:0]
#define PAN211_P0_STATUS2               0x76    // default 0x00
                                                // [0:7]RX_HEADER[15:8]
#define PAN211_P0_STATUS3               0x77    // default 0x00
                                                // [0:7]RX_PAYLOAD_LENGTH
#define PAN211_P0_PKT_RSSI_L            0x7A    // default 0x00
                                                // [0:7]PKT_RSSI_L, PKT_RSSI LSB, noise(dBm) = (PKT_RSSI -16384)/4
#define PAN211_P0_PKT_RSSI_H            0x7B    // default 0x00
                                                // [0:5]PKT_RSSI_H, PKT_RSSI MSB
#define PAN211_P0_RT_RSSI_L             0x7C    // default 0x00
                                                // [0:7]RT_RSSI_L, RT_RSSI LSB, noise(dBm) = (RT_RSSI -16384)/4
#define PAN211_P0_RT_RSSI_H             0x7D    // default 0x00
                                                // [0:5]RT_RSSI_H


// PAN211 IRQ

#define RF_IT_TX_IRQ             0x80  /* Transmission complete interrupt */
#define RF_IT_MAX_RT_IRQ         0x40  /* Transmission failure interrupt when maximum retransmission count reached in enhanced mode */
#define RF_IT_ADDR_ERR_IRQ       0x20  /* Address error interrupt */
#define RF_IT_CRC_ERR_IRQ        0x10  /* CRC error interrupt */
#define RF_IT_LEN_ERR_IRQ        0x08  /* Data length error interrupt */
#define RF_IT_PID_ERR_IRQ        0x04  /* Received incorrect PID interrupt in enhanced mode */
#define RF_IT_RX_TIMEOUT_IRQ     0x02  /* Reception timeout interrupt */
#define RF_IT_RX_IRQ             0x01  /* Reception complete interrupt */
#define RF_IT_ALL_IRQ            0xFF  /* All interrupt flags */

// PAN211 RF Power

#define PAN211_TXPWR_0dBm        0x00  /* 0dBm */
#define PAN211_TXPWR_1dBm        0x01  /* 1dBm */
#define PAN211_TXPWR_2dBm        0x02  /* 2dBm */
#define PAN211_TXPWR_3dBm        0x03  /* 3dBm */
#define PAN211_TXPWR_4dBm        0x04  /* 4dBm */
#define PAN211_TXPWR_5dBm        0x05  /* 5dBm */
#define PAN211_TXPWR_6dBm        0x06  /* 6dBm */
#define PAN211_TXPWR_7dBm        0x07  /* 7dBm */
#define PAN211_TXPWR_8dBm        0x08  /* 8dBm */
#define PAN211_TXPWR_9dBm        0x09  /* 9dBm */


/**
 * @brief Write single byte to specified register
 * @param Addr Register address to write
 * @param Value Single byte data to write to register
 * @return None
 */
void PAN211_WriteReg(unsigned char Addr, unsigned char Value);
/**
 * @brief Read single byte from specified register
 * @param Addr Register address to read
 * @return unsigned char Value read from register
 */
unsigned char PAN211_ReadReg(unsigned char Addr);
/**
 * @brief Continuously write multiple PAN211 register values
 * @param Addr Starting register address
 * @param Buffer Buffer containing data to write
 * @param Len Number of bytes to write
 */
void PAN211_WriteRegs(unsigned char Addr, unsigned char *Buffer, unsigned char Len);
/**
 * @brief Continuously read multiple PAN211 register values
 * @param Addr Starting register address
 * @param Buffer Buffer to store read data
 * @param Len Number of bytes to read
 */
void PAN211_ReadRegs(unsigned char Addr, unsigned char *Buffer, unsigned char Len);
/**
 * @brief Write data to PAN211 TX FIFO
 * @param Buffer Data buffer to write to FIFO
 * @param Size Number of bytes of data to write
 */
void PAN211_WriteFIFO(unsigned char *Buffer, unsigned char Size);
/**
 * @brief Read data from data FIFO
 * @param Buffer Buffer to store data read from FIFO
 * @param Size Number of bytes to read
 */
void PAN211_ReadFIFO(unsigned char *Buffer, unsigned char Size);
/**
 * @brief When DATA pin multiplexed interrupt function is enabled, detect if DATA pin has interrupt event during SPI idle period, active low.
 * @note Method to enable DATA pin multiplexed interrupt function is to set IRQ_DATA_MUX_EN (Page0 0x03[2]) to 1.
 * @return 1: Interrupt triggered
 * @return 0: No interrupt triggered
 */
unsigned char IRQDetected(void);

/**
 * @brief Initialize PAN211 transceiver to STB3 state after power-on
 */
unsigned char PAN211_Init(void);
/**
 * @brief PAN211 transceiver soft reset (SOFT_RSTL)
 * @note This function must be called in STB3 state.
 * @note This function will reset PAN211 transceiver internal logic, must call PAN211_Init function to reinitialize after reset.
 */
void PAN211_SoftRst(void);
/**
 * @brief Enter sleep state from standby state
 */
void PAN211_EnterSleep(void);
/**
 * @brief Enter standby state
 * @note Before calling this function, please ensure to clear IRQ
 */
void PAN211_EnterStandby(void);
/**
 * @brief Exit sleep state and enter standby state
 */
void PAN211_ExitSleep(void);
/**
 * @brief Exit transmission state and enter standby state. Before calling this function, please ensure to clear IRQ
 */
void PAN211_ExitTxMode(void);
/**
 * @brief Exit reception state and enter standby state
 * @note Before calling this function, please ensure to clear IRQ
 */
void PAN211_ExitRxMode(void);
/**
 * @brief Enter transmission state
 * @note Will return to standby state first before entering transmission state
 */
void PAN211_TxStart(void);
/**
 * @brief Exit reception state and enter standby state, before calling this function, please ensure to clear IRQ
 */
void PAN211_RxStart(void);

/**
 * @brief Set PAN211 transceiver frequency channel
 * @param Channel Desired RF channel, range from 0 to 83
 * @note Actual frequency will be (2400 + Channel)MHz
 */
void PAN211_SetChannel(unsigned char Channel);
/**
 * @brief Set PAN211 transceiver frequency channel
 * @param Channel Desired RF channel, range from 0 to 83
 * @note Actual frequency will be (2400 + Channel)MHz
 */
void PAN211_SetAddrWidth(unsigned char AddrWidth);
/**
 * @brief Set static reception address for specified channel
 * @param Pipe Channel to configure address, range from 0 to 5
 * @param Addr Buffer pointer containing address
 * @param Len Address length
 * @note Buffer length must equal transceiver current address width
 * @note For channels [2..5], only write first byte of address, because channels 1-5 share four most significant address bytes
 */
void PAN211_SetRxAddr(unsigned char Pipe, unsigned char *Addr, unsigned char Len);
/**
 * @brief Set transceiver static transmission address
 * @param Addr Buffer pointer containing address
 * @param Len Address length
 */
void PAN211_SetTxAddr(unsigned char *Addr, unsigned char Len);
/**
 * @brief Get channel number from which payload can be read from RX FIFO
 * @return Channel number (0-5), returns 0xFF if RX FIFO is empty
 */
unsigned char PAN211_GetRxPipeNum(void);
/**
 * @brief Set ACK response channel number
 * @param AckPipeNum ACK channel number to set, range from 0 to 5
 * @note This function is used to set ACK response channel number when receiving data packet
 *       In enhanced mode, receiving end will automatically send ACK response after receiving data packet,
 *       Before sending ACK response, must first get receiving data packet channel number through PAN211_GetRxPipeNum() function,
 *       Then call this function to set ACK response channel number.
 */
void PAN211_SetAckPipeNum(unsigned char AckPipeNum);
/**
 * @brief Get pending IRQ flags
 * @return Current status of STATUS register RX_DONE, TX_DONE, RX_TIMEOUT and MAX_RT bits
 */
unsigned char PAN211_GetIRQFlags(void);
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
void PAN211_ClearIRQFlags(unsigned char Flags);
/**
 * @brief Set BLE whitening initial value
 * @param Value Whitening initial value
 */
void PAN211_SetWhiteInitVal(unsigned char Value);
/**
 * @brief Set PAN211 transmission power (auto-generated by tool)
 * @param TxPower Transmission power value, use PAN211_TXPWR_* constants
 */
unsigned char PAN211_GetRecvLen(void);
/**
 * @brief Set PAN211 transmission power (auto-generated by tool)
 * @param TxPower Transmission power value, use PAN211_TXPWR_* constants
 */
void PAN211_SetTxPower(int8_t TxPower);

/**
 * @brief Enter carrier wave transmission mode
 * @note Can set frequency point and transmission power before entering carrier mode
 *          - PAN211_SetChannel(12)
 *          - PAN211_SetTxPower(PAN211_TXPWR_9dBm)
 */
void PAN211_StartCarrierWave(void);
/**
 * @brief Exit carrier wave transmission mode
 */
void PAN211_ExitCarrierWave(void);

#endif
