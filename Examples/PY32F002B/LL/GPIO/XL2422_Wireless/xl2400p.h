// Copyright 2021 IOsetting <iosetting(at)outlook.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __FW_XL2400P_H__
#define __FW_XL2400P_H__

#include "main.h"

/**
 * REGISTER TABLE
 */
/****************** SPI  REGISTER  ********************/
#define XL2400_CMD_R_REGISTER    0x00 // [000A AAAA] Register read
#define XL2400_CMD_W_REGISTER    0x20 // [001A AAAA] Register write
#define XL2400_CMD_R_RX_PAYLOAD  0x61 // Read RX payload
#define XL2400_CMD_W_TX_PAYLOAD  0xA0 // Write TX payload
#define XL2400_CMD_FLUSH_TX      0xE1 // Flush TX FIFO
#define XL2400_CMD_FLUSH_RX      0xE2 // Flush RX FIFO
#define XL2400_CMD_REUSE_TX_PL   0xE3 // Reuse TX Payload
#define XL2400_CMD_R_RX_PL_WID   0x60 // Read width of RX data 
#define XL2400_CMD_W_ACK_PAYLOAD 0xA8 // Data with ACK
#define XL2400_CMD_W_TX_PAYLOAD_NOACK 0xB0 // TX Payload no ACK Request
#define XL2400_CMD_NOP           0xFF // No operation (used for reading status register)

/******************CONTROL  REGISTER ******************/
#define XL2400_REG_CFG_TOP       0x00 // Configuration register, 24 bits
                                      // [0]:CE flag, 0:low, 1:high
#define XL2400_REG_EN_AA         0x01 // Enable "Auto acknowledgment", 48 bits
#define XL2400_REG_EN_RXADDR     0x02 // Enable RX addresses, 8 bits
#define XL2400_REG_SETUP_AW      0x03 // Setup of address width of all pipes, 8 bits
#define XL2400_REG_SETUP_RETR    0x04 // Setup of automatic re-transmit, 30 bits
#define XL2400_REG_RF_CH         0x05 // RF channel, 16 bits
#define XL2400_REG_RF_SETUP      0x06 // RF setup, 24 bits
#define XL2400_REG_STATUS        0x07 // Status, 8 bits
#define XL2400_REG_OBSERVE_TX    0x08 // Transmit observe register, 32 bits
#define XL2400_REG_RSSI          0x09 // Data output and RSSI, 14 bits
#define XL2400_REG_RX_ADDR_P0    0x0A // Receive address data pipe 0, 40 bits
#define XL2400_REG_RX_ADDR_P1    0x0B // Receive address data pipe 1, 40 bits
#define XL2400_REG_RX_ADDR_P2_P5 0x0C // Receive address data pipe 2~5, 32 bits
#define XL2400_REG_BER_RESULT    0x0D // BER(PN9) test result, 64 bits
#define XL2400_REG_AGC_SETTING   0x0E // AGC settings, 51 bits
#define XL2400_REG_PGA_SETTING   0x0F // PGA settings, 296 bits
#define XL2400_REG_TX_ADDR       0x10 // Transmit address, 40 bits
#define XL2400_REG_RX_PW_PX      0x11 // Number of bytes in RX payload in data pipe 0 ~ pipe 5, 48 bits
#define XL2400_REG_ANALOG_CFG0   0x12 // Analog config 0, 128 bits
#define XL2400_REG_ANALOG_CFG1   0x13 // Analog config 1, 128 bits
#define XL2400_REG_ANALOG_CFG2   0x14 // Analog config 2, 128 bits
#define XL2400_REG_ANALOG_CFG3   0x15 // Analog config 3, 128 bits
#define XL2400_REG_FIFO_STATUS   0x17 // FIFO status, 20 bits
#define XL2400_REG_RSSIREC       0x18 // RSSI recorder feature, 32 bits
#define XL2400_REG_TXPROC_CFG    0x19 // TX Process configuration, 29 bits
#define XL2400_REG_RXPROC_CFG    0x1A // RX Process configuration, 40 bits
#define XL2400_REG_DYNPD         0x1C // Enable dynamic payload length, 8 bits
#define XL2400_REG_FEATURE       0x1D // Feature config, 8 bits
#define XL2400_REG_RAMP_CFG      0x1E // PA Ramp Configuration, 88 bits


/**************************** CONFIGs ************************************/

#define XL2400_PL_WIDTH_MAX      128  // Max payload width

#define XL2400_RF_10DB           0x3F
#define XL2400_RF_9DB            0x38
#define XL2400_RF_8DB            0x34
#define XL2400_RF_7DB            0x30
#define XL2400_RF_6DB            0x2C // 250Kbps Maximum
#define XL2400_RF_5DB            0x28
#define XL2400_RF_4DB            0x24
#define XL2400_RF_3DB            0x20
#define XL2400_RF_2DB            0x14
#define XL2400_RF_0DB            0x10 // 1Mbps Maximum
#define XL2400_RF__2DB           0x0C
#define XL2400_RF__6DB           0x08
#define XL2400_RF__12DB          0x04
#define XL2400_RF__18DB          0x02
#define XL2400_RF__24DB          0x01

#define XL2400_RF_DR_2M          0x08 // 2Mbps
#define XL2400_RF_DR_1M          0x00 // 1Mbps
#define XL2400_RF_DR_250K        0x20 // 250Kbps
#define XL2400_RF_DR_125K        0x28 // 125Kbps

#define XL2400_FLAG_RX_DR        0X40   // Data ready
#define XL2400_FLAG_TX_DS        0X20   // Data sent
#define XL2400_FLAG_RX_TX_CMP    0X60   // Data sent & acked
#define XL2400_FLAG_MAX_RT       0X10   // Max retried
#define XL2400_FLAG_TX_FULL      0x01 // 1:TX FIFO full


/* FIFO Status */
#define TX_REUSE        		(0x1 << 6)//@
#define TX_FULL            		(0x1 << 5)//@
#define TX_EMPTY        		(0x1 << 4)//@
#define RX_FULL            		(0x2)//@
#define RX_EMPTY        		(0x1)//@


typedef void (*XL2400_SetPinHigh_fptr_t)(GPIO_TypeDef *GPIOx, uint32_t PinMask);
typedef void (*XL2400_SetPinLow_fptr_t)(GPIO_TypeDef *GPIOx, uint32_t PinMask);
typedef uint32_t (*XL2400_GetPinState_fptr_t)(GPIO_TypeDef *GPIOx, uint32_t PinMask);

typedef struct {
    GPIO_TypeDef    *NssPort;
    uint32_t        NssPin;
    GPIO_TypeDef    *MosiPort;
    uint32_t        MosiPin;
    GPIO_TypeDef    *MisoPort;
    uint32_t        MisoPin;
    GPIO_TypeDef    *ClockPort;
    uint32_t        ClockPin;
    GPIO_TypeDef    *InterruptPort;
    uint32_t        InterruptPin;
    XL2400_SetPinHigh_fptr_t    SetPinHigh;
    XL2400_SetPinLow_fptr_t     SetPinLow;
    XL2400_GetPinState_fptr_t   GetPinState;
} XL2400_InitTypedef_t;

void XL2400_Config(XL2400_InitTypedef_t initStruct);

void XL2400_Init(void);

void XL2400_Reset(void);
void XL2400_SetChannel(uint8_t ch);
void XL2400_SetAddress(uint8_t *addr);
void XL2400_SetPower(uint8_t power);
void XL2400_Sleep(void);
void XL2400_SetTxMode(void);
void XL2400_SetRxMode(void);

uint8_t XL2400_Rx(uint8_t *pBuff);
uint8_t XL2400_Tx(uint8_t *pBuff , uint8_t len);



#endif
