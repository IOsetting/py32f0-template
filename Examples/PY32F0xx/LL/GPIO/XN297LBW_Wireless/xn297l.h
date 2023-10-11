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

#ifndef __FW_XN297L_H__
#define __FW_XN297L_H__

#include "main.h"
#include "string.h"

#define XN297L_PLOAD_WIDTH       32   // Payload width

#define XN297L_DATA_OUT()        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT)
#define XN297L_DATA_IN()         LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT)
#define XN297L_DATA_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define XN297L_DATA_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define XN297L_DATA_READ()       LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7)

#define XN297L_SCK_LOW()         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define XN297L_SCK_HIGH()        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1)

#define XN297L_CSN_LOW()         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define XN297L_CSN_HIGH()        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)

#define XN297L_CE_LOW()          XN297L_WriteReg(XN297L_CMD_CE_FSPI_OFF, 0)
#define XN297L_CE_HIGH()         XN297L_WriteReg(XN297L_CMD_CE_FSPI_ON, 0)

/**
 * REGISTER TABLE
 */
/****************** SPI  REGISTER  ********************/
#define XN297L_CMD_R_REGISTER    0x00 // [000A AAAA] Register read
#define XN297L_CMD_W_REGISTER    0x20 // [001A AAAA] Register write
#define XN297L_CMD_R_RX_PAYLOAD  0x61 // Read RX payload
#define XN297L_CMD_W_TX_PAYLOAD  0xA0 // Write TX payload
#define XN297L_CMD_FLUSH_TX      0xE1 // Flush TX FIFO
#define XN297L_CMD_FLUSH_RX      0xE2 // Flush RX FIFO
#define XN297L_CMD_REUSE_TX_PL   0xE3 // Reuse TX Payload
#define XN297L_CMD_ACTIVATE      0x50 // ACTIVATE
#define XN297L_CMD_DEACTIVATE    0x50 // DEACTIVATE
#define XN297L_CMD_RST_FSPI      0x53 // RESET
#define XN297L_CMD_R_RX_PL_WID   0x60 // Read width of RX data 
#define XN297L_CMD_W_ACK_PAYLOAD 0xA8 // Data with ACK
#define XN297L_CMD_W_TX_PAYLOAD_NOACK 0xB0 // TX Payload no ACK Request
#define XN297L_CMD_CE_FSPI_ON    0xFD // CE HIGH
#define XN297L_CMD_CE_FSPI_OFF   0xFC // CE LOW
#define XN297L_CMD_NOP           0xFF // No operation (used for reading status register)

/******************CONTROL  REGISTER ******************/
#define XN297L_REG_CONFIG        0x00 // Configuration register
#define XN297L_REG_EN_AA         0x01 // Enable "Auto acknowledgment"
#define XN297L_REG_EN_RXADDR     0x02 // Enable RX addresses
#define XN297L_REG_SETUP_AW      0x03 // Setup of address widths
#define XN297L_REG_SETUP_RETR    0x04 // Setup of automatic re-transmit
#define XN297L_REG_RF_CH         0x05 // RF channel
#define XN297L_REG_RF_SETUP      0x06 // RF setup
#define XN297L_REG_STATUS        0x07 // Status
#define XN297L_REG_OBSERVE_TX    0x08 // Transmit observe register
#define XN297L_REG_RPD           0x09 // 
#define XN297L_REG_RX_ADDR_P0    0x0A // Receive address data pipe 0, 40 bits
#define XN297L_REG_RX_ADDR_P1    0x0B // Receive address data pipe 1, 40 bits
#define XN297L_REG_RX_ADDR_P2    0x0C // Receive address data pipe 2, 40 bits
#define XN297L_REG_RX_ADDR_P3    0x0D // Receive address data pipe 3, 40 bits
#define XN297L_REG_RX_ADDR_P4    0x0E // Receive address data pipe 4, 40 bits
#define XN297L_REG_RX_ADDR_P5    0x0F // Receive address data pipe 5, 40 bits

#define XN297L_REG_TX_ADDR       0x10 // Transmit address, 40 bits
#define XN297L_REG_RX_PW_P0      0x11 // Number of bytes in RX payload in data pipe 0
#define XN297L_REG_RX_PW_P1      0x12 // Number of bytes in RX payload in data pipe 1
#define XN297L_REG_RX_PW_P2      0x13 // Number of bytes in RX payload in data pipe 2
#define XN297L_REG_RX_PW_P3      0x14 // Number of bytes in RX payload in data pipe 3
#define XN297L_REG_RX_PW_P4      0x15 // Number of bytes in RX payload in data pipe 4
#define XN297L_REG_RX_PW_P5      0x16 // Number of bytes in RX payload in data pipe 5

#define XN297L_REG_FIFO_STATUS   0x17 // FIFO status, 20 bits
#define XN297L_REG_DEM_CAL       0x19 // 8bit
#define XN297L_REG_RF_CAL2       0x1A // 48bit
#define XN297L_REG_DEM_CAL2      0x1B // 24bit
#define XN297L_REG_DYNPD         0x1C // Enable dynamic payload length
#define XN297L_REG_FEATURE       0x1D // Feature config
#define XN297L_REG_RF_CAL        0x1E // 24bit
#define XN297L_REG_BB_CAL        0x1F // 40bit

/**************************** CONFIGs ************************************/

#define XN297L_RF_POWER_P_11     0x27  // 100 111: 11dbm
#define XN297L_RF_POWER_P_10     0x26  // 100 110: 10dbm
#define XN297L_RF_POWER_P_9      0x15  // 010 101: 9dbm
#define XN297L_RF_POWER_P_5      0x2c  // 101 100: 5dbm
#define XN297L_RF_POWER_P_4      0x14  // 010 100: 4dbm
#define XN297L_RF_POWER_N_1      0x2A  // 101 010: -1dbm
#define XN297L_RF_POWER_N_9      0x29  // 101 001: -9dbm
#define XN297L_RF_POWER_N_10     0x19  // 011 001: -10dbm
#define XN297L_RF_POWER_N_23     0x30  // 110 000: -23dbm

#define XN297L_RF_DR_2M          0x40  // 2Mbps
#define XN297L_RF_DR_1M          0X00  // 1Mbps
#define XN297L_RF_DR_250K        0XC0  // 250Kbps (work with XN297L_RF_POWER_P_9)

#define XN297L_FLAG_RX_DR        0X40  // Data ready
#define XN297L_FLAG_TX_DS        0X20  // Data sent
#define XN297L_FLAG_RX_TX_CMP    0X60  // Data sent & acked
#define XN297L_FLAG_MAX_RT       0X10  // Max retried
#define XN297L_FLAG_TX_FULL      0x01  // TX FIFO full

#define XN297L_SETUP_AW_3BYTE    0x01  // Address width 3 bytes
#define XN297L_SETUP_AW_4BYTE    0x10  // Address width 4 bytes
#define XN297L_SETUP_AW_5BYTE    0x11  // Address width 5 bytes

#define XN297L_FEATURE_BIT6_MUX_PA          0x40
#define XN297L_FEATURE_BIT6_MUX_IRQ         0x00
#define XN297L_FEATURE_BIT5_CE_SOFT         0x20
#define XN297L_FEATURE_BIT5_CE_HARD         0x00
#define XN297L_FEATURE_BIT43_DATA_64BYTE    0x18
#define XN297L_FEATURE_BIT43_DATA_32BYTE    0x00
#define XN297L_FEATURE_BIT2_EN_DPL_ON       0x04
#define XN297L_FEATURE_BIT2_EN_DPL_OFF      0x00
#define XN297L_FEATURE_BIT1_EN_ACK_PAY_ON   0x02
#define XN297L_FEATURE_BIT1_EN_ACK_PAY_OFF  0x00
#define XN297L_FEATURE_BIT0_EN_NOACK_ON     0x01
#define XN297L_FEATURE_BIT0_EN_NOACK_OFF    0x00

#define XN297L_TEST_ADDR         "XN297"


/******************* FUNCTION DECLARE *******************/

void XN297L_WriteReg(uint8_t reg, uint8_t value);
uint8_t XN297L_ReadReg(uint8_t reg);

void XN297L_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len);
void XN297L_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);

void XN297L_Init(void);
ErrorStatus XN297L_SPI_Test(void);

void XN297L_SetTxAddress(const uint8_t *address);
void XN297L_SetRxAddress(const uint8_t *address);

void XN297L_SetChannel(uint8_t channel);
void XN297L_SetTxMode(void);
void XN297L_SetRxMode(void);

uint8_t XN297L_ReadStatus(void);
void XN297L_ClearStatus(void);

uint8_t XN297L_ClearFIFO(void);
uint8_t XN297L_TxData(uint8_t *ucPayload, uint8_t length);
uint8_t XN297L_DumpRxData(void);
void XN297L_Carrier(uint8_t ucChannel_Set);

uint8_t XN297L_PrintStatus(void);

#endif
