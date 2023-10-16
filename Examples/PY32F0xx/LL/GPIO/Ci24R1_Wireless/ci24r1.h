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

#ifndef __FW_CI24R1_H__
#define __FW_CI24R1_H__

#include "main.h"
#include "string.h"

#define CI24R1_PLOAD_WIDTH       32  // Payload width, 0:dynamic, [1,32]:fixed

#define CI24R1_DATA_OUT()        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT)
#define CI24R1_DATA_IN()         LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT)
#define CI24R1_DATA_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define CI24R1_DATA_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define CI24R1_DATA_READ()       LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7)

#define CI24R1_CLK_LOW()         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define CI24R1_CLK_HIGH()        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1)

#define CI24R1_NSS_LOW()         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define CI24R1_NSS_HIGH()        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)

#define CI24R1_CE_LOW()          CI24R1_WriteReg(CI24R1_CMD_CE_OFF, CI24R1_CMD_NOP)
#define CI24R1_CE_HIGH()         CI24R1_WriteReg(CI24R1_CMD_CE_ON, CI24R1_CMD_NOP)

/**
 * REGISTER TABLE
 */
/****************** CMD  REGISTER  ********************/
#define CI24R1_CMD_R_REGISTER    0x00 // [000A AAAA] Register read
#define CI24R1_CMD_W_REGISTER    0x20 // [001A AAAA] Register write
#define CI24R1_CMD_R_RX_PAYLOAD  0x61 // Read RX payload
#define CI24R1_CMD_W_TX_PAYLOAD  0xA0 // Write TX payload
#define CI24R1_CMD_FLUSH_TX      0xE1 // Flush TX FIFO
#define CI24R1_CMD_FLUSH_RX      0xE2 // Flush RX FIFO
#define CI24R1_CMD_REUSE_TX_PL   0xE3 // Reuse TX Payload
#define CI24R1_CMD_R_RX_PL_WID   0x60 // Read width of RX data 
#define CI24R1_CMD_W_ACK_PAYLOAD 0xA8 // Data with ACK
#define CI24R1_CMD_W_TX_PAYLOAD_NOACK 0xB0 // TX Payload no ACK Request
#define CI24R1_CMD_NOP           0xFF // No operation (used for reading status register)
#define CI24R1_CMD_CE_ON         0x70 // CE high
#define CI24R1_CMD_CE_OFF        0x71 // CE low
#define CI24R1_CMD_SELSPI        0x74 // DATA pin as SPI
#define CI24R1_CMD_SELIRQ        0x75 // DATA pin as IRQ

/******************CONTROL  REGISTER ******************/
#define CI24R1_REG_CONFIG        0x00 // Configuration register
#define CI24R1_REG_EN_AA         0x01 // Enable "Auto acknowledgment"
#define CI24R1_REG_EN_RXADDR     0x02 // Enable RX addresses
#define CI24R1_REG_SETUP_AW      0x03 // Setup of address widths
#define CI24R1_REG_SETUP_RETR    0x04 // Setup of automatic re-transmit
#define CI24R1_REG_RF_CH         0x05 // RF channel, [0,6] -> 0~125 -> 2400~2525 MHz
#define CI24R1_REG_RF_SETUP      0x06 // RF setup
#define CI24R1_REG_STATUS        0x07 // Status
#define CI24R1_REG_OBSERVE_TX    0x08 // Transmit observe register
#define CI24R1_REG_RSSI          0x09 // Data output and RSSI
#define CI24R1_REG_RX_ADDR_P0    0x0A // Receive address data pipe 0, 40 bits
#define CI24R1_REG_RX_ADDR_P1    0x0B // Receive address data pipe 1, 40 bits
#define CI24R1_REG_RX_ADDR_P2    0x0C // Receive address data pipe 2
#define CI24R1_REG_RX_ADDR_P3    0x0D // Receive address data pipe 3
#define CI24R1_REG_RX_ADDR_P4    0x0E // Receive address data pipe 4
#define CI24R1_REG_RX_ADDR_P5AF  0x0F // Receive address data pipe 5 and other settings
                                      // 0x0F can be switched to different function registers 
                                      // according 0x06(CI24R1_REG_RF_SETUP) bit [0,2], defined 
                                      // by CI24R1_EN_RXADDR_xxx

#define CI24R1_REG_TX_ADDR       0x10 // Transmit address, 40 bits
#define CI24R1_REG_RX_PW_P0      0x11 // Length of RX payload of pipe 0, set to 0 will stop receiving
#define CI24R1_REG_RX_PW_P1      0x12 // Length of RX payload of pipe 1
#define CI24R1_REG_RX_PW_P2      0x13 // Length of RX payload of pipe 2
#define CI24R1_REG_RX_PW_P3      0x14 // Length of RX payload of pipe 3
#define CI24R1_REG_RX_PW_P4      0x15 // Length of RX payload of pipe 4
#define CI24R1_REG_RX_PW_P5      0x16 // Length of RX payload of pipe 5
#define CI24R1_REG_FIFO_STATUS   0x17 // FIFO status
#define CI24R1_REG_DYNPD         0x1C // Enable dynamic payload length
#define CI24R1_REG_FEATURE       0x1D // Feature config

/**************************** CONFIGs ************************************/

#define CI24R1_EN_RXADDR_P5      0x00 // [0,1]->EN_AA[6,7], [2,3]->EN_RXADDR[6,7]
#define CI24R1_EN_RXADDR_CRC     0x01
#define CI24R1_EN_RXADDR_OSC_C   0x02
#define CI24R1_EN_RXADDR_BT      0x04
#define CI24R1_EN_RXADDR_BT_CRC_L 0x06
#define CI24R1_EN_RXADDR_BT_CRC_M 0x07
#define CI24R1_EN_RXADDR_BT_CRC_H 0x08

#define CI24R1_RF_SETUP_11DB     0x07
#define CI24R1_RF_SETUP_10DB     0x06
#define CI24R1_RF_SETUP_9DB      0x05
#define CI24R1_RF_SETUP_7DB      0x04
#define CI24R1_RF_SETUP_3DB      0x03
#define CI24R1_RF_SETUP__1DB     0x02
#define CI24R1_RF_SETUP__4DB     0x01
#define CI24R1_RF_SETUP__9DB     0x00

#define CI24R1_RF_SETUP_250K     0x20
#define CI24R1_RF_SETUP_1M       0x00
#define CI24R1_RF_SETUP_2M       0x08


#define CI24R1_FLAG_RX_READY     0x40 // RX_DR bit (data ready RX FIFO interrupt)
#define CI24R1_FLAG_TX_SENT      0x20 // TX_DS bit (data sent TX FIFO interrupt)
#define CI24R1_FLAG_MAX_RT       0x10 // MAX_RT bit (maximum number of TX re-transmits interrupt)

#define CI24R1_PLOAD_MAX_WIDTH   32  // Max payload width
#define CI24R1_TEST_ADDR         "ABCDE"

/******************* FUNCTION DECLARE *******************/
void CI24R1_WriteReg(uint8_t reg, uint8_t value);
uint8_t CI24R1_ReadReg(uint8_t reg);

void CI24R1_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len);
void CI24R1_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);

ErrorStatus CI24R1_SPI_Test(void);

void CI24R1_Init(void);
void CI24R1_SetChannel(uint8_t channel);

void CI24R1_SetTxAddress(const uint8_t *address);
void CI24R1_SetRxAddress(const uint8_t *address);

void CI24R1_SetTxMode(void);
void CI24R1_SetRxMode(void);

uint8_t CI24R1_Tx(uint8_t *ucPayload, uint8_t length);
uint8_t CI24R1_Tx2(uint8_t *ucPayload, uint8_t length);
uint8_t CI24R1_Rx(void);

uint8_t CI24R1_ReadStatus(void);
uint8_t CI24R1_PrintStatus(void);

#endif
