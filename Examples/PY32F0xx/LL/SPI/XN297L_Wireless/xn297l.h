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

/****************** SPI  REGISTER  ********************/

#define XN297L_CMD_R_REGISTER    0x00 // [000A AAAA] Register read
#define XN297L_CMD_W_REGISTER    0x20 // [001A AAAA] Register write
#define XN297L_CMD_R_RX_PAYLOAD  0x61 // Read RX payload
#define XN297L_CMD_W_TX_PAYLOAD  0xA0 // Write TX payload
#define XN297L_CMD_FLUSH_TX      0xE1 // Flush TX FIFO, used in TX mode
#define XN297L_CMD_FLUSH_RX      0xE2 // Flush RX FIFO, used in RX mode
#define XN297L_CMD_REUSE_TX_PL   0xE3 // Used for a PTX device, reuse last transmitted payload
#define XN297L_CMD_ACTIVATE      0x50 // This command followed by data 0x73 activates R_RX_PL_WID, W_TX_PAYLOAD_NOACK and W_ACK_PAYLOAD, executable in power down or standby modes only
#define XN297L_CMD_DEACTIVATE    0x50 // This command followed by data 0x8C deactivates the above features
#define XN297L_CMD_RST_FSPI      0x53 // This command followed by data 0x5A, switch the chip to reset mode; followed by data 0xA5, switch to normal mode
#define XN297L_CMD_R_RX_PL_WID   0x60 // Read width of RX data 
#define XN297L_CMD_W_ACK_PAYLOAD 0xA8 // Used in RX mode. Write Payload to be transmitted together with ACK packet on PIPE PPP. (PPP valid in the range from 000 to 101).
#define XN297L_CMD_W_TX_PAYLOAD_NOACK 0xB0 // Write Payload to be transmitted, used in TX mode. Disable auto ACK on this specific packet.
#define XN297L_CMD_CE_FSPI_ON    0xFD // Set CE internal logic to 1, use the command followed by the data 0x00
#define XN297L_CMD_CE_FSPI_OFF   0xFC // Set CE internal logic to 0, use the command followed by the data 0x00
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
#define XN297L_REG_RPD           0x09 // Data output and RSSI
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

/**
 * 01 CONFIG BIT[7:0]
*/
#define XN297L_CONFIG_EN_PM         0x80  // STB3 Mode(when PWR_UP=1), wait 50us+ before switching to other mode
#define XN297L_CONFIG_MASK_RX_DR    0x40  // Supress RX_DR interrupt on IRQ pin
#define XN297L_CONFIG_MASK_TX_DS    0x20  // Supress TX_DS interrupt on IRQ pin
#define XN297L_CONFIG_MASK_MAX_RT   0x10  // Supress MAX_RT interrupt on IRQ pin
#define XN297L_CONFIG_EN_CRC        0x08  // Enable CRC (need 2 additional bytes)
#define XN297L_CONFIG_PWR_UP        0x02  // Enable Chip (Power up)
#define XN297L_CONFIG_PRIM_RX       0x01  // RX mode
/**
 * 01 EN_AA BIT[5:0]
 *   Each bit stands for one channel
 * 
 * 02 EN_RXADDR BIT[5:0]
 *   Each bit stands for one channel
 *
 * 03 SETUP_AW BIT[1:0]
*/
#define XN297L_SETUP_AW_3BYTE    0x01  // Address width 3 bytes
#define XN297L_SETUP_AW_4BYTE    0x10  // Address width 4 bytes
#define XN297L_SETUP_AW_5BYTE    0x11  // Address width 5 bytes
/**
 * 04 SETUP_RETR BIT[7:0]
 *   ARD BIT[7:4] 0x00:250us, 0x10:500us, ... 0xF0:4000us
 *   ARC BIT[3:0] 0x00:no retry, 0x01:1 retry, 0x0F:15 retries
 * 
 * 05 RF_CH BIT[6:0]
 *   Channel: 0x00 ~ 0x7F, Frequency(MHz) = 2400 + channel
 * 
 * 06 RF_SETUP BIT[5:0]
*/
#define XN297L_RF_POWER_P_11     0x27  // 100 111: 11dbm
#define XN297L_RF_POWER_P_10     0x26  // 100 110: 10dbm
#define XN297L_RF_POWER_P_9      0x15  // 010 101: 9dbm
#define XN297L_RF_POWER_P_7      0x0D  // 001 101: 7dbm, incompatible with 250Kbps. Recommend for safety regulations test(in 1Mbps).
#define XN297L_RF_POWER_P_6      0x06  // 000 110: 6dbm, incompatible with 250Kbps. Recommend for safety regulations test(in 1Mbps).
#define XN297L_RF_POWER_P_5      0x2C  // 101 100: 5dbm
#define XN297L_RF_POWER_P_5L     0x05  // 000 101: 5dbm, incompatible with 250Kbps. Recommend for safety regulations test(in 1Mbps).
#define XN297L_RF_POWER_P_4      0x14  // 010 100: 4dbm
#define XN297L_RF_POWER_P_3      0x0C  // 001 100: 3dbm, incompatible with 250Kbps. Recommend for safety regulations test(in 1Mbps).
#define XN297L_RF_POWER_N_1      0x2A  // 101 010: -1dbm
#define XN297L_RF_POWER_N_9      0x29  // 101 001: -9dbm
#define XN297L_RF_POWER_N_10     0x19  // 011 001: -10dbm
#define XN297L_RF_POWER_N_23     0x30  // 110 000: -23dbm
/**
 * 06 RF_SETUP BIT[7:6]
*/
#define XN297L_RF_DR_2M          0x40  // 2Mbps
#define XN297L_RF_DR_1M          0X00  // 1Mbps
#define XN297L_RF_DR_250K        0XC0  // 250Kbps (work with XN297L_RF_POWER_P_9)
/**
 * 07 STATUS BIT[6:0]
*/
#define XN297L_FLAG_RX_DR        0X40  // RX FIFO, data ready
#define XN297L_FLAG_TX_DS        0X20  // TX FIFO, data sent completed
#define XN297L_FLAG_MAX_RT       0X10  // Reach max retries, sending failed
#define XN297L_FLAG_RX_P_NO      0X0E  // RX FIFO, mask, data ready pipes
#define XN297L_FLAG_TX_FULL      0x01  // TX FIFO is full
/**
 * 08 OBSERVE_TX
 *   PLOS_CNT BIT[7:4]: Package lost counter, max value is 15
 *   ARC_CNT BIT[3:0]: Retry counter, when it reaches SETUP_RETR.ARC, PLOS_CNT + 1. Write data to TX FIFO resets this value
 * 
 * 09 DATAOUT BIT[7:0]
 *   BIT[7:4]: Realtime RSSI
 *   BIT[3:0]: Packet received RSSI
 * 
 * 0A RX_ADDR_P0 BIT[39:0]
 * 0B RX_ADDR_P1 BIT[39:0]
 * 0C RX_ADDR_P2 BIT[7:0]
 * 0D RX_ADDR_P3 BIT[7:0]
 * 0E RX_ADDR_P4 BIT[7:0]
 * 0F RX_ADDR_P5 BIT[7:0]
 * 
 * 10 TX_ADDR    BIT[39:0]
 * 
 * 11 RX_PW_P0   BIT[6:0] RX pipe0 payload length
 * 12 RX_PW_P1   BIT[6:0] RX pipe1 payload length
 * 13 RX_PW_P2   BIT[6:0] RX pipe2 payload length
 * 14 RX_PW_P3   BIT[6:0] RX pipe3 payload length
 * 15 RX_PW_P4   BIT[6:0] RX pipe4 payload length
 * 16 RX_PW_P5   BIT[6:0] RX pipe5 payload length
 * 
 * 17 FIFO_STATUS BIT[6:0]
*/
#define XN297L_FIFO_STATUS_TX_REUSE     0x40
#define XN297L_FIFO_STATUS_TX_FULL      0x20
#define XN297L_FIFO_STATUS_TX_EMPTY     0x10
#define XN297L_FIFO_STATUS_RX_FULL      0x02
#define XN297L_FIFO_STATUS_RX_EMPTY     0x01

/**
 * 19 DEMOD_CAL BIT[7:0]        * 0x01 *
 *   CHIP BIT[7]:               0, Debug mode, 1:ON, 0:OFF
 *   CARR BIT[6:5]:             00, Carrior test mode, 11:ON and CHIP=1, 00:OFF
 *   GAUS_CAL BIT[4:1]:         0111, Gauss filter on DAC signal, 1111:low, 0000:high
 *   SCRAMBLE_EN BIT[0]:        1, Scrambling code enabled, 1:ON, 0:OFF
 * 
 * 1A RF_CAL2 BIT[45:0]         * 0x45, 0x21, 0xEF, 0x2C, 0x5A, 0x40 *
 *   BW_500K BIT[45]:           Filter band width, 0:narrow, 1:wide
 *   GC_500K BIT[44]:           Filter gain control, 0:low, 1:high
 * 
 * 1B DEM_CAL2 BIT[23:0]        * 0x0B, 0xDF, 0x02 *
 *   
 * 
 * 1C DYNPD BIT[5:0]
 *   Enable dynamic payload length, each bit stands for one channel
 * 
 * 1D FEATURE BIT[6:0]
*/
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
/**
 * 1E RF_CAL BIT[23:0]          * 0xF6(0x06 for safety regulations test), 0x3F, 0x5D *
 *   EN_CLK_OUT BIT[23]:        OSC output to CLK_OUT, 1:ON, 0:OFF
 *   DA_VREF_MB BIT[22:20]:     DAC vref+, 111:high, 000:low
 *   DA_VREF_LB BIT[19:17]:     DAC vref-, 111:low, 000:high
 *   DA_LPF_CTRL BIT[16]:       DAC output amp, 1:0.8, 0:0.5
 *   RSSI_EN BIT[15]:           RSSI Enabled, 1:ON, 0:OFF
 *   RSSI_Gain_CTR BIT[14:13]:  RSSI gain control, 00:None, 01: -6dB, 10: -12dB, 11: -18dB
 *   MIXL_GC BIT[12]:           RX MIXL gain control, 1: 14dB, 0: 8dB
 *   PA_BC BIT[11:10]:          PA DC output current, 00: ×1, 01: ×2, 10: ×3, 11: ×4
 *   LNA_GC BIT[9:8]:           LNA gain control, 11: 17dB, 10: 11dB, 01: 5.4dB, 00: -0.4dB
 *   VCO_BIAS BIT[7:5]:         VCO current, 000:900uA, 001:1050uA, 010:1200uA, 011:1350uA, 100:1500uA, 101:1650uA, 110:1800uA, 111:1950uA
 *   RES_SEL BIT[4:3]:          Bias current resistor, 00: 26kR, 01: 24kR, 10: 22kR, 11: 20kR
 *   LNA_HCURR BIT[2]:          LNA high current enabled, 1:ON, 0:OFF
 *   MIXL_BC BIT[1]:            RX MIXL current, 1: ×1, 0: ×0.5
 *   IB_BPF_TRIM BIT[0]:        RX band pass filter current, 1: ×1, 0: ×0.5
 * 
 * 1F BB_CAL BIT[39:0]          * 0x12, 0xED, 0x67, 0x9C, 0x46 *
 *   Reserved BIT[39:32]:       Only 0X01000110 allowed
 *   INVERTER BIT[31]:          Whether to reverse the RX path data before entering RX Block, 1: reverse, 0: remain unchanged
 *   DAC_MODE BIT[30]:          The format of dac_out[5:0] for DAC input, 1:dac_out[5:0]<= [0:5], 0: dac_out[5:0]<= [5:0]
 *   DAC_BASAL BIT[29:24]:      The initial offset of DAC input
 *   TRX_TIME BIT[23:21]:       The time from sending carrier to sending data packet = TRX_TIME * 8 + 7.5
 *   EX_PA_TIME BIT[20:16]      The time from TX PLL enable to PA enable = EX_PA_TIME*16, the unit is us.
 *   TX_SETUP_TIME BIT[15:11]:  The time from PA enable to TX PLL Open = TX_SETUP_TIME*16, the unit is us
 *   RX_SETUP_TIME BIT[10:6]:   The time from RX PLL enable to RX enable = RX_SETUP_TIME*16, the unit is us
 *   RX_ACK_TIME BIT[5:0]:      The time from entering RX mode to waiting ACK = RX_ACK_TIME*16(2Mbps), RX_ACK_TIME*32(1Mbps), RX_ACK_TIME*128(250Kbps)
*/

#define XN297L_TEST_ADDR         "XN297"


/******************* FUNCTION DECLARE *******************/

uint8_t XN297L_WriteReg(uint8_t reg, uint8_t value);
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

ErrorStatus XN297L_TxFast(const uint8_t *ucPayload, uint8_t length);
void XN297L_ReuseTX(void);

uint8_t XN297L_PrintStatus(void);

#endif
