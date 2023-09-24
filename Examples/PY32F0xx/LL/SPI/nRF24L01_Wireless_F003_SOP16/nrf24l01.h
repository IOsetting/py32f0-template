#ifndef _NRF24L01_H
#define _NRF24L01_H

#include <main.h>

// CE Pin & CSN Pin & IRQ Pin
#define CSN_HIGH    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define CSN_LOW     LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define CE_HIGH     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define CE_LOW      LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define IRQ         READ_BIT(GPIOA->IDR, LL_GPIO_PIN_4)

// SPI(nRF24L01) commands
#define NRF24L01_CMD_R_REGISTER     0x00 // Register read
#define NRF24L01_CMD_W_REGISTER     0x20 // Register write
/**
 * ACTIVATE. For NRF24L01 only, removed from NRF24L01+ product specs
 * - https://github.com/nRF24/RF24/issues/401
 *
 * This write command followed by data 0x73 activates the following features: R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK.
 * A new ACTIVATE command with the same data deactivates them again. This is executable in power down or stand by modes only.
*/
#define NRF24L01_CMD_ACTIVATE       0x50 // (De)Activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features
#define NRF24L01_CMD_R_RX_PL_WID    0x60 // Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO. Flush RX FIFO if the read value is larger than 32 bytes
#define NRF24L01_CMD_R_RX_PAYLOAD   0x61 // Read RX payload
#define NRF24L01_CMD_W_TX_PAYLOAD   0xA0 // Write TX payload
#define NRF24L01_CMD_W_ACK_PAYLOAD  0xA8 // 1010 1PPP, Used in RX mode. Write Payload to be transmitted together with ACK packet on PIPE PPP. (PPP valid in the range from 000 to 101)
#define NRF24L01_CMD_W_TX_PAYLOAD_NOACK 0xB0 // Used in TX mode. Disables AUTOACK on this specific packet
#define NRF24L01_CMD_FLUSH_TX       0xE1 // Flush TX FIFO
#define NRF24L01_CMD_FLUSH_RX       0xE2 // Flush RX FIFO
#define NRF24L01_CMD_REUSE_TX_PL    0xE3 // Reuse TX payload
#define NRF24L01_CMD_LOCK_UNLOCK    0x50 // Lock/unlock exclusive features
#define NRF24L01_CMD_NOP            0xFF // No operation (used for reading status register)

// SPI(nRF24L01) register address definitions
#define NRF24L01_REG_CONFIG         0x00 // Configuration register
#define NRF24L01_REG_EN_AA          0x01 // Enable "Auto acknowledgment"
#define NRF24L01_REG_EN_RXADDR      0x02 // Enable RX addresses
#define NRF24L01_REG_SETUP_AW       0x03 // Setup of address widths
#define NRF24L01_REG_SETUP_RETR     0x04 // Setup of automatic re-transmit
#define NRF24L01_REG_RF_CH          0x05 // RF channel
#define NRF24L01_REG_RF_SETUP       0x06 // RF setup
#define NRF24L01_REG_STATUS         0x07 // Status register
#define NRF24L01_REG_OBSERVE_TX     0x08 // Transmit observe register
#define NRF24L01_REG_RPD            0x09 // Received power detector
#define NRF24L01_REG_RX_ADDR_P0     0x0A // Receive address data pipe 0
#define NRF24L01_REG_RX_ADDR_P1     0x0B // Receive address data pipe 1
#define NRF24L01_REG_RX_ADDR_P2     0x0C // Receive address data pipe 2
#define NRF24L01_REG_RX_ADDR_P3     0x0D // Receive address data pipe 3
#define NRF24L01_REG_RX_ADDR_P4     0x0E // Receive address data pipe 4
#define NRF24L01_REG_RX_ADDR_P5     0x0F // Receive address data pipe 5
#define NRF24L01_REG_TX_ADDR        0x10 // Transmit address
#define NRF24L01_REG_RX_PW_P0       0x11 // Number of bytes in RX payload in data pipe 0
#define NRF24L01_REG_RX_PW_P1       0x12 // Number of bytes in RX payload in data pipe 1
#define NRF24L01_REG_RX_PW_P2       0x13 // Number of bytes in RX payload in data pipe 2
#define NRF24L01_REG_RX_PW_P3       0x14 // Number of bytes in RX payload in data pipe 3
#define NRF24L01_REG_RX_PW_P4       0x15 // Number of bytes in RX payload in data pipe 4
#define NRF24L01_REG_RX_PW_P5       0x16 // Number of bytes in RX payload in data pipe 5
#define NRF24L01_REG_FIFO_STATUS    0x17 // FIFO status register
#define NRF24L01_REG_DYNPD          0x1C // Enable dynamic payload length
#define NRF24L01_REG_FEATURE        0x1D // Feature register

// Register bits definitions
#define NRF24L01_CONFIG_PRIM_RX     0x01 // PRIM_RX bit in CONFIG register
#define NRF24L01_CONFIG_PWR_UP      0x02 // PWR_UP bit in CONFIG register

// Enable dynamic payload length on data pipes
#define NRF24L01_DYNPD_DPL_P5	    0x20
#define NRF24L01_DYNPD_DPL_P4	    0x10
#define NRF24L01_DYNPD_DPL_P3	    0x08
#define NRF24L01_DYNPD_DPL_P2	    0x04
#define NRF24L01_DYNPD_DPL_P1	    0x02
#define NRF24L01_DYNPD_DPL_P0	    0x01

/**
 * EN_DYN_ACK - this bit enables the W_TX_PAYLOAD_NOACK command
 *
 * The PTX can set the NO_ACK flag bit in the Packet Control Field with this command: W_TX_PAYLOAD_NOACK
 * However, the function must first be enabled in the FEATURE register by setting the EN_DYN_ACK bit.
 * When you use this option the PTX goes directly to standby-I mode after transmitting the packet. The PRX
 * does not transmit an ACK packet when it receives the packet.
*/
#define NRF24L01_FEATURE_EN_DYN_ACK 0x01 // EN_DYN_ACK bit in FEATURE register
/**
 * EN_ACK_PAY - Enables Payload with ACK
 * If ACK packet payload is activated, ACK packets have dynamic payload lengths and the Dynamic Payload
 * Length feature should be enabled for pipe 0 on the PTX and PRX. This is to ensure that they receive the
 * ACK packets with payloads. If the ACK payload is more than 15 byte in 2Mbps mode the ARD must be
 * 500µS or more, and if the ACK payload is more than 5 byte in 1Mbps mode the ARD must be 500µS or
 * more. In 250kbps mode (even when the payload is not in ACK) the ARD must be 500µS or more.
*/
#define NRF24L01_FEATURE_EN_ACK_PAY 0x02 // EN_ACK_PAY bit in FEATURE register
#define NRF24L01_FEATURE_EN_DPL     0x04 // EN_DPL bit in FEATURE register, enables Dynamic Payload Length

// Status Flags
#define NRF24L01_FLAG_RX_DR         0x40 // RX_DR bit (data ready RX FIFO interrupt)
#define NRF24L01_FLAG_TX_DS         0x20 // TX_DS bit (data sent TX FIFO interrupt)
#define NRF24L01_FLAG_MAX_RT        0x10 // MAX_RT bit (maximum number of TX re-transmits interrupt)
#define NRF24L01_FLAG_IT_BITS       0x70 // RX_DR|TX_DS|MAX_RT
#define NRF24L01_FLAG_TX_FULL       0x01 // TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO

// Register masks definitions
#define NRF24L01_MASK_REG_MAP       0x1F // Mask bits[4:0] for CMD_RREG and CMD_WREG commands
#define NRF24L01_MASK_CRC           0x0C // Mask for CRC bits [3:2] in CONFIG register
#define NRF24L01_MASK_STATUS_IRQ    0x70 // Mask for all IRQ bits in STATUS register
#define NRF24L01_MASK_RF_PWR        0x06 // Mask RF_PWR[2:1] bits in RF_SETUP register
#define NRF24L01_MASK_RX_P_NO       0x0E // Mask RX_P_NO[3:1] bits in STATUS register
#define NRF24L01_MASK_DATARATE      0x28 // Mask RD_DR_[5,3] bits in RF_SETUP register
#define NRF24L01_MASK_EN_RX         0x3F // Mask ERX_P[5:0] bits in EN_RXADDR register
#define NRF24L01_MASK_RX_PW         0x3F // Mask [5:0] bits in RX_PW_Px register
#define NRF24L01_MASK_RETR_ARD      0xF0 // Mask for ARD[7:4] bits in SETUP_RETR register
#define NRF24L01_MASK_RETR_ARC      0x0F // Mask for ARC[3:0] bits in SETUP_RETR register
#define NRF24L01_MASK_RXFIFO        0x03 // Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
#define NRF24L01_MASK_TXFIFO        0x30 // Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
#define NRF24L01_MASK_PLOS_CNT      0xF0 // Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
#define NRF24L01_MASK_ARC_CNT       0x0F // Mask for ARC_CNT[3:0] bits in OBSERVE_TX register

// Register masks definitions
#define NRF24L01_MASK_REG_MAP       0x1F // Mask bits[4:0] for CMD_RREG and CMD_WREG commands

// RXFIFO status
#define NRF24L01_RXFIFO_STATUS_DATA  0x00 // The RX FIFO contains data and available locations
#define NRF24L01_RXFIFO_STATUS_EMPTY 0x01 // The RX FIFO is empty
#define NRF24L01_RXFIFO_STATUS_FULL  0x02 // The RX FIFO is full
#define NRF24L01_RXFIFO_STATUS_ERROR 0x03 // Impossible state: RX FIFO cannot be empty and full at the same time

#define NRF24L01_ADDR_WIDTH         5    // RX/TX address width
#define NRF24L01_PLOAD_WIDTH        32   // Payload width
#define NRF24L01_TEST_ADDR          "nRF24"

void NRF24L01_Init(void);

uint8_t   NRF24L01_Check(void);

/**
* Dump nRF24L01+ configuration
*/
void NRF24L01_DumpConfig(void);

/**
* Read a 1-bit register
*/
uint8_t   NRF24L01_Read_Reg(uint8_t reg);

/**
* Write a 1-byte register
*/
uint8_t   NRF24L01_Write_Reg(uint8_t reg,uint8_t value);

/**
* Read a multi-byte register
*  reg - register to read
*  buf - pointer to the buffer to write
*  len - number of bytes to read
*/
uint8_t   NRF24L01_Read_To_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len);

/**
* Write a multi-byte register
*  reg - register to write
*  buf - pointer to the buffer with data
*  len - number of bytes to write
*/
uint8_t   NRF24L01_Write_From_Buf(uint8_t reg, const uint8_t *pBuf,uint8_t len);

void NRF24L01_SetEnableDynamicPayloads(uint8_t mode);

void NRF24L01_SetEnableAckPayload(uint8_t mode);

/**
 * Get status of the RX FIFO
*/
uint8_t NRF24L01_RXFIFO_GetStatus(void);

/**
 * Read received data (no fifo status check, no status clear, just read)
*/
uint8_t   NRF24L01_ReadPayload(uint8_t *pBuf, uint8_t *length, uint8_t dpl);

/**
* Send data in tx_buf and wait till data is sent or max re-tr reached
*/
uint8_t   NRF24L01_TxPacket(uint8_t *tx_buf, uint8_t len);

/**
 * Send data in FIFO without sync ack
*/
uint8_t NRF24L01_TxFast(const void *pBuf, uint8_t len);

/**
* Switch NRF24L01 to RX mode
*/
void NRF24L01_RX_Mode(uint8_t *rx_addr, uint8_t *tx_addr);

/**
* Switch NRF24L01 to TX mode
*/
void NRF24L01_TX_Mode(uint8_t *rx_addr, uint8_t *tx_addr);

/**
* Flush the RX FIFO
*/
void NRF24L01_FlushRX(void);

/**
* Flush the TX FIFO
*/
void NRF24L01_FlushTX(void);

/**
 * Clear TX error flags
*/
void NRF24L01_ResetTX(void);

/**
* Clear IRQ bit of the STATUS register
*   reg - NRF24L01_FLAG_RX_DREADY, NRF24L01_FLAG_TX_DSENT, NRF24L01_FLAG_MAX_RT
*/
void NRF24L01_ClearIRQFlag(uint8_t reg);

/**
* Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
*/
void NRF24L01_ClearIRQFlags(void);

#endif /*_NRF24L01_H*/
