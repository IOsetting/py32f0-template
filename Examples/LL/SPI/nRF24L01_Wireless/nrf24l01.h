#ifndef _NRF24L01_H
#define _NRF24L01_H

#include <main.h>

// CE Pin & CSN Pin & IRQ Pin
#define CSN(x)      x ? LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6) : LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define CE(x)       x ? LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5) : LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define IRQ         READ_BIT(GPIOA->IDR, LL_GPIO_PIN_4)

// SPI(nRF24L01) commands
#define NRF24L01_CMD_REGISTER_R     0x00 // Register read
#define NRF24L01_CMD_REGISTER_W     0x20 // Register write
#define NRF24L01_CMD_ACTIVATE       0x50 // (De)Activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features
#define NRF24L01_CMD_RX_PLOAD_WID_R 0x60 // Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
#define NRF24L01_CMD_RX_PLOAD_R     0x61 // Read RX payload
#define NRF24L01_CMD_TX_PLOAD_W     0xA0 // Write TX payload
#define NRF24L01_CMD_ACK_PAYLOAD_W  0xA8 // Write ACK payload
#define NRF24L01_CMD_TX_PAYLOAD_NOACK_W 0xB0 //Write TX payload and disable AUTOACK
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
#define NRF24L01_FEATURE_EN_DYN_ACK 0x01 // EN_DYN_ACK bit in FEATURE register
#define NRF24L01_FEATURE_EN_ACK_PAY 0x02 // EN_ACK_PAY bit in FEATURE register
#define NRF24L01_FEATURE_EN_DPL     0x04 // EN_DPL bit in FEATURE register
#define NRF24L01_FLAG_RX_DREADY     0x40 // RX_DR bit (data ready RX FIFO interrupt)
#define NRF24L01_FLAG_TX_DSENT      0x20 // TX_DS bit (data sent TX FIFO interrupt)
#define NRF24L01_FLAG_MAX_RT        0x10 // MAX_RT bit (maximum number of TX re-transmits interrupt)

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
uint8_t   NRF24L01_Write_From_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len);

/**
* Hold till data received and written to rx_buf
*/
uint8_t   NRF24L01_RxPacket(uint8_t *rx_buf);
void      NRF24L01_IntRxPacket(uint8_t *rx_buf);

/**
* Send data in tx_buf and wait till data is sent or max re-tr reached
*/
uint8_t   NRF24L01_TxPacket(uint8_t *tx_buf, uint8_t len);

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
* Clear IRQ bit of the STATUS register
*   reg - NRF24L01_FLAG_RX_DREADY, NRF24L01_FLAG_TX_DSENT, NRF24L01_FLAG_MAX_RT
*/
void NRF24L01_ClearIRQFlag(uint8_t reg);

/**
* Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
*/
void NRF24L01_ClearIRQFlags(void);

#endif /*_NRF24L01_H*/
