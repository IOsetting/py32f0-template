#include <stdio.h>
#include "nrf24l01.h"
#include "py32f0xx_bsp_printf.h"


uint8_t RX_BUF[NRF24L01_PLOAD_WIDTH];
uint8_t TX_BUF[NRF24L01_PLOAD_WIDTH];

void NRF24L01_Init(void)
{
    CSN_HIGH;
}

/**
* Read a 1-bit register
*/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t value;
    CSN_LOW;
    SPI_TxRxByte(reg);
    value = SPI_TxRxByte(NRF24L01_CMD_NOP);
    CSN_HIGH;
    return value;
}

/**
* Write a 1-byte register
*/
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    CSN_LOW;
    if (reg < NRF24L01_CMD_W_REGISTER)
    {
        // This is a register access
        status = SPI_TxRxByte(NRF24L01_CMD_W_REGISTER | (reg & NRF24L01_MASK_REG_MAP));
        SPI_TxRxByte(value);
    }
    else
    {
        // This is a single byte command or future command/register
        status = SPI_TxRxByte(reg);
        if ((reg != NRF24L01_CMD_FLUSH_TX) 
            && (reg != NRF24L01_CMD_FLUSH_RX) 
            && (reg != NRF24L01_CMD_REUSE_TX_PL) 
            && (reg != NRF24L01_CMD_NOP)) {
            // Send register value
            SPI_TxRxByte(value);
        }
    }
    CSN_HIGH;
    return status; 
}

/**
* Read a multi-byte register
*  reg  - register to read
*  buf  - pointer to the buffer to write
*  len  - number of bytes to read
*/
uint8_t NRF24L01_Read_To_Buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    CSN_LOW;
    uint8_t status = SPI_TxRxByte(reg);
    while (len--)
    {
        *buf++ = SPI_TxRxByte(NRF24L01_CMD_NOP);
    }
    CSN_HIGH;
    return status;
}

/**
* Write a multi-byte register
*  reg - register to write
*  buf - pointer to the buffer with data
*  len - number of bytes to write
*/
uint8_t NRF24L01_Write_From_Buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    CSN_LOW;
    uint8_t status = SPI_TxRxByte(reg);
    while (len--)
    {
        SPI_TxRxByte(*buf++);
    }
    CSN_HIGH;
    return status;
}


uint8_t NRF24L01_Check(void)
{
    uint8_t rxbuf[5];
    uint8_t i;
    uint8_t *ptr = (uint8_t *)NRF24L01_TEST_ADDR;

    // Write test TX address and read TX_ADDR register
    NRF24L01_Write_From_Buf(NRF24L01_CMD_W_REGISTER | NRF24L01_REG_TX_ADDR, ptr, 5);
    NRF24L01_Read_To_Buf(NRF24L01_CMD_R_REGISTER | NRF24L01_REG_TX_ADDR, rxbuf, 5);

    // Compare buffers, return error on first mismatch
    for (i = 0; i < 5; i++)
    {
        if (rxbuf[i] != *ptr++) return 1;
    }
    return 0;
}

/**
* Flush the RX FIFO
*/
void NRF24L01_FlushRX(void)
{
    NRF24L01_Write_Reg(NRF24L01_CMD_FLUSH_RX, NRF24L01_CMD_NOP);
}

/**
* Flush the TX FIFO
*/
void NRF24L01_FlushTX(void)
{
    NRF24L01_Write_Reg(NRF24L01_CMD_FLUSH_TX, NRF24L01_CMD_NOP);
}

void NRF24L01_ResetTX(void)
{
    NRF24L01_Write_Reg(NRF24L01_REG_STATUS, NRF24L01_FLAG_MAX_RT); // Clear max retry flag
    CE_LOW;
    CE_HIGH;
}

/**
* Clear IRQ bit of the STATUS register
*   reg - NRF24L01_FLAG_RX_DREADY
*         NRF24L01_FLAG_TX_DSENT
*         NRF24L01_FLAG_MAX_RT
*/
void NRF24L01_ClearIRQFlag(uint8_t reg)
{
    NRF24L01_Write_Reg(NRF24L01_REG_STATUS, reg);
}

/**
* Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
*/
void NRF24L01_ClearIRQFlags(void) 
{
    uint8_t reg;
    reg  = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    reg |= NRF24L01_MASK_STATUS_IRQ;
    NRF24L01_Write_Reg(NRF24L01_REG_STATUS, reg);
}

/**
* Common configurations of RX and TX, internal function
*/
void _NRF24L01_Config(uint8_t *tx_addr)
{
    // TX Address
    NRF24L01_Write_From_Buf(NRF24L01_CMD_W_REGISTER + NRF24L01_REG_TX_ADDR, tx_addr, NRF24L01_ADDR_WIDTH);
    // RX P0 Payload Width
    NRF24L01_Write_Reg(NRF24L01_REG_RX_PW_P0, NRF24L01_PLOAD_WIDTH);
    // Enable Auto ACK
    NRF24L01_Write_Reg(NRF24L01_REG_EN_AA, 0x3f);
    // Enable RX channels
    NRF24L01_Write_Reg(NRF24L01_REG_EN_RXADDR, 0x3f);
    // RF channel: 2.400G  + 0.001 * x
    NRF24L01_Write_Reg(NRF24L01_REG_RF_CH, 40);
    // Format: 000+0+[0:1Mbps,1:2Mbps]+[00:-18dbm,01:-12dbm,10:-6dbm,11:0dbm]+[0:LNA_OFF,1:LNA_ON]
    // 01:1Mbps,-18dbm; 03:1Mbps,-12dbm; 05:1Mbps,-6dbm; 07:1Mbps,0dBm
    // 09:2Mbps,-18dbm; 0b:2Mbps,-12dbm; 0d:2Mbps,-6dbm; 0f:2Mbps,0dBm, 
    NRF24L01_Write_Reg(NRF24L01_REG_RF_SETUP, 0x03);
    // 0A:delay=250us,count=10, 1A:delay=500us,count=10
    NRF24L01_Write_Reg(NRF24L01_REG_SETUP_RETR, 0x0a);
}

/**
* Switch NRF24L01 to RX mode
*/
void NRF24L01_RX_Mode(uint8_t *rx_addr, uint8_t *tx_addr)
{
    CE_LOW;
    _NRF24L01_Config(tx_addr);
    // RX Address of P0
    NRF24L01_Write_From_Buf(NRF24L01_CMD_W_REGISTER + NRF24L01_REG_RX_ADDR_P0, rx_addr, NRF24L01_ADDR_WIDTH);
    /**
    REG 0x00: 
    0)PRIM_RX     0:TX             1:RX
    1)PWR_UP      0:OFF            1:ON
    2)CRCO        0:8bit CRC       1:16bit CRC
    3)EN_CRC      Enabled if any of EN_AA is high
    4)MASK_MAX_RT 0:IRQ low        1:NO IRQ
    5)MASK_TX_DS  0:IRQ low        1:NO IRQ
    6)MASK_RX_DR  0:IRQ low        1:NO IRQ
    7)Reserved    0
    */
    NRF24L01_Write_Reg(NRF24L01_REG_CONFIG, 0x0f); //RX,PWR_UP,CRC16,EN_CRC
    CE_HIGH;
    NRF24L01_FlushRX();
}

/**
* Switch NRF24L01 to TX mode
*/
void NRF24L01_TX_Mode(uint8_t *rx_addr, uint8_t *tx_addr)
{
    CE_LOW;
    _NRF24L01_Config(tx_addr);
    // On the PTX the **TX_ADDR** must be the same as the **RX_ADDR_P0** and as the pipe address for the designated pipe
    // RX_ADDR_P0 will be used for receiving ACK
    NRF24L01_Write_From_Buf(NRF24L01_CMD_W_REGISTER + NRF24L01_REG_RX_ADDR_P0, tx_addr, NRF24L01_ADDR_WIDTH);
    NRF24L01_Write_Reg(NRF24L01_REG_CONFIG, 0x0e); //TX,PWR_UP,CRC16,EN_CRC
    CE_HIGH;
}

uint8_t NRF24L01_RX_GetPayloadWidth(void)
{
    uint8_t value;
    CSN_LOW;
    value = NRF24L01_Read_Reg(NRF24L01_CMD_R_RX_PL_WID);
    CSN_HIGH;
    return value;
}

uint8_t NRF24L01_RXFIFO_GetStatus(void)
{
    uint8_t reg = NRF24L01_Read_Reg(NRF24L01_REG_FIFO_STATUS);
	return (reg & NRF24L01_MASK_RXFIFO);
}

uint8_t NRF24L01_ReadPayload(uint8_t *pBuf, uint8_t *length, uint8_t dpl)
{
    uint8_t status, pipe;

    // Extract a payload pipe number from the STATUS register
    status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    pipe = (status & NRF24L01_MASK_RX_P_NO) >> 1;
    // RX FIFO empty?
    if (pipe < 6)
    {
        if (dpl)
        {
            // Get payload width
            *length = NRF24L01_RX_GetPayloadWidth();
            if (*length > 32)
            {
                // Error
                *length = 0;
                NRF24L01_FlushRX();
            }
        }
        else
        {
            *length = NRF24L01_Read_Reg(NRF24L01_REG_RX_PW_P0 + pipe);
        }
        // Read a payload from the RX FIFO
        if (*length)
        {
            NRF24L01_Read_To_Buf(NRF24L01_CMD_R_RX_PAYLOAD, pBuf, *length);
        }
        return pipe;
    }
    // pipe value = 110: Not Used, 111: RX FIFO Empty
    *length = 0;
    return pipe;
}

/**
* Send data in tx_buf and wait till data is sent or max re-tr reached
*/
uint8_t NRF24L01_TxPacket(uint8_t *tx_buf, uint8_t len)
{
    uint8_t status = 0x00;
    CE_LOW;
    len = len > NRF24L01_PLOAD_WIDTH? NRF24L01_PLOAD_WIDTH : len;
    NRF24L01_Write_From_Buf(NRF24L01_CMD_W_TX_PAYLOAD, tx_buf, len);
    CE_HIGH;
    while(IRQ != 0); // Waiting send finish

    CE_LOW;
    status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    BSP_UART_TxHex8(status);
    BSP_UART_TxChar(':');
    if(status & NRF24L01_FLAG_TX_DS)
    {
        BSP_UART_TxString("Data sent: ");
        for (uint8_t i = 0; i < len; i++) {
            BSP_UART_TxHex8(tx_buf[i]);
        }
        BSP_UART_TxString("\r\n");
        NRF24L01_ClearIRQFlag(NRF24L01_FLAG_TX_DS);
    } 
    else if(status & NRF24L01_FLAG_MAX_RT) 
    {
        BSP_UART_TxString("Sending exceeds max retries\r\n");
        NRF24L01_FlushTX();
        NRF24L01_ClearIRQFlag(NRF24L01_FLAG_MAX_RT);
    }
    CE_HIGH;
    return status;
}

void NRF24L01_TxPacketFast(const void *pBuf, uint8_t len)
{
    NRF24L01_Write_From_Buf(NRF24L01_CMD_W_TX_PAYLOAD, pBuf, len);
    CE_HIGH;
}

uint8_t NRF24L01_TxFast(const void *pBuf, uint8_t len)
{
    uint8_t status;
    // Blocking only if FIFO is full. This will loop and block until TX is successful or fail
    do
    {
        status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
        if (status & NRF24L01_FLAG_MAX_RT)
        {
            return 1;
        }

    } while (status & NRF24L01_FLAG_TX_FULL);
    NRF24L01_TxPacketFast(pBuf, len);
    return 0;
}

void NRF24L01_ToggleFeatures(void)
{
    CSN_LOW;
    NRF24L01_Write_Reg(NRF24L01_CMD_ACTIVATE, 0x73);
    CSN_HIGH;
}

void NRF24L01_SetEnableDynamicPayloads(uint8_t mode)
{
    uint8_t reg = NRF24L01_Read_Reg(NRF24L01_REG_FEATURE);
    if (mode == 0)
    {
        // Disable dynamic payload throughout the system
        NRF24L01_Write_Reg(NRF24L01_REG_FEATURE, reg & (~NRF24L01_FEATURE_EN_DPL));
        // If it didn't work, the features are not enabled
        reg = NRF24L01_Read_Reg(NRF24L01_REG_FEATURE);
        if ((reg & NRF24L01_FEATURE_EN_DPL) != 0)
        {
            // Enable them and try again
            NRF24L01_ToggleFeatures();
            NRF24L01_Write_Reg(NRF24L01_REG_FEATURE, reg & (~NRF24L01_FEATURE_EN_DPL));
        }
        // Disable dynamic payload on all pipes
        NRF24L01_Write_Reg(NRF24L01_REG_DYNPD, 0);
    }
    else
    {
        NRF24L01_Write_Reg(NRF24L01_REG_FEATURE, reg | NRF24L01_FEATURE_EN_DPL);
        reg = NRF24L01_Read_Reg(NRF24L01_REG_FEATURE);
        if ((reg & NRF24L01_FEATURE_EN_DPL) != NRF24L01_FEATURE_EN_DPL)
        {
            NRF24L01_ToggleFeatures();
            NRF24L01_Write_Reg(NRF24L01_REG_FEATURE, reg | NRF24L01_FEATURE_EN_DPL);
        }
        // Enable dynamic payload on all pipes
        NRF24L01_Write_Reg(NRF24L01_REG_DYNPD, NRF24L01_DYNPD_DPL_P0 | NRF24L01_DYNPD_DPL_P1
            | NRF24L01_DYNPD_DPL_P2 | NRF24L01_DYNPD_DPL_P3 | NRF24L01_DYNPD_DPL_P4 | NRF24L01_DYNPD_DPL_P5);
    }
    
}

void NRF24L01_SetEnableAckPayload(uint8_t mode)
{
    uint8_t reg = NRF24L01_Read_Reg(NRF24L01_REG_FEATURE);
    if (mode == 0)
    {
        // Disable ack payload and dynamic payload features
        NRF24L01_Write_Reg(NRF24L01_REG_FEATURE, reg &(~NRF24L01_FEATURE_EN_ACK_PAY));
        // If it didn't work, the features are not enabled
        reg = NRF24L01_Read_Reg(NRF24L01_REG_FEATURE);
        if ((reg & NRF24L01_FEATURE_EN_ACK_PAY) != NRF24L01_FEATURE_EN_ACK_PAY)
        {
            // Enable them and try again
            NRF24L01_ToggleFeatures();
            NRF24L01_Write_Reg(NRF24L01_REG_FEATURE, reg &(~NRF24L01_FEATURE_EN_ACK_PAY));
        }
    }
    else
    {
        NRF24L01_Write_Reg(NRF24L01_REG_FEATURE, reg | NRF24L01_FEATURE_EN_ACK_PAY);
        reg = NRF24L01_Read_Reg(NRF24L01_REG_FEATURE);
        if ((reg & NRF24L01_FEATURE_EN_ACK_PAY) != NRF24L01_FEATURE_EN_ACK_PAY)
        {
            NRF24L01_ToggleFeatures();
            NRF24L01_Write_Reg(NRF24L01_REG_FEATURE, reg | NRF24L01_FEATURE_EN_ACK_PAY);
        }
        // Enable dynamic payload on pipes 0 & 1
        reg = NRF24L01_Read_Reg(NRF24L01_REG_DYNPD);
        NRF24L01_Write_Reg(NRF24L01_REG_DYNPD, reg | NRF24L01_DYNPD_DPL_P0 | NRF24L01_DYNPD_DPL_P1);
    }
}

/**
* Dump nRF24L01 configuration
*/
void NRF24L01_DumpConfig(void) {
    uint8_t i,j;
    uint8_t aw;
    uint8_t buf[5];

    // CONFIG
    i = NRF24L01_Read_Reg(NRF24L01_REG_CONFIG);
    printf("[0x%02X] 0x%02X MASK:%02X CRC:%02X PWR:%s MODE:P%s\r\n",
        NRF24L01_REG_CONFIG,
        i,
        i >> 4,
        (i & 0x0c) >> 2,
        (i & 0x02) ? "ON" : "OFF",
        (i & 0x01) ? "RX" : "TX"
      );
    // EN_AA
    i = NRF24L01_Read_Reg(NRF24L01_REG_EN_AA);
    printf("[0x%02X] 0x%02X ENAA: ",NRF24L01_REG_EN_AA,i);
    for (j = 0; j < 6; j++) {
      printf("[P%1u%s]%s",j,
          (i & (1 << j)) ? "+" : "-",
          (j == 5) ? "\r\n" : " "
        );
    }
    // EN_RXADDR
    i = NRF24L01_Read_Reg(NRF24L01_REG_EN_RXADDR);
    printf("[0x%02X] 0x%02X EN_RXADDR: ",NRF24L01_REG_EN_RXADDR,i);
    for (j = 0; j < 6; j++) {
      printf("[P%1u%s]%s",j,
          (i & (1 << j)) ? "+" : "-",
          (j == 5) ? "\r\n" : " "
        );
    }
    // SETUP_AW
    i = NRF24L01_Read_Reg(NRF24L01_REG_SETUP_AW);
    aw = (i & 0x03) + 2;
    printf("[0x%02X] 0x%02X EN_RXADDR=%03X (address width = %u)\r\n",NRF24L01_REG_SETUP_AW,i,i & 0x03,aw);
    // SETUP_RETR
    i = NRF24L01_Read_Reg(NRF24L01_REG_SETUP_RETR);
    printf("[0x%02X] 0x%02X ARD=%04X ARC=%04X (retr.delay=%uus, count=%u)\r\n",
        NRF24L01_REG_SETUP_RETR,
        i,
        i >> 4,
        i & 0x0F,
        ((i >> 4) * 250) + 250,
        i & 0x0F
      );
    // RF_CH
    i = NRF24L01_Read_Reg(NRF24L01_REG_RF_CH);
    printf("[0x%02X] 0x%02X (%.3uGHz)\r\n",NRF24L01_REG_RF_CH,i,2400 + i);
    // RF_SETUP
    i = NRF24L01_Read_Reg(NRF24L01_REG_RF_SETUP);
    printf("[0x%02X] 0x%02X CONT_WAVE:%s PLL_LOCK:%s DataRate=",
        NRF24L01_REG_RF_SETUP,
        i,
        (i & 0x80) ? "ON" : "OFF",
        (i & 0x80) ? "ON" : "OFF"
      );
    switch ((i & 0x28) >> 3) {
      case 0x00:
        printf("1M");
        break;
      case 0x01:
        printf("2M");
        break;
      case 0x04:
        printf("250k");
        break;
      default:
        printf("???");
        break;
    }
    printf("pbs RF_PWR=");
    switch ((i & 0x06) >> 1) {
      case 0x00:
        printf("-18");
        break;
      case 0x01:
        printf("-12");
        break;
      case 0x02:
        printf("-6");
        break;
      case 0x03:
        printf("0");
        break;
      default:
        printf("???");
        break;
    }
    printf("dBm\r\n");
    // STATUS
    i = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    printf("[0x%02X] 0x%02X IRQ:%03X RX_PIPE:%u TX_FULL:%s\r\n",
        NRF24L01_REG_STATUS,
        i,
        (i & 0x70) >> 4,
        (i & 0x0E) >> 1,
        (i & 0x01) ? "YES" : "NO"
      );

    // OBSERVE_TX
    i = NRF24L01_Read_Reg(NRF24L01_REG_OBSERVE_TX);
    printf("[0x%02X] 0x%02X PLOS_CNT=%u ARC_CNT=%u\r\n",NRF24L01_REG_OBSERVE_TX,i,i >> 4,i & 0x0F);

    // RPD
    i = NRF24L01_Read_Reg(NRF24L01_REG_RPD);
    printf("[0x%02X] 0x%02X RPD=%s\r\n",NRF24L01_REG_RPD,i,(i & 0x01) ? "YES" : "NO");

    // RX_ADDR_P0
    NRF24L01_Read_To_Buf(NRF24L01_REG_RX_ADDR_P0,buf,aw);
    printf("[0x%02X] RX_ADDR_P0 \"",NRF24L01_REG_RX_ADDR_P0);
    for (i = 0; i < aw; i++) printf("%X ",buf[i]);
    printf("\"\r\n");

    // RX_ADDR_P1
    NRF24L01_Read_To_Buf(NRF24L01_REG_RX_ADDR_P1,buf,aw);
    printf("[0x%02X] RX_ADDR_P1 \"",NRF24L01_REG_RX_ADDR_P1);
    for (i = 0; i < aw; i++) printf("%X ",buf[i]);
    printf("\"\r\n");

    // RX_ADDR_P2
    printf("[0x%02X] RX_ADDR_P2 \"",NRF24L01_REG_RX_ADDR_P2);
    for (i = 0; i < aw - 1; i++) printf("%X ",buf[i]);
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_ADDR_P2);
    printf("%X\"\r\n",i);

    // RX_ADDR_P3
    printf("[0x%02X] RX_ADDR_P3 \"",NRF24L01_REG_RX_ADDR_P3);
    for (i = 0; i < aw - 1; i++) printf("%X ",buf[i]);
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_ADDR_P3);
    printf("%X\"\r\n",i);

    // RX_ADDR_P4
    printf("[0x%02X] RX_ADDR_P4 \"",NRF24L01_REG_RX_ADDR_P4);
    for (i = 0; i < aw - 1; i++) printf("%X ",buf[i]);
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_ADDR_P4);
    printf("%X\"\r\n",i);

    // RX_ADDR_P5
    printf("[0x%02X] RX_ADDR_P5 \"",NRF24L01_REG_RX_ADDR_P5);
    for (i = 0; i < aw - 1; i++) printf("%X ",buf[i]);
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_ADDR_P5);
    printf("%X\"\r\n",i);

    // TX_ADDR
    NRF24L01_Read_To_Buf(NRF24L01_REG_TX_ADDR,buf,aw);
    printf("[0x%02X] TX_ADDR \"",NRF24L01_REG_TX_ADDR);
    for (i = 0; i < aw; i++) printf("%X ",buf[i]);
    printf("\"\r\n");

    // RX_PW_P0
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_PW_P0);
    printf("[0x%02X] RX_PW_P0=%u\r\n",NRF24L01_REG_RX_PW_P0,i);

    // RX_PW_P1
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_PW_P1);
    printf("[0x%02X] RX_PW_P1=%u\r\n",NRF24L01_REG_RX_PW_P1,i);

    // RX_PW_P2
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_PW_P2);
    printf("[0x%02X] RX_PW_P2=%u\r\n",NRF24L01_REG_RX_PW_P2,i);

    // RX_PW_P3
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_PW_P3);
    printf("[0x%02X] RX_PW_P3=%u\r\n",NRF24L01_REG_RX_PW_P3,i);

    // RX_PW_P4
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_PW_P4);
    printf("[0x%02X] RX_PW_P4=%u\r\n",NRF24L01_REG_RX_PW_P4,i);

    // RX_PW_P5
    i = NRF24L01_Read_Reg(NRF24L01_REG_RX_PW_P5);
    printf("[0x%02X] RX_PW_P5=%u\r\n",NRF24L01_REG_RX_PW_P5,i);

    // NRF24L01_REG_FIFO_STATUS
    i = NRF24L01_Read_Reg(NRF24L01_REG_FIFO_STATUS);
    printf("[0x%02X] FIFO_STATUS=0x%02x\r\n",NRF24L01_REG_FIFO_STATUS,i);

    // NRF24L01_REG_DYNPD
    i = NRF24L01_Read_Reg(NRF24L01_REG_DYNPD);
    printf("[0x%02X] DYNPD=0x%02x\r\n",NRF24L01_REG_DYNPD,i);

    // NRF24L01_REG_FEATURE
    i = NRF24L01_Read_Reg(NRF24L01_REG_FEATURE);
    printf("[0x%02X] FEATURE=0x%02x\r\n",NRF24L01_REG_FEATURE,i);
}
