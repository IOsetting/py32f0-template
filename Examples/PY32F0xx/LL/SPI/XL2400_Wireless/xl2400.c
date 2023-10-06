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

#include <stdio.h>
#include "xl2400.h"

uint8_t xl2400_state, cbuf[2], xbuf[XL2400_PL_WIDTH_MAX + 1];

uint8_t XL2400_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t reg_val;
    XL2400_NSS_LOW();
    SPI_TxRxByte(reg);
    reg_val = SPI_TxRxByte(value);
    XL2400_NSS_HIGH();
    return reg_val;
}

uint8_t XL2400_ReadReg(uint8_t reg)
{
    uint8_t reg_val;
    XL2400_NSS_LOW();
    SPI_TxRxByte(reg);
    reg_val = SPI_TxRxByte(XL2400_CMD_NOP);
    XL2400_NSS_HIGH();
    return reg_val;
}

void XL2400_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    XL2400_NSS_LOW();
    SPI_TxRxByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        SPI_TxRxByte(*pBuf++);
    }
    XL2400_NSS_HIGH();
}

void XL2400_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    XL2400_NSS_LOW();
    SPI_TxRxByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        pBuf[ctr] = SPI_TxRxByte(XL2400_CMD_NOP);
    }
    XL2400_NSS_HIGH();
}

void XL2400_CE_Low(void)
{
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
    *(cbuf + 1) &= 0xBF;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
}

void XL2400_CE_High(void)
{
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
    *(cbuf + 1) |= 0x40;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
}

ErrorStatus XL2400_SPI_Test(void)
{
    uint8_t i;
    const uint8_t *ptr = (const uint8_t *)XL2400_TEST_ADDR;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_TX_ADDR, ptr, 5);
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
        if (*(xbuf + i) != *ptr++) return ERROR;
    }
    return SUCCESS;
}

void XL2400_Init(void)
{
    // Analog config
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG0, xbuf, 13);
    *(xbuf + 4) &= ~0x04;
    *(xbuf + 12) |= 0x40;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG0, xbuf, 13);
    // Switch to software CE control, wake up RF
    XL2400_WakeUp();
    // Enable Auto ACK Pipe 0
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_EN_AA, 0x3F);
    // Enable Pipe 0
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_EN_RXADDR, 0x3F);
    // Address Width, 5 bytes
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_SETUP_AW, 0xAF);
    // Retries and interval
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_SETUP_RETR, 0x33);
    // RF Data Rate: 04:2Mbps, 00:1Mbps, 20:250Kbps, 24:125Kbps
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_RF_SETUP, 0x04);
    // Number of bytes in RX payload, pipe 0 and pipe 1
    *(cbuf + 0) = XL2400_PLOAD_WIDTH;
    *(cbuf + 1) = XL2400_PLOAD_WIDTH;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RX_PW_PX, cbuf, 2);
    // Dynamic payload width: off
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_DYNPD, 0x00);
    // Other features
    //bit7&6=00 return status when send register address
    //bit5=0 long data pack off
    //bit4=1 FEC off
    //bit3=1 FEATURE on
    //bit2=0 Dynamic length off
    //bit1=0 ACK without payload
    //bit0=0 W_TX_PAYLOAD_NOACK off
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_FEATURE, 0x18);
    // Enable RSSI
    *(cbuf + 0) = 0x10;
    *(cbuf + 1) = 0x00;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RSSI, cbuf, 2);
}

void XL2400_SetChannel(uint8_t channel)
{
    if (channel > 80) channel = 80;
    // AFC reset
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG0, 0x06);
    // AFC on
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG0, 0x0E);
    // Frequency(MHz) 2400:0x960 -> 2480:0x9B0
    *(cbuf + 0) = 0x60 + channel;
    *(cbuf + 1) = 0x09;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_CH, cbuf, 2);
    // AFC Locked
    *(cbuf + 1) |= 0x20;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_CH, cbuf, 2);
}

void XL2400_SetTxAddress(const uint8_t *address)
{
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_TX_ADDR, address, 5);
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RX_ADDR_P0, address, 5);
}

void XL2400_SetRxAddress(const uint8_t *address)
{
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RX_ADDR_P1, address, 5);
}

void XL2400_SetPower(uint8_t power)
{
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RF_CH, xbuf, 3);
    *(xbuf + 2) = power;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_CH, xbuf, 3);
}

void XL2400_Sleep(void)
{
    XL2400_CE_Low();
    XL2400_ClearStatus();

    *(xbuf + 0) = 0x7C;
    *(xbuf + 1) = 0x82;
    *(xbuf + 2) = 0x03;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, xbuf, 3);
}

void XL2400_WakeUp(void)
{
    *(xbuf + 0) = 0x7E;
    *(xbuf + 1) = 0x82;
    *(xbuf + 2) = 0x0B;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, xbuf, 3);
    XL2400_CE_Low();
    XL2400_ClearStatus();
}

ErrorStatus XL2400_RxCalibrate(void)
{
    uint8_t i, j;
    for (i = 0; i < 10; i++)
    {
        LL_mDelay(2);
        XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);
        *(cbuf + 1) |= 0x90;
        *(cbuf + 1) &= ~0x20;
        XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);
        *(cbuf + 1) |= 0x40;
        XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);
        LL_mDelay(1);
        XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_FIFO_STATUS, cbuf, 2);

        if (*(cbuf + 1) & 0x20)
        {
            j = *(cbuf + 1) << 3;
            XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);
            *(cbuf + 1) &= 0x8F;
            *(cbuf + 1) |= 0x20;
            *(cbuf + 0) &= 0x07;
            *(cbuf + 0) |= j;
            XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);
            return SUCCESS;
        }
    }
    return ERROR;
}

void XL2400_SetTxMode(void)
{
    XL2400_CE_Low();
    XL2400_ClearStatus();
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x7E);
    XL2400_RxCalibrate();
    LL_mDelay(2);
}

void XL2400_SetRxMode(void)
{
    XL2400_CE_Low();
    XL2400_ClearStatus();
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x7F);
    // XL2400_RxCalibrate();
    XL2400_CE_High();
    LL_mDelay(1);
}

ErrorStatus XL2400_TxFast(const uint8_t *ucPayload, uint8_t length)
{
    //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
    while ((XL2400_ReadStatus() & XL2400_FLAG_TX_FULL)) {
        if (xl2400_state & XL2400_FLAG_MAX_RT) {
            return ERROR;
        }
    }
    XL2400_WriteFromBuf(XL2400_CMD_W_TX_PAYLOAD, ucPayload, length);
    XL2400_CE_High();
    return SUCCESS;
}

void XL2400_ReuseTX(void)
{
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_STATUS, XL2400_FLAG_MAX_RT); //Clear max retry flag
    XL2400_CE_Low();
    XL2400_CE_High();
}

uint8_t XL2400_Tx(const uint8_t *ucPayload, uint8_t length)
{
    uint8_t y = 16, status = 0;
    XL2400_ClearStatus();
    XL2400_WriteFromBuf(XL2400_CMD_W_TX_PAYLOAD, ucPayload, length);
    XL2400_CE_High();
    // Retry until timeout, usually status will be retrievable in 10 ms 
    while (y--)
    {
        status = XL2400_ReadStatus();
        // If TX successful or retry timeout, exit
        if ((status & (XL2400_FLAG_MAX_RT | XL2400_FLAG_TX_DS)) != 0)
        {
            break;
        }
        // delay actually 1ms
        LL_mDelay(0);
    }
    return status;
}

uint8_t XL2400_Rx(void)
{
    uint8_t status, rxplWidth;
    /*
     * Use XL2400_WriteReg() can get status and clear it at the same time
       but in the test this method takes much longer, so I still use XL2400_ReadStatus()
       
       status = XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_STATUS, 0x70);
    */
    status = XL2400_ReadStatus();
    if (status & XL2400_FLAG_RX_DR)
    {
        //XL2400_CE_Low();
        XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_STATUS, status);
        rxplWidth = XL2400_ReadReg(XL2400_CMD_R_RX_PL_WID);
        XL2400_ReadToBuf(XL2400_CMD_R_RX_PAYLOAD, xbuf, rxplWidth);
    }
    return status;
}

uint8_t XL2400_ReadStatus(void)
{
    xl2400_state = XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_STATUS);
    return xl2400_state;
}

void XL2400_ClearStatus(void)
{
    XL2400_WriteReg(XL2400_CMD_FLUSH_TX, XL2400_CMD_NOP);
    XL2400_WriteReg(XL2400_CMD_FLUSH_RX, XL2400_CMD_NOP);
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_STATUS, 0x70);
}

void XL2400_FlushRxTX(void)
{
    XL2400_WriteReg(XL2400_CMD_FLUSH_TX, XL2400_CMD_NOP);
    XL2400_WriteReg(XL2400_CMD_FLUSH_RX, XL2400_CMD_NOP);
}

void XL2400_CarrierTest(void)
{
    XL2400_CE_Low();
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG0, xbuf, 13);
    *(xbuf + 12) |= 0x40;
    *(xbuf + 4) &= ~0x04;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG0, xbuf, 13);
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_TXPROC_CFG, 0x00);
    *(xbuf + 0) = 0x01;
    *(xbuf + 1) = 0x00;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_SETUP, xbuf, 2);
    XL2400_ClearStatus();
}

uint8_t XL2400_PrintStatus(void)
{
    uint8_t i, status;

    printf("Bytes from low to high: 0,1,2,3,...\r\n[Config]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_CFG_TOP, xbuf, 3);
    for (i = 0; i < 3; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("  [EN_AA]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_EN_AA));

    printf("  [EN_RxAddr]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_EN_RXADDR));

    printf("  [AddrWidth]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_SETUP_AW));

    printf("  [Retry]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_SETUP_RETR, xbuf, 4);
    for (i = 0; i < 4; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[RF_Channel]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RF_CH, xbuf, 3);
    for (i = 0; i < 3; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("  [RF_Setup]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RF_SETUP, xbuf, 2);
    for (i = 0; i < 2; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("  [Observe_Tx]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_OBSERVE_TX, xbuf, 4);
    for (i = 0; i < 4; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("  [RSSI]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RSSI, xbuf, 2);
    for (i = 0; i < 2; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[TxAddr]  ");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[RxAddrP0]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RX_ADDR_P0, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [RxAddrP1]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RX_ADDR_P1, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [RxAddrP2-P5]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RX_ADDR_P2_P5, xbuf, 4);
    for (i = 0; i < 4; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[RxPloadWidth_P0-P5]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RX_PW_PX, xbuf, 6);
    for (i = 0; i < 6; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[FIFO_Status]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_FIFO_STATUS, xbuf, 3);
    for (i = 0; i < 3; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf("  [DynPloadWidth]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_DYNPD));
    printf("  [Feature]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_FEATURE));

    status = XL2400_ReadStatus();
    printf("\r\n[Status]");
    printf("%02X", status);
    printf("\r\n\r\n");
    return status;
}
