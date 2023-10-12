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
#include "xn297l.h"

const uint8_t 
    BB_cal_data[]    = {0x12,0xED,0x67,0x9C,0x46},
    RF_cal_data[]    = {0xF6,0x3F,0x5D},
    RF_cal2_data[]   = {0x45,0x21,0xEF,0x2C,0x5A,0x42},
    Dem_cal_data[]   = {0x01},
    Dem_cal2_data[]  = {0x0B,0xDF,0x02};

uint8_t xn297l_state, cbuf[2], xbuf[XN297L_PLOAD_WIDTH + 1];


uint8_t XN297L_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t reg_val;
    XN297L_CSN_LOW();
    SPI_TxRxByte(reg);
    reg_val = SPI_TxRxByte(value);
    XN297L_CSN_HIGH();
    return reg_val;
}

uint8_t XN297L_ReadReg(uint8_t reg)
{
    uint8_t reg_val;
    XN297L_CSN_LOW();
    SPI_TxRxByte(reg);
    reg_val = SPI_TxRxByte(XN297L_CMD_NOP);
    XN297L_CSN_HIGH();
    return reg_val;
}

void XN297L_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    XN297L_CSN_LOW();
    SPI_TxRxByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        SPI_TxRxByte(*pBuf++);
    }
    XN297L_CSN_HIGH();
}

void XN297L_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    XN297L_CSN_LOW();
    SPI_TxRxByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        pBuf[ctr] = SPI_TxRxByte(XN297L_CMD_NOP);
    }
    XN297L_CSN_HIGH();
}

void XN297L_Init(void)
{
    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0x5A); // Soft reset
    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0XA5);

    XN297L_WriteReg(XN297L_CMD_FLUSH_TX, 0);
    XN297L_WriteReg(XN297L_CMD_FLUSH_RX, 0);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, 0x70);       // Clear status flags
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_EN_AA, 0x3F);        // AutoAck on all pipes
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_EN_RXADDR, 0x3F);    // Enable all pipes (P0 ~ P5, bit0 ~ bit5)
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_SETUP_AW, XN297L_SETUP_AW_5BYTE); // Address width
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CH, 78);          // Channel 78, 2478M HZ
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P0, XN297L_PLOAD_WIDTH ); // Payload width of P0
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P1, XN297L_PLOAD_WIDTH ); // Payload width of P1
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P2, XN297L_PLOAD_WIDTH ); // Payload width of P2
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P3, XN297L_PLOAD_WIDTH ); // Payload width of P3
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P4, XN297L_PLOAD_WIDTH ); // Payload width of P4
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P5, XN297L_PLOAD_WIDTH ); // Payload width of P5

    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_DYNPD, 0x00); // Dynamic payload width: off
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_SETUP,  XN297L_RF_POWER_P_9|XN297L_RF_DR_1M); // 9dbm 1Mbps
    XN297L_WriteReg(XN297L_CMD_ACTIVATE, 0x73);

    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_SETUP_RETR, 0x10|0x05); // Retry interval 500µs, 5 times

    if(XN297L_PLOAD_WIDTH >32)
    {
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_FEATURE, XN297L_FEATURE_BIT5_CE_SOFT|XN297L_FEATURE_BIT43_DATA_64BYTE);
    }
    else
    {
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_FEATURE, XN297L_FEATURE_BIT5_CE_SOFT);
    }
}

ErrorStatus XN297L_SPI_Test(void)
{
    uint8_t i;
    const uint8_t *ptr = (const uint8_t *)XN297L_TEST_ADDR;
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_TX_ADDR, ptr, 5);
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
        if (*(xbuf + i) != *ptr++) return ERROR;
    }
    return SUCCESS;
}

void XN297L_SetTxAddress(const uint8_t *address)
{
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_TX_ADDR, address, 5);
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RX_ADDR_P0, address, 5);
}

void XN297L_SetRxAddress(const uint8_t *address)
{
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RX_ADDR_P1, address, 5);
}

void XN297L_SetChannel(uint8_t channel)
{    
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CH, channel);
}

void XN297L_SetTxMode(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG,  0X8E);
    XN297L_CE_LOW(); 
}

void XN297L_SetRxMode(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG,  0X8F);
    LL_mDelay(10);   
    XN297L_CE_HIGH();	
}

uint8_t XN297L_ReadStatus(void)
{
    xn297l_state = XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_STATUS);
    return xn297l_state;
}

void XN297L_ClearStatus(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, 0x70);
}

ErrorStatus XN297L_TxFast(const uint8_t *ucPayload, uint8_t length)
{
    //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
    while ((XN297L_ReadStatus() & XN297L_FLAG_TX_FULL)) {
        if (xn297l_state & XN297L_FLAG_MAX_RT) {
            return ERROR;
        }
    }
    XN297L_WriteFromBuf(XN297L_CMD_W_TX_PAYLOAD, ucPayload, length);
    XN297L_CE_HIGH();
    return SUCCESS;
}

void XN297L_ReuseTX(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, XN297L_FLAG_MAX_RT); //Clear max retry flag
    XN297L_CE_LOW();
    XN297L_CE_HIGH();
}

uint8_t XN297L_TxData(uint8_t *ucPayload, uint8_t length)
{
    uint8_t y = 100, status = 0;
    XN297L_CE_HIGH();
    __NOP();
    XN297L_WriteFromBuf(XN297L_CMD_W_TX_PAYLOAD, ucPayload, length);
    // Retry until timeout
    while (y--)
    {
        LL_mDelay(1);
        status = XN297L_ReadStatus();
        // If TX successful or retry timeout, exit
        if ((status & (XN297L_FLAG_MAX_RT | XN297L_FLAG_TX_DS)) != 0)
        {
            //printf(" %d %02x\r\n", y, status);
            break;
        }
    }
    XN297L_WriteReg(XN297L_CMD_FLUSH_TX, 0);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, 0x70);
    XN297L_CE_LOW();
    return status;
}

uint8_t XN297L_DumpRxData(void)
{
    uint8_t status, rxplWidth;
    status = XN297L_ReadStatus();
    if (status & XN297L_FLAG_RX_DR)
    {
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, status);
        rxplWidth = XN297L_ReadReg(XN297L_CMD_R_RX_PL_WID);
        XN297L_ReadToBuf(XN297L_CMD_R_RX_PAYLOAD, xbuf, rxplWidth);
    }
    return status;
}

void XN297L_Carrier(uint8_t ucChannel_Set)
{
    uint8_t BB_cal_data[]   = {0x12, 0xED, 0x67, 0x9C, 0x46};
    uint8_t RF_cal_data[]   = {0xF6, 0x3F, 0x5D};
    uint8_t RF_cal2_data[]  = {0x45, 0x21, 0xEF, 0x2C, 0x5A, 0x42};
    uint8_t Dem_cal_data[]  = {0xE1};
    uint8_t Dem_cal2_data[] = {0x0B, 0xDF, 0x02};

    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0x5A); // Soft reset
    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0XA5);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_FEATURE, XN297L_FEATURE_BIT5_CE_SOFT);

    XN297L_CE_LOW();
    LL_mDelay(0);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG, 0X8E);         // tx mode
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CH, ucChannel_Set); // carrier frequency point
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_SETUP, XN297L_RF_POWER_P_9|XN297L_RF_DR_1M);
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_BB_CAL, BB_cal_data, sizeof(BB_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_RF_CAL2, RF_cal2_data, sizeof(RF_cal2_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_DEM_CAL, Dem_cal_data, sizeof(Dem_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_RF_CAL, RF_cal_data, sizeof(RF_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_DEM_CAL2, Dem_cal2_data, sizeof(Dem_cal2_data));
    LL_mDelay(0);
}

uint8_t XN297L_PrintStatus(void)
{
    uint8_t i, status;

    printf("\r\n[Config]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_CONFIG));
    printf("  [EN_AA]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_EN_AA));
    printf("  [EN_RxAddr]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_EN_RXADDR));
    printf("  [AddrWidth]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_SETUP_AW));
    printf("  [Retry]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_SETUP_RETR));

    printf("\r\n[RF_Channel]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RF_CH));
    printf("  [RF_Setup]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RF_SETUP));
    printf("  [Observe_Tx]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_OBSERVE_TX, xbuf, 4);
    for (i = 0; i < 4; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[TxAddr] ");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[RxAddrP0]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P0, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P1]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P1, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P2]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P2, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P3]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P3, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P4]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P4, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P5]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P5, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[RxPloadWidth_P0-P5]");
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P0));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P1));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P2));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P3));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P4));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P5));

    printf("\r\n[FIFO_Status]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_FIFO_STATUS, xbuf, 3);
    for (i = 0; i < 3; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf("  [DynPloadWidth]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_DYNPD));
    printf("  [Feature]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_FEATURE));

    status = XN297L_ReadStatus();
    printf("\r\n[Status]0x%02X\r\n\r\n", status);
    return status;
}
