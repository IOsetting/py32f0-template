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

#include "xl2400p.h"

static GPIO_TypeDef     *NssPort, *MosiPort, *MisoPort, *ClockPort, *InterruptPort;
static uint32_t         NssPin, MosiPin, MisoPin, ClockPin, InterruptPin;
static XL2400_SetPinHigh_fptr_t     XL2400_SetPinHigh;
static XL2400_SetPinLow_fptr_t      XL2400_SetPinLow;
static XL2400_GetPinState_fptr_t    XL2400_GetPinState;

uint8_t RF_Test_Adrress[5]={0xcc,0xcc,0xcc,0xcc,0xcc};

static void XL2400_ClearBuffer(void);

void XL2400_Config(XL2400_InitTypedef_t initStruct)
{
  NssPort           = initStruct.NssPort;
  NssPin            = initStruct.NssPin;
  MosiPort          = initStruct.MosiPort;
  MosiPin           = initStruct.MosiPin;
  MisoPort          = initStruct.MisoPort;
  MisoPin           = initStruct.MisoPin;
  ClockPort         = initStruct.ClockPort;
  ClockPin          = initStruct.ClockPin;
  InterruptPort     = initStruct.InterruptPort;
  InterruptPin      = initStruct.InterruptPin;
  XL2400_SetPinHigh = initStruct.SetPinHigh;
  XL2400_SetPinLow  = initStruct.SetPinLow;
  XL2400_GetPinState = initStruct.GetPinState;
}

/**
 * Emulate SPI Write on GPIO pins
 */
void XL2400_WriteByte(uint8_t value)
{
  uint8_t i = 0;
  XL2400_SetPinLow(ClockPort, ClockPin);
  for (i = 0; i < 8; i++)
  {
    XL2400_SetPinLow(ClockPort, ClockPin);
    if (value & 0x80)
    {
      XL2400_SetPinHigh(MosiPort, MosiPin);
    }
    else
    {
      XL2400_SetPinLow(MosiPort, MosiPin);
    }
    XL2400_SetPinHigh(ClockPort, ClockPin);
    value = value << 1;
  }
  XL2400_SetPinHigh(MosiPort, MosiPin);
  XL2400_SetPinLow(ClockPort, ClockPin);
}

/**
 * Emulate SPI Read on GPIO pins
 */
uint8_t XL2400_ReadByte(void)
{
  uint8_t i = 0, RxData = 0;

  for (i = 0; i < 8; i++)
  {
    RxData = RxData << 1;
    XL2400_SetPinHigh(ClockPort, ClockPin);
    __NOP();
    if (XL2400_GetPinState(MisoPort, MisoPin))
    {
      RxData |= 0x01;
    }
    else
    {
      RxData &= 0xfe;
    }
    XL2400_SetPinLow(ClockPort, ClockPin);
  }
  return RxData;
}

void XL2400_WriteReg(uint8_t reg, uint8_t value)
{
  XL2400_SetPinLow(NssPort, NssPin);
  XL2400_WriteByte(reg);
  XL2400_WriteByte(value);
  XL2400_SetPinHigh(NssPort, NssPin);
}

uint8_t XL2400_ReadReg(uint8_t reg)
{
  uint8_t reg_val;
  XL2400_SetPinLow(NssPort, NssPin);
  XL2400_WriteByte(reg);
  reg_val = XL2400_ReadByte();
  XL2400_SetPinHigh(NssPort, NssPin);
  return reg_val;
}

void XL2400_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
  uint8_t ctr;
  XL2400_SetPinLow(NssPort, NssPin);
  XL2400_WriteByte(reg);
  for (ctr = 0; ctr < len; ctr++)
  {
    XL2400_WriteByte(*pBuf++);
  }
  XL2400_SetPinHigh(NssPort, NssPin);
}

void XL2400_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t ctr;
  XL2400_SetPinLow(NssPort, NssPin);
  XL2400_WriteByte(reg);
  for (ctr = 0; ctr < len; ctr++)
  {
    pBuf[ctr] = XL2400_ReadByte();
  }
  XL2400_SetPinHigh(NssPort, NssPin);
}

void XL2400_CE_High(void)
{
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, 0xEF);
}

void XL2400_CE_Low(void)
{
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, 0xEE);
}

void XL2400_Init(void)
{
	uint8_t RF_Init_Buff[16]={0};

  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, 0x02);
  LL_mDelay(2);

  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, 0x3E);
  LL_mDelay(2);

  XL2400_ReadToBuf(XL2400_REG_ANALOG_CFG3, RF_Init_Buff, 6);
  RF_Init_Buff[5] = (RF_Init_Buff[5] | 0x6D);
  XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER + XL2400_REG_ANALOG_CFG3, RF_Init_Buff, 6);

  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_EN_AA, 0x3F);         // 0x00:no ack, 0x3F:all ack
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_EN_RXADDR, 0x3F);
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_SETUP_AW, 0xaf);      // Address width
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_SETUP_RETR, 0x33);    // retries and interval
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_RF_SETUP, XL2400_RF_DR_250K); // speed rate

  // RF_Init_Buff[0] = RF_PACKET_SIZE;
  // RF_Init_Buff[1] = RF_PACKET_SIZE;
  // XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER+RX_PW_PX, RF_Init_Buff ,2);

  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_DYNPD, 0x3f); // Use dynamic length

  //bit7&6=00 Return status when send register address
  //bit5=0 Disable long-pack
  //bit4=1 FEC off
  //bit3=1 FEATURE on
  //bit2=0 Dynamic length off
  //bit1=0 ACK without payload
  //bit0=0 Disable W_TX_PAYLOAD_NOACK mode
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_FEATURE, 0x1C);

  XL2400_SetPower(XL2400_RF_0DB);

	XL2400_SetAddress(RF_Test_Adrress);
}

static void XL2400_ClearBuffer(void)
{
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_STATUS, 0x70);
  XL2400_WriteReg(XL2400_CMD_FLUSH_TX, XL2400_CMD_NOP);
  XL2400_WriteReg(XL2400_CMD_FLUSH_RX, XL2400_CMD_NOP);
}

void XL2400_Reset(void)
{
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, 0xEA);
  LL_mDelay(0);
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, 0xEE);
  LL_mDelay(1);
}

void XL2400_SetChannel(uint8_t ch)
{
  if (ch > 80) ch = 80;
  uint8_t bak = XL2400_ReadReg(XL2400_CMD_W_REGISTER + XL2400_REG_EN_AA);

  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_EN_AA, bak & ~0x40);
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_RF_CH, 0x60 + ch);
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_EN_AA, bak | 0x40);
}

void XL2400_SetAddress(uint8_t *addr)
{
  XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER + XL2400_REG_TX_ADDR, addr, 5);
  XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER + XL2400_REG_RX_ADDR_P0, addr, 5);
}

void XL2400_SetPower(uint8_t power)
{
  uint8_t buff[2];
  XL2400_ReadToBuf(XL2400_CMD_R_REGISTER + XL2400_REG_RF_SETUP, buff, 2);
  buff[1] = power;
  XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER + XL2400_REG_RF_SETUP, buff, 2);
}

void XL2400_Sleep(void)
{
  XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, 0x00);
}

void XL2400_SetTxMode(void)
{
  uint8_t buff[2];
  buff[0] = 0xee;
  buff[1] = 0x80;
  XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, buff, 2);
  XL2400_ClearBuffer();
  LL_mDelay(1);
}

void XL2400_SetRxMode(void)
{
  uint8_t buff[2];
  buff[0] = 0xee;
  buff[1] = 0xc0;
  XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER + XL2400_REG_CFG_TOP, buff, 2);
  XL2400_ClearBuffer();
  XL2400_CE_High();
  LL_mDelay(1);
}

uint8_t XL2400_Rx(uint8_t *pBuff)
{
  uint8_t status, rxplWidth;

  status = XL2400_ReadReg(XL2400_REG_STATUS);
  if (status & XL2400_FLAG_RX_DR)
  {
    rxplWidth = XL2400_ReadReg(XL2400_CMD_R_RX_PL_WID);
    XL2400_ReadToBuf(XL2400_CMD_R_RX_PAYLOAD, pBuff, rxplWidth);
    XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_STATUS, status);
  }
  return status;
}

uint8_t XL2400_Tx(uint8_t *pBuff , uint8_t len)
{
  uint8_t status, timeout = 200;

  XL2400_CE_High();
  XL2400_WriteReg(XL2400_CMD_FLUSH_TX, XL2400_CMD_NOP);
  XL2400_WriteFromBuf(XL2400_CMD_W_TX_PAYLOAD, pBuff, len);
  while (timeout--)
  {
    LL_mDelay(0);
    status = XL2400_ReadReg(XL2400_CMD_R_REGISTER + XL2400_REG_STATUS);
    if ((status & (XL2400_FLAG_TX_DS | XL2400_FLAG_MAX_RT)) != 0)
    {
      printf("%d: %02X\r\n", timeout, status);
      XL2400_WriteReg(XL2400_CMD_W_REGISTER + XL2400_REG_STATUS, status);
      break;
    }
  }
  return status;
}