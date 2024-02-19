#include "lt8910.h"
#include <stdio.h>

#define bit(b) (1UL << (b))

#define REGISTER_READ       0b10000000  //bin
#define REGISTER_WRITE      0b00000000  //bin
#define REGISTER_MASK       0b01111111  //bin

#define R_CHANNEL           7

#define R_CURRENT           9
#define CURRENT_POWER_SHIFT 12
#define CURRENT_POWER_MASK  0b1111000000000000
#define CURRENT_GAIN_SHIFT  7
#define CURRENT_GAIN_MASK   0b0000011110000000

#define R_SYNCWORD1         36
#define R_SYNCWORD2         37
#define R_SYNCWORD3         38
#define R_SYNCWORD4         39

#define R_PACKETCONFIG      41
#define PACKETCONFIG_CRC_ON             0x8000
#define PACKETCONFIG_SCRAMBLE_ON        0x4000
#define PACKETCONFIG_PACK_LEN_ENABLE    0x2000 // bit[13], 0:off, 1:on, first byte indicate the packet length
#define PACKETCONFIG_FW_TERM_TX         0x1000
#define PACKETCONFIG_AUTO_ACK           0x0800
#define PACKETCONFIG_PKT_FIFO_POLARITY  0x0400

#define R_DATARATE          44
#define R_STATUS            48
#define STATUS_CRC_BIT      15

#define R_FIFO              50
#define R_FIFO_CONTROL      52


uint8_t _lt8910_channel;

uint16_t LT8910_ReadRegister(uint8_t reg)
{
  uint8_t lsb, msb;
  LT8910_SS_LOW();
  SPI_TxRxByte(reg | REGISTER_READ);
  msb = SPI_TxRxByte(0xFF);
  lsb = SPI_TxRxByte(0xFF);
  LT8910_SS_HIGH();
  return (msb << 8 | lsb);
}

uint16_t LT8910_WriteRegister(uint8_t reg, uint8_t high, uint8_t low)
{
  uint8_t lsb, msb;
  LT8910_SS_LOW();
  SPI_TxRxByte(reg & REGISTER_MASK);
  msb = SPI_TxRxByte(high);
  lsb = SPI_TxRxByte(low);
  LT8910_SS_HIGH();
  return (msb << 8 | lsb);
}

uint16_t LT8910_WriteRegister16(uint8_t reg, uint16_t value)
{
  uint8_t lsb, msb;
  LT8910_SS_LOW();
  SPI_TxRxByte(reg & REGISTER_MASK);
  msb = SPI_TxRxByte(value >> 8);
  lsb = SPI_TxRxByte(value & 0xFF);
  LT8910_SS_HIGH();
  return (msb << 8 | lsb);
}

uint8_t LT8910_ReadToBuf(uint8_t reg, uint8_t *pBuf)
{
  uint8_t size, i;
  
  LT8910_SS_LOW();

  SPI_TxRxByte(reg | REGISTER_READ);
  size = SPI_TxRxByte(0xFF);
  i = size;
  while (i--)
  {
    *pBuf++ = SPI_TxRxByte(0xFF);
  }
  LT8910_SS_HIGH();
  return size;
}

void LT8910_Init(void)
{
  LT8910_RST_LOW();
  LT8910_DelayMs(200);
  LT8910_RST_HIGH();
  LT8910_DelayMs(200);

  LT8910_WriteRegister16(0, 0x6FE0);
  LT8910_WriteRegister16(1, 0x5681);
  LT8910_WriteRegister16(2, 0x6617);
  LT8910_WriteRegister16(4, 0x9CC9);
  LT8910_WriteRegister16(5, 0x6637);
  LT8910_WriteRegister16(7, 0x0030);
  LT8910_WriteRegister16(8, 0x6C90); // Undocumented
  LT8910_WriteRegister16(9, LT8910__1_4DBM);
  LT8910_WriteRegister16(10, 0x7FFD);
  LT8910_WriteRegister16(11, 0x0008); // RSSI on
  LT8910_WriteRegister16(12, 0x0000);
  LT8910_WriteRegister16(13, 0x48BD);

  LT8910_WriteRegister16(22, 0x00FF);
  LT8910_WriteRegister16(23, 0x8005);
  LT8910_WriteRegister16(24, 0x0067);
  LT8910_WriteRegister16(25, 0x1659);
  LT8910_WriteRegister16(26, 0x19E0);

  LT8910_WriteRegister16(27, 0x1301); // 1Mbps, Undocumented
  LT8910_WriteRegister16(28, 0x1800);

  LT8910_WriteRegister16(32, LT8910_PREAMBLE_3BYTE 
                           | LT8910_SYNCWORD_32BIT 
                           | LT8910_TRAILER_4BIT 
                           | LT8910_PACKET_NRZ_RAW 
                           | LT8910_BRCLK_LOW0);

  LT8910_WriteRegister16(33, 0x3FC7);
  LT8910_WriteRegister16(34, 0x2000);
  LT8910_WriteRegister16(35, 0x0300);

  LT8910_WriteRegister16(36, 0x0380);
  LT8910_WriteRegister16(37, 0x068C);
  LT8910_WriteRegister16(38, 0x5A5A);
  LT8910_WriteRegister16(39, 0x5A5A);
  LT8910_WriteRegister16(40, 0x4402);
  LT8910_WriteRegister16(41, 0xB000);
  LT8910_WriteRegister16(42, 0xFDB0);
  LT8910_WriteRegister16(43, 0x000F);

  LT8910_WriteRegister16(44, LT8910_DATARATE_1MBPS);
  LT8910_WriteRegister16(45, LT8910_DATARATE_1MBPS_OPT);
  LT8910_WriteRegister16(R_FIFO_CONTROL, 0x8080); //Fifo Rx/Tx queue reset

  LT8910_SetSyncWord(0x03805a5a03800380);

  LT8910_DelayMs(200);
}

void LT8910_Sleep()
{
  /* set bit 14 on register 35.
   * It will wake up when SPI_SS is low
   */
  LT8910_WriteRegister16(35, LT8910_ReadRegister(35) | bit(14));
}

void LT8910_SetChannel(uint8_t channel)
{
  _lt8910_channel = channel & 0x7F;
}

void LT8910_SetCurrentControl(uint8_t power, uint8_t gain)
{
  LT8910_WriteRegister16(R_CURRENT,
                ((power << CURRENT_POWER_SHIFT) & CURRENT_POWER_MASK) |
                ((gain << CURRENT_GAIN_SHIFT) & CURRENT_GAIN_MASK));
}

void LT8910_SetSyncWord(uint64_t syncWord)
{
  LT8910_WriteRegister16(R_SYNCWORD1, syncWord);
  LT8910_WriteRegister16(R_SYNCWORD2, syncWord >> 16);
  LT8910_WriteRegister16(R_SYNCWORD3, syncWord >> 32);
  LT8910_WriteRegister16(R_SYNCWORD4, syncWord >> 48);
}

void LT8910_SetDataRate(uint16_t regval)
{
  LT8910_WriteRegister16(R_DATARATE, regval);
}

/** Set BRCLK_SEL, register 32, bits 3:1 */
void LT8910_SetBRClk(uint8_t clock)
{
  uint16_t val = LT8910_ReadRegister(32);
  val &= 0xFFF1;
  val |= (clock & 0x07) << 1;
  LT8910_WriteRegister16(32, val);
}

uint8_t LT8910_IsAvailable(void)
{
  //read the PKT_FLAG state; this can also be done with a hard wire.
  if (LT8910_PKT_READ() != 0)
  {
    return 1;
  }

  return 0;
}

void LT8910_Tx(uint8_t *data, uint8_t size)
{
  uint8_t msb, lsb, pos = 0;

  LT8910_WriteRegister16(R_CHANNEL, _lt8910_channel);
  LT8910_WriteRegister16(52, 0x8080);
  LT8910_WriteRegister16(8, 0x6C90);

  // packets are sent in 16bit, the first 8bit will be the packet size.
  //printf("TX size:%d, %02X ", size, data[pos]);
  LT8910_WriteRegister(R_FIFO, size, data[pos++]);
  while (pos < size)
  {
    //printf("%02X ", data[pos]);
    msb = data[pos++];
    //printf("%02X ", data[pos]);
    lsb = data[pos++];
    LT8910_WriteRegister(R_FIFO, msb, lsb);
  }
  //printf("\r\n");
  LT8910_WriteRegister16(R_CHANNEL, LT8910_TX_EN | _lt8910_channel);
  LT8910_DelayMs(0);
  //Wait until the packet is sent.
  while (LT8910_PKT_READ() == 0)
  {
    //printf(".");
    LT8910_DelayMs(0);
  }
}

void LT8910_SetRx(void)
{
  LT8910_WriteRegister16(R_CHANNEL, _lt8910_channel);
  LT8910_WriteRegister16(52, 0x8080);
  LT8910_WriteRegister16(8, 0x6C90);
  LT8910_WriteRegister16(R_CHANNEL, LT8910_RX_EN | _lt8910_channel);
}

uint8_t LT8910_Rx(uint8_t *pBuf)
{
  uint8_t len = 0;

  if (LT8910_PKT_READ() != 0)
  {
    //printf(":");
    if ((LT8910_ReadRegister(48) & 0x8000) == 0)
    {
      len = LT8910_ReadToBuf(R_FIFO, pBuf);
      //printf("~%d ", len);
    }
    // Enable RX
    LT8910_WriteRegister16(R_CHANNEL, LT8910_RX_EN | _lt8910_channel);
  }
  return len;
}

void LT8910_WhatsUp(void)
{
  uint16_t val = LT8910_ReadRegister(R_CHANNEL);
  printf("%04X ", val);
  printf("RF Channel:%d ", val & 0x7F);
  printf("TX:%d ", (uint8_t)((val & LT8910_TX_EN) >> 8));
  printf("RX:%d\r\n", (uint8_t)((val & LT8910_RX_EN) >> 7));

  val = LT8910_ReadRegister(R_DATARATE);
  printf("Data_Rate:0x%04X ", val);
  val = LT8910_ReadRegister(29);
  printf("REG29:0x%04X ", val);
  val = LT8910_ReadRegister(32);
  printf("REG32:0x%04X ", val);
  val = LT8910_ReadRegister(41);
  printf("REG41:0x%04X ", val);

  val = LT8910_ReadRegister(48);
  printf("REG48:0x%04X\r\n", val);

  uint8_t crc_error = (val & bit(15)) != 0;
  uint8_t fec23_error = (val & bit(14)) != 0;
  uint8_t framer_st = (val & 0b0011111100000000) >> 8;
  uint8_t pkt_flag = (val & bit(6)) != 0;
  uint8_t fifo_flag = (val & bit(5)) != 0;

  printf("CRC=%d ", crc_error);
  printf("FEC=%d ", fec23_error);
  printf("FRAMER_ST=0X%02X ", framer_st);
  printf("PKT=%d ", pkt_flag);
  printf("FIFO=%d\r\n", fifo_flag);

  uint16_t fifo = LT8910_ReadRegister(R_FIFO_CONTROL);
  printf("FIFO_WR_PTR=0X%02X ", (fifo >> 8) & 0b111111);
  printf("FIFO_RD_PTR=0X%02X\r\n", fifo & 0b111111);
}
