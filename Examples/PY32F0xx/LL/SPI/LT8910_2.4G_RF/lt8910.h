#ifndef __LT8910_H
#define __LT8910_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LT8910_DelayMs(__VAL__)     LL_mDelay(__VAL__)
// PA4
#define LT8910_RST_LOW()            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define LT8910_RST_HIGH()           LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4)
// PA6
#define LT8910_SS_LOW()             LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)
#define LT8910_SS_HIGH()            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)
// PA5
#define LT8910_PKT_READ()           LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5)


// Reg 7 bit[8]
#define LT8910_TX_EN                (0x01 << 8)
// Reg 7 bit[7]
#define LT8910_RX_EN                (0x01 << 7)

// Reg 9 bit[15,12] current control
// Reg 9 bit[10, 7] gain control
#define LT8910_6DBM                 0x4800 // 6dBm
#define LT8910_2DBM                 0X1840 // 2dBm
#define LT8910_1_7DBM               0X18C0 // 1.7dBm
#define LT8910_1DBM                 0X1940 // 1dBm
#define LT8910__0_3DBM              0X19C0 // -0.3dBm
#define LT8910__1_4DBM              0X1A40 // -1.4dBm
#define LT8910__2_2DBM              0X1AC0
#define LT8910__3DBM                0X1B40
#define LT8910__4DBM                0X1BC0    
#define LT8910__6_5DBM              0X1C40 
#define LT8910__7_3DBM              0X1CC0    
#define LT8910__8_2DBM              0X1D40
#define LT8910__9_5DBM              0X1DC0    
#define LT8910__10_7DBM             0X1E40       
#define LT8910__12_2DBM             0X1EC0    
#define LT8910__14_2DBM             0X1F40
#define LT8910__17DBM               0X1FC0

// Reg 32, bit[15,13], default 0x02
#define LT8910_PREAMBLE_1BYTE       (0x00 << 13)
#define LT8910_PREAMBLE_2BYTE       (0x01 << 13)
#define LT8910_PREAMBLE_3BYTE       (0x02 << 13)
#define LT8910_PREAMBLE_4BYTE       (0x03 << 13)
#define LT8910_PREAMBLE_5BYTE       (0x04 << 13)
#define LT8910_PREAMBLE_6BYTE       (0x05 << 13)
#define LT8910_PREAMBLE_7BYTE       (0x06 << 13)
#define LT8910_PREAMBLE_8BYTE       (0x07 << 13)
// Reg 32, bit[12,11], default 0x03
#define LT8910_SYNCWORD_16BIT       (0x00 << 11) // Reg36
#define LT8910_SYNCWORD_32BIT       (0x01 << 11) // Reg39,Reg36
#define LT8910_SYNCWORD_48BIT       (0x02 << 11) // Reg39,Reg38,Reg36
#define LT8910_SYNCWORD_64BIT       (0x03 << 11) // Reg39,Reg38,Reg37,Reg36
// Reg 32, bit[10, 8], default 0x00
#define LT8910_TRAILER_4BIT         (0x00 << 8)
#define LT8910_TRAILER_6BIT         (0x01 << 8)
#define LT8910_TRAILER_8BIT         (0x02 << 8)
#define LT8910_TRAILER_10BIT        (0x03 << 8)
#define LT8910_TRAILER_12BIT        (0x04 << 8)
#define LT8910_TRAILER_14BIT        (0x05 << 8)
#define LT8910_TRAILER_16BIT        (0x06 << 8)
#define LT8910_TRAILER_18BIT        (0x07 << 8)
// Reg 32, bit[ 7, 6], default 0x00
#define LT8910_PACKET_NRZ_RAW       (0x00 << 6)
#define LT8910_PACKET_MANCHESTER    (0x01 << 6)
#define LT8910_PACKET_8B10B_LINE    (0x02 << 6)
#define LT8910_PACKET_INTERLEAVE    (0x03 << 6)
// Reg 32, bit[ 3, 1], default 0x03
#define LT8910_BRCLK_LOW0           (0x00 << 1)
#define LT8910_BRCLK_DIV1           (0x01 << 1)
#define LT8910_BRCLK_DIV2           (0x02 << 1)
#define LT8910_BRCLK_DIV4           (0x03 << 1)
#define LT8910_BRCLK_DIV8           (0x04 << 1)
#define LT8910_BRCLK_1MHZ           (0x05 << 1)
#define LT8910_BRCLK_APLL_CLK       (0x06 << 1)
#define LT8910_BRCLK_LOW1           (0x07 << 1)

// Reg 44, bit[15, 8], default 0x00
#define LT8910_DATARATE_1MBPS       (0x01 << 8)
#define LT8910_DATARATE_250KBPS     (0x04 << 8)
#define LT8910_DATARATE_125KBPS     (0x08 << 8)
#define LT8910_DATARATE_62KBPS      (0x10 << 8)

// Reg 45, bit[15, 0]
#define LT8910_DATARATE_1MBPS_OPT   (0x0152)
#define LT8910_DATARATE_250KBPS_OPT (0x0552)
#define LT8910_DATARATE_125KBPS_OPT (0x0552)
#define LT8910_DATARATE_62KBPS_OPT  (0x0552)


uint16_t LT8910_ReadRegister(uint8_t reg);
uint16_t LT8910_WriteRegister(uint8_t reg, uint8_t high, uint8_t low);
uint16_t LT8910_WriteRegister16(uint8_t reg, uint16_t value);
uint8_t LT8910_ReadToBuf(uint8_t reg, uint8_t *pBuf);

void LT8910_Init(void);
void LT8910_SetChannel(uint8_t channel);
void LT8910_SetCurrentControl(uint8_t power, uint8_t gain);
void LT8910_SetSyncWord(uint64_t syncWord);
void LT8910_SetDataRate(uint16_t regval);
void LT8910_WhatsUp(void);
uint8_t LT8910_IsAvailable(void);
void LT8910_SetRx(void);
uint8_t LT8910_Rx(uint8_t *pBuf);
void LT8910_Tx(uint8_t *data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* __LT8910_H */
