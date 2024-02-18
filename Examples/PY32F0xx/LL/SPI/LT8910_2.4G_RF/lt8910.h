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

#define LT8910_PKT_READ()           LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_5)

#define bit(b) (1UL << (b))

#define REGISTER_READ       0b10000000  //bin
#define REGISTER_WRITE      0b00000000  //bin
#define REGISTER_MASK       0b01111111  //bin

#define R_CHANNEL           7
#define CHANNEL_RX_BIT      7
#define CHANNEL_TX_BIT      8
#define CHANNEL_MASK        0x7F  ///bin
#define DEFAULT_CHANNEL     0x30

#define R_CURRENT           9
#define CURRENT_POWER_SHIFT 12
#define CURRENT_POWER_MASK  0b1111000000000000
#define CURRENT_GAIN_SHIFT  7
#define CURRENT_GAIN_MASK   0b0000011110000000

#define LT89xx_6dBm             0x4800
#define LT89xx_2dBm             0X1840
#define LT89xx_1_7dBm           0X18C0
#define LT89xx_1dBm             0X1940
#define LT89xx_n0_3dBm          0X19C0
#define LT89xx_n1_4dBm          0X1A40    
#define LT89xx_n2_2dBm          0X1AC0
#define LT89xx_n3dBm            0X1B40
#define LT89xx_n4dBm            0X1BC0    
#define LT89xx_n6_5dBm          0X1C40
#define LT89xx_n7_3dBm          0X1CC0    
#define LT89xx_n8_2dBm          0X1D40
#define LT89xx_n9_5dBm          0X1DC0    
#define LT89xx_n10_7dBm         0X1E40       
#define LT89xx_n12_2dBm         0X1EC0    
#define LT89xx_n14_2dBm         0X1F40
#define LT89xx_n17dBm           0X1FC0    

/* LT8910S only */
#define R_DATARATE          44
#define DATARATE_MASK       0xFF00
#define DATARATE_1MBPS      0x0100
#define DATARATE_250KBPS    0x0400
#define DATARATE_125KBPS    0x0800
#define DATARATE_62KBPS     0x1000

#define R_SYNCWORD1         36
#define R_SYNCWORD2         37
#define R_SYNCWORD3         38
#define R_SYNCWORD4         39

#define R_PACKETCONFIG      41
#define PACKETCONFIG_CRC_ON             0x8000
#define PACKETCONFIG_SCRAMBLE_ON        0x4000
#define PACKETCONFIG_PACK_LEN_ENABLE    0x2000
#define PACKETCONFIG_FW_TERM_TX         0x1000
#define PACKETCONFIG_AUTO_ACK           0x0800
#define PACKETCONFIG_PKT_FIFO_POLARITY  0x0400

#define R_STATUS            48
#define STATUS_CRC_BIT      15


#define R_FIFO              50
#define R_FIFO_CONTROL      52

typedef enum
{
    LT8910_1MBPS,   /** default transmit rate */
    LT8910_250KBPS, /** 250 Kpbs, only on lt8910 */
    LT8910_125KBPS, /** 125 Kbps, only on lt8910 */
    LT8910_62KBPS   /** 62 Kbps, only on lt8910 */
} DataRate;


uint16_t LT8910_ReadRegister(uint8_t reg);
uint16_t LT8910_WriteRegister(uint8_t reg, uint8_t high, uint8_t low);
uint16_t LT8910_WriteRegister16(uint8_t reg, uint16_t value);
uint8_t LT8910_ReadToBuf(uint8_t reg, uint8_t *pBuf);

void LT8910_Init(void);
void LT8910_SetChannel(uint8_t channel);
void LT8910_SetCurrentControl(uint8_t power, uint8_t gain);
void LT8910_SetSyncWord(uint64_t syncWord);
ErrorStatus LT8910_SetDataRate(DataRate rate);
void LT8910_WhatsUp(void);
uint8_t LT8910_IsAvailable(void);
void LT8910_SetRx(void);
uint8_t LT8910_Rx(uint8_t *pBuf);
void LT8910_Tx(uint8_t *data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* __LT8910_H */
