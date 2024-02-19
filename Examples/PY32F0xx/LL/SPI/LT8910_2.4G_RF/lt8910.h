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

#define LT8910_6dBm                 0x4800
#define LT8910_2dBm                 0X1840
#define LT8910_1_7dBm               0X18C0
#define LT8910_1dBm                 0X1940
#define LT8910_n0_3dBm              0X19C0
#define LT8910_n1_4dBm              0X1A40    
#define LT8910_n2_2dBm              0X1AC0
#define LT8910_n3dBm                0X1B40
#define LT8910_n4dBm                0X1BC0    
#define LT8910_n6_5dBm              0X1C40
#define LT8910_n7_3dBm              0X1CC0    
#define LT8910_n8_2dBm              0X1D40
#define LT8910_n9_5dBm              0X1DC0    
#define LT8910_n10_7dBm             0X1E40       
#define LT8910_n12_2dBm             0X1EC0    
#define LT8910_n14_2dBm             0X1F40
#define LT8910_n17dBm               0X1FC0

#define LT8910_DATARATE_MASK        0xFF00
#define LT8910_DATARATE_1MBPS       0x0100
#define LT8910_DATARATE_250KBPS     0x0400
#define LT8910_DATARATE_125KBPS     0x0800
#define LT8910_DATARATE_62KBPS      0x1000

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
