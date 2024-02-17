#ifndef __LT8960Ldrv_H
#define __LT8960Ldrv_H

#include "main.h"

// IIC_CLK  >>>  PA5
// IIC_DAT  >>>  PA6

///////////////////////////////////////////////////////////////////////////////
#define LT8960L_CLK_H           GPIOA->BSRR = LL_GPIO_PIN_5
#define LT8960L_CLK_L           GPIOA->BRR = LL_GPIO_PIN_5

#define LT8960L_DAT_H           GPIOA->BSRR = LL_GPIO_PIN_6
#define LT8960L_DAT_L           GPIOA->BRR = LL_GPIO_PIN_6
#define LT8960L_DAT_read        (GPIOA->IDR  & LL_GPIO_PIN_6)

#define LT8960L_DAT_Input()     LT8960L_DAT_InputMode()
#define LT8960L_DAT_Output()    LT8960L_DAT_OutputMode()

#define LED_ON     GPIOA->BSRR= LL_GPIO_PIN_7// PA7
#define LED_OFF    GPIOA->BRR = LL_GPIO_PIN_7// PA7
#define LED_CPL    (GPIOA->ODR&LL_GPIO_PIN_7)?(GPIOA->BRR=LL_GPIO_PIN_7):(GPIOA->BSRR = LL_GPIO_PIN_7)


#define LT8960L_RF_Power        LT8960L_7dBm

/* Frequency = 2402 + LT8960L_RF_CHANNEL MHz */
#define LT8960L_RF_CHANNEL      76

#define LT8960L_PACKET_SIZE     12   // Byte

#define LT8960L_TX_INTERVAL_MS  10   // mS

/* Data rate */

//#define Air_rate_1M
//#define Air_rate_250K
//#define Air_rate_125K
#define Air_rate_62K5

#define LT8960L_SYNC_WORD       0x03,0x80,0x5A,0x5A        

//*************************************
//变量与函数声明部分 不建议修改   

#define LT8960L_7dBm            0x7830
#define LT8960L_6dBm            0X7930
#define LT8960L_5dBm            0X7A30
#define LT8960L_3_4dBm          0X7B30
#define LT8960L_0_2dBm          0X7C30
#define LT8960L_n1_5dBm         0X7D30    
#define LT8960L_n4dBm           0X7E30
#define LT8960L_n7dBm           0X7F30
#define LT8960L_n9dBm           0X3F30    
#define LT8960L_n13dBm          0X3FB0
#define LT8960L_n19dBm          0X3FC0    


void  LT8960L_ack(void);
void  LT8960L_start(void);
void  LT8960L_Send_Byte(unsigned char reg);  
void  LT8960L_stop(void);
void  LT8960L_WriteReg(unsigned char reg, unsigned char H, unsigned char L);
void  LT8960L_ReadReg(unsigned char reg);
void  LT8960L_Carrier_Wave(unsigned char FreqChannel);
void  LT8960L_Sleep(void);
void  LT8960L_Wakeup(void);
unsigned char LT8960L_GetPKT(void);
void  LT8960L_INIT(void);

void LT8960L_OpenRx(unsigned char FreqChannel);

void  LT8960L_TxData(unsigned char FreqChannel,unsigned char *pBuf,unsigned char length);
unsigned char LT8960L_RxData(unsigned char FreqChannel,unsigned char *pBuf,unsigned char length);

#endif
