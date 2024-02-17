#include "LT8960Ldrv.h"

#include <stdio.h>
#include <stdint.h>

unsigned char LT8960L_RegH,LT8960L_RegL;

unsigned char SyncWord_Value[4]={LT8960L_SYNC_WORD};


#define _delay()     __NOP()
#define _delayRD()   __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP()

unsigned char DAT_IO_Type;    //0=output 1=input

void LT8960L_DAT_InputMode(void)
{
    //PA6
    GPIOA->MODER  &= (uint32_t)~(0x03 << (6*2));    //INPUT        
    DAT_IO_Type=1;
}
void LT8960L_DAT_OutputMode(void)
{
    //PA6
    GPIOA->MODER  |= (uint32_t)( 0x01 << (6*2));        //OUTPUT    
    DAT_IO_Type = 0;
}

void LT8960L_start(void)
{
  LT8960L_DAT_Output();    
  LT8960L_DAT_H;
  LT8960L_CLK_H;
  _delay();
  
  LT8960L_DAT_L;
  _delay();
  
  LT8960L_CLK_L;
  //_delay();  
}

void LT8960L_stop(void)
{
  LT8960L_CLK_L;
  //_delay();
  LT8960L_DAT_L;
  _delay();
  LT8960L_CLK_H;
  _delay();
  LT8960L_DAT_H;
}

void LT8960L_Send_Byte(unsigned char iDATA)
{
#ifdef Slow_Clock
    int i = 8;
    while (i--)
    {
        LT8960L_CLK_L;
        _delay();
        if (iDATA & 0x80)
            LT8960L_DAT_H;
        else
            LT8960L_DAT_L;
        iDATA <<= 1;
        _delay();
        LT8960L_CLK_H;
        _delay();
    }
    LT8960L_CLK_L;  
#else    
    LT8960L_CLK_L;
    //_delay();
    if (iDATA & 0x80)
            LT8960L_DAT_H;
    else
            LT8960L_DAT_L;
    // iDATA <<= 1;
    //_delay();
    LT8960L_CLK_H;
    _delay();

    LT8960L_CLK_L;
    //_delay();
    if (iDATA & 0x40)
            LT8960L_DAT_H;
    else
            LT8960L_DAT_L;
    // iDATA <<= 1;
    //_delay();
    LT8960L_CLK_H;
    _delay();        

    LT8960L_CLK_L;
    //_delay();
    if (iDATA & 0x20)
            LT8960L_DAT_H;
    else
            LT8960L_DAT_L;
    // iDATA <<= 1;
    //_delay();
    LT8960L_CLK_H;
    _delay();        

    LT8960L_CLK_L;
    //_delay();
    if (iDATA & 0x10)
            LT8960L_DAT_H;
    else
            LT8960L_DAT_L;
    // iDATA <<= 1;
    //_delay();
    LT8960L_CLK_H;
    _delay(); 


    LT8960L_CLK_L;
    //_delay();
    if (iDATA & 0x8)
            LT8960L_DAT_H;
    else
            LT8960L_DAT_L;
    // iDATA <<= 1;
    //_delay();
    LT8960L_CLK_H;
    _delay();

    LT8960L_CLK_L;
    //_delay();
    if (iDATA & 0x4)
            LT8960L_DAT_H;
    else
            LT8960L_DAT_L;
    // iDATA <<= 1;
    //_delay();
    LT8960L_CLK_H;
    _delay();        

    LT8960L_CLK_L;
    //_delay();
    if (iDATA & 0x2)
            LT8960L_DAT_H;
    else
            LT8960L_DAT_L;
    // iDATA <<= 1;
    //_delay();
    LT8960L_CLK_H;
    _delay();        

    LT8960L_CLK_L;
    //_delay();
    if (iDATA & 0x1)
            LT8960L_DAT_H;
    else
            LT8960L_DAT_L;
    // iDATA <<= 1;
    //_delay();
    LT8960L_CLK_H;
    _delay();   

     LT8960L_CLK_L;
#endif    
}
unsigned char LT8960L_Read_Byte(void)
{
  unsigned char byte = 0;
  
  LT8960L_DAT_H;
  LT8960L_DAT_Input();

#ifdef Slow_Clock
  unsigned char i = 8;    
  while (i--)
  {
    byte <<= 1;
    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= 0x01;
    }
    else
    {
      byte &= ~0x01;
    }      
      
    LT8960L_CLK_L;

   
  }
  LT8960L_CLK_L;
  
#else

    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= (1<<7);
    }
    
    LT8960L_CLK_L;  
    _delayRD();
    
    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= (1<<6);
    }
     
    LT8960L_CLK_L;      
    _delayRD();
    
    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= (1<<5);
    }
     
    LT8960L_CLK_L;      
    _delayRD();
    
    
    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= (1<<4);
    }
    LT8960L_CLK_L;      
    _delayRD();
    
    
    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= (1<<3);
    }
    LT8960L_CLK_L;      
    _delayRD();

    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= (1<<2);
    }

    LT8960L_CLK_L; 
    _delayRD();
    
    
    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= (1<<1);
    }

    LT8960L_CLK_L;      
    _delayRD();
    
    
    LT8960L_CLK_L;
    //_delay();//_delay();
    LT8960L_CLK_H;
    //_delay();//_delay();
      
    if (LT8960L_DAT_read)
    {
      byte |= (1<<0);
    }

    LT8960L_CLK_L;
    _delayRD();
    
#endif
  LT8960L_DAT_Output();
  return byte;  
}

void LT8960L_ack(void)
{
  LT8960L_DAT_L;      //mcu to chip   ACK
  //_delay();
    //_delay();
  LT8960L_CLK_H;      //9th clock
  _delay(); 
    //_delay();
  LT8960L_CLK_L; 
    //_delay();
}
void LT8960L_ack_1uS(void)
{
  LT8960L_DAT_L;      //mcu to chip   ACK
  //_delay();
    //_delay();
  LT8960L_CLK_H;      //9th clock
    
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();    
    
  LT8960L_CLK_L; 
}
void LT8960L_nack(void)
{
  LT8960L_DAT_H;      //mcu to chip   ACK
  LT8960L_CLK_H;      //9th clock
  _delay();   
  LT8960L_CLK_L; 
}

void LT8960L_WriteReg(unsigned char reg, unsigned char H, unsigned char L)
{
  LT8960L_start();
  LT8960L_Send_Byte(reg);   LT8960L_ack();
  LT8960L_Send_Byte(H);     LT8960L_ack();
  LT8960L_Send_Byte(L);     LT8960L_ack();
  LT8960L_stop();
}

void LT8960L_ReadReg(unsigned char reg)
{
  LT8960L_start();
  LT8960L_Send_Byte(reg|0X80);   LT8960L_ack();
  LT8960L_RegH=LT8960L_Read_Byte();LT8960L_ack();
  LT8960L_RegL=LT8960L_Read_Byte();LT8960L_nack();
  LT8960L_stop();
}
void LT8960L_WriteBUF(unsigned char reg, unsigned char *pBuf, unsigned char len)
{
    unsigned char i;
    LT8960L_start();
    LT8960L_Send_Byte(reg);   LT8960L_ack();
    LT8960L_Send_Byte(len);     LT8960L_ack();
    for(i=0; i<len; i++)
    {
        LT8960L_Send_Byte(pBuf[i]);
        LT8960L_ack();
    }
    
    LT8960L_stop();
}
unsigned char LT8960L_ReadBUF(unsigned char reg, unsigned char *pBuf)
{
    
    unsigned char len,FirstValue;
    
        FirstValue=0;
        
        LT8960L_start();
        LT8960L_Send_Byte(reg|0x80);    LT8960L_ack();      //Register address  
            
        len = LT8960L_Read_Byte(); //LT8960L_ack();

        if (len != LT8960L_PACKET_SIZE)
        {
            LT8960L_nack();
            LT8960L_stop();
            return FirstValue;
        }
        FirstValue = len;
    for(unsigned char i=0; i<len; i++)
    {
        LT8960L_ack_1uS();
        pBuf[i]=LT8960L_Read_Byte(); 
    } 
    LT8960L_nack();
    LT8960L_stop();

    return FirstValue;
}
void LT8960L_INIT(void)
{

reTry:
    LT8960L_WriteReg(0x38, 0xBF, 0xFE); // Wake up
    LL_mDelay(10);
    LT8960L_WriteReg(0x38, 0xBF, 0xFD); // Reset
    LL_mDelay(10);
    LT8960L_WriteReg(0x38, 0xBF, 0xFF); // Reset ready
    LL_mDelay(10);

    LT8960L_ReadReg(0); // Check reg value at address 0x00, which should be 0x6FE0
    printf("H:%02X, L:%02X\r\n", LT8960L_RegH, LT8960L_RegL);
    if (LT8960L_RegH != 0x6F || LT8960L_RegL != 0XE0)
    {
        printf("ReadReg 0 error\r\n");
        goto reTry;
    }

    LT8960L_WriteReg(1, 0x57, 0x81);
    LT8960L_WriteReg(26, 0x3A, 0x00);

    LT8960L_WriteReg(9, LT8960L_RF_Power >> 8, LT8960L_RF_Power & 0xff); // 设置发射功率
    
    LT8960L_WriteReg(28, 0x18, 0x00);    //频偏微调 0x1800~0x1807
    LT8960L_WriteReg(32, 0x48, 0x00);    //数据包配置3Byte前导 32bits同步字 NRZ格式
    LT8960L_WriteReg(35, 0x03, 0x00);    //重发3次=发1包 重发2包  最大15包
        
    LT8960L_WriteReg(36, SyncWord_Value[0], SyncWord_Value[1]);  //同步字配置
    LT8960L_WriteReg(39, SyncWord_Value[2], SyncWord_Value[3]);     //同步字配置
        
    LT8960L_WriteReg(40, 0x44, 0x02);        //允错1位
    LT8960L_WriteReg(41, 0xB0, 0x00);        //打开CRC校验 FIFO首字节是长度信息

    LT8960L_WriteReg(42, 0xFD, 0xB0);    
            
#ifdef Air_rate_1M        
    LT8960L_WriteReg(15, 0x65, 0xCC);
    LT8960L_WriteReg(17, 0x60, 0x00);

    LT8960L_WriteReg( 8, 0x6c, 0x90);
    LT8960L_WriteReg(44, 0x01, 0x00);
    LT8960L_WriteReg(45, 0x00, 0x80);
#endif



#ifdef Air_rate_250K    
    //LT8960L_WriteReg(15, 0x64, 0x4C);
    //LT8960L_WriteReg(17, 0x00, 0x00);
    
      LT8960L_WriteReg( 8, 0x6c, 0x90);
    LT8960L_WriteReg(44, 0x04, 0x00);
    LT8960L_WriteReg(45, 0x05, 0x52);
#endif



#ifdef Air_rate_125K    
        LT8960L_WriteReg(15, 0x64, 0x4C);
        LT8960L_WriteReg(17, 0x00, 0x00);
        
        LT8960L_WriteReg( 8, 0x6c, 0x90);
    LT8960L_WriteReg(44, 0x08, 0x00);
    LT8960L_WriteReg(45, 0x05, 0x52);
#endif
        
        
        
#ifdef Air_rate_62K5    
//        LT8960L_WriteReg(15, 0x64, 0x4C);
//        LT8960L_WriteReg(17, 0x00, 0x00);
        
    LT8960L_WriteReg( 8, 0x6c, 0x50);
    LT8960L_WriteReg(44, 0x10, 0x00);
    LT8960L_WriteReg(45, 0x05, 0x52);
#endif


    LT8960L_WriteReg(52, 0x80, 0x80);      

}

//=================LT8960L_BLE_INIT=====================
void LT8960L_BLE_INIT(void)
{
    //CLK PA1
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    //DAT PA0
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
    LL_mDelay(200);        //上电等待电源稳定
reTry:  
    LT8960L_WriteReg(0x38,0xBF,0xFE);    //唤醒LT8960L
    LL_mDelay(10);
    LT8960L_WriteReg(0x38,0xBF,0xFD);    //复位LT8960L
    LL_mDelay(10);
    LT8960L_WriteReg(0x38,0xBF,0xFF);    //复位就绪
    LL_mDelay(10);

    LT8960L_ReadReg(0); // 读取reg0是否为0x6FE0
    if (LT8960L_RegH != 0x6F || LT8960L_RegL != 0XE0)
    {
        printf("\nLT8960L Connect Fail \r\n");
        goto reTry; // 继续判断LT8960L的连接
    }

    LT8960L_WriteReg(1, 0x57, 0x81);
    LT8960L_WriteReg(26, 0x3A, 0x00);

    LT8960L_WriteReg(9, LT8960L_RF_Power >> 8, LT8960L_RF_Power & 0xff); // 设置发射功率

    LT8960L_WriteReg(28, 0x18, 0x00);    //频偏微调 0x1800~0x1807
    LT8960L_WriteReg(35, 0x03, 0x00);    //重发3次=发1包 重发2包  最大15包
        
    LT8960L_WriteReg(40, 0x44, 0x02);        //允错1位
    LT8960L_WriteReg(41, 0xB0, 0x00);        //打开CRC校验 FIFO首字节是长度信息

    LT8960L_WriteReg(42, 0xFD, 0xB0);    
    LT8960L_WriteReg(52, 0x80, 0x80);

    LT8960L_WriteReg(15, 0xec, 0x4c); // 开启ble模式
    LT8960L_WriteReg(32, 0x4A, 0x00);
    LT8960L_WriteReg(36, 0xBE, 0xD6); // 接入地址
    LT8960L_WriteReg(39, 0x8E, 0x89);
    LT8960L_WriteReg(44, 0x01, 0x01); // 1Mbps
    LT8960L_WriteReg(45, 0x00, 0x80);
}

unsigned char LT8960L_RegSetCPL;

void LT8960L_Change_0x38(void)
{
    if(LT8960L_RegSetCPL)
    {
        LT8960L_RegSetCPL=0;
        LT8960L_WriteReg(0x38,0xBC,0xDF);
    }
    else
    {
        LT8960L_RegSetCPL=1;
        LT8960L_WriteReg(0x38,0xBF,0xFF);
    }
}

uint8_t LT8960L_Analog=0;
void LT8960L_Change_Analog(void)
{
    if(LT8960L_Analog)
    {
        LT8960L_Analog=0;
        LT8960L_WriteReg(15, 0xEC, 0x4C);
        LT8960L_WriteReg(17, 0x00, 0x00);
    }
    else
    {
        LT8960L_Analog=1;
        LT8960L_WriteReg(15, 0xED, 0xCC);
        LT8960L_WriteReg(17, 0x63, 0x4F);
    }
}

void LT8960L_Carrier_Wave(unsigned char FreqChannel)
{
        LT8960L_WriteReg(7,0,0);
        LT8960L_WriteReg(28,0x18,0x02);
        LT8960L_WriteReg(44,0x01,0x00);            
        LT8960L_WriteReg(45,0x00,0x80);            
        
        LT8960L_WriteReg(41,0x00,0x00);
        LT8960L_WriteReg(52, 0x80, 0x80);
        
        for(char i=0;i<=16;i++)
                LT8960L_WriteReg(50, 0, 0);
        LT8960L_WriteReg(8,0x6C,0x90);
        LT8960L_WriteReg(7,1,FreqChannel+1);      
}
void LT8960L_Sleep(void)
{
        LT8960L_WriteReg(7,0,0);
      LT8960L_WriteReg(35,0X43,0x00);                    //LT8960L进入休眠
}

void LT8960L_Wakeup(void)
{
        //唤醒操作
        //LT8960L_WriteReg(0x38,0xBF,0xFE);    
        //LL_mDelay(10);
        LT8960L_INIT();                                            //重新复位LT8960L    
}

void LT8960L_TxData(unsigned char FreqChannel,unsigned char *pBuf,unsigned char length)
{
    LT8960L_WriteReg(7,0,FreqChannel);
    LT8960L_WriteReg(52, 0x80, 0x80);
    LT8960L_WriteBUF(50,pBuf,length);
    
    #ifdef Air_rate_62K5    
    LT8960L_WriteReg( 8, 0x6c, 0x50);
    #else
    LT8960L_WriteReg( 8, 0x6c, 0x90);
    #endif
    
    LT8960L_WriteReg(7,1,FreqChannel);
    
    LL_mDelay(0);
    while(LT8960L_GetPKT() ==0)
    LL_mDelay(0);
    
    LT8960L_WriteReg(7,0,FreqChannel);

}
void LT8960L_WriteBUF_BLE(unsigned char reg, unsigned char *pBuf, unsigned char len)
{
    unsigned char i;
    LT8960L_start();
    LT8960L_Send_Byte(reg);   LT8960L_ack();
    for(i=0; i<len; i++)
    {
        LT8960L_Send_Byte(pBuf[i]);
        LT8960L_ack();
    }
    
    LT8960L_stop();
}
unsigned char LT8960L_ReadBUF_BLE(unsigned char reg, unsigned char *pBuf)
{
    unsigned char i, len;
    LT8960L_start();
    LT8960L_Send_Byte(50 | 0x80);
    LT8960L_ack(); // Register address
    pBuf[0] = LT8960L_Read_Byte();
    LT8960L_ack_1uS();                   // pdu-type
    len = pBuf[1] = LT8960L_Read_Byte(); // pdu-length

    if (len > 37)
    {
        // DataOK_flag=2;
        // printf("Fifo overflow");
        len = 0; // error
        goto RxExit;
        // len=37;
    }

    for (i = 2; i < len + 2; i++)
    {
        LT8960L_ack_1uS();
        pBuf[i] = LT8960L_Read_Byte();
    }
    // DataOK_flag=1;

    RxExit:                    
        LT8960L_nack();
        LT8960L_stop();
        return len;
        
}
unsigned char RXBusy=0;
unsigned char Rx_TimeOUT=0;
unsigned char LT8960L_RxData(unsigned char FreqChannel,unsigned char *pBuf,unsigned char length)
{
    //1mS
    unsigned char Status=0,len;
    if(RXBusy==0)
    {
        RXBusy=1;
        LT8960L_WriteReg(7,0,FreqChannel);
        //LT8960L_Change_0x38();
        
        #ifdef Air_rate_62K5    
        LT8960L_WriteReg( 8, 0x6c, 0x50);
        #else
        LT8960L_WriteReg( 8, 0x6c, 0x90);
        #endif
        
        LT8960L_WriteReg(52, 0x80, 0x80);
        LT8960L_WriteReg(7,0,FreqChannel|0X80); 
        return Status;
    }
    if(LT8960L_GetPKT() && RXBusy)
    {
        LT8960L_ReadReg(48);
        RXBusy=0; 
        Rx_TimeOUT=0;
        if((LT8960L_RegH&0x80)==0)
        {
            len = LT8960L_ReadBUF(50,pBuf);
            return len;
        }
    }
    Rx_TimeOUT++;            //1mS++ / 或者放在定时器中断+1
    if(Rx_TimeOUT>100)
    {
        Rx_TimeOUT=0;
        RXBusy=0;
    }            
    return Status;    
}

void LT8960L_OpenRx(unsigned char FreqChannel)
{
    RXBusy=1;
    Rx_TimeOUT=0;
    LT8960L_WriteReg(7,0,FreqChannel);
    //LT8960L_Change_0x38();
    
    #ifdef Air_rate_62K5    
    LT8960L_WriteReg( 8, 0x6c, 0x50);
    #else
    LT8960L_WriteReg( 8, 0x6c, 0x90);
    #endif
    
    LT8960L_WriteReg(52, 0x80, 0x80);
    LT8960L_WriteReg(7,0,FreqChannel|0X80); 
}

void LT8960L_Tx_BLE_Data(unsigned char *pBuf,unsigned char length,unsigned char ChannelEnable)
{
    
    LT8960L_WriteReg(7,0x00,0x00);  //IDLE 
    LT8960L_WriteReg(52,0x80,0x80);  //Clean fifo
    LT8960L_WriteBUF_BLE(50,pBuf,length);                

    if(ChannelEnable&0x01)
    {
        //ble ch37 tx
        LT8960L_WriteReg(46, 0x25, 0x00); 
        LT8960L_WriteReg(7, 0x01,0);
        do
        {
            LL_mDelay(0);
        }
        while(LT8960L_GetPKT()==0);    
    }        


    if(ChannelEnable&0x02)
    {
        //ble ch38 tx
        LT8960L_WriteReg(46, 0x26, 0x00); 
        LT8960L_WriteReg(7, 0x01,24);
        do
        {
            LL_mDelay(0);
        }
        while(LT8960L_GetPKT()==0);            
    }

    if(ChannelEnable&0x04)
    {
        //ble ch39 tx
        LT8960L_WriteReg(46, 0x27, 0x00);
        LT8960L_WriteReg(7, 0x01,78);        
        do
        {
            LL_mDelay(0);
        }
        while(LT8960L_GetPKT()==0);        
    }        

    LT8960L_WriteReg(7,0x00,0x00);  //IDLE 

}
unsigned char Rx_TimeBLE=0;
unsigned char Rx_ChannelBLE=0,Rx_ChannelMAP;
unsigned char LT8960L_Rx_BLE_Data(unsigned char *pBuf,unsigned char length,unsigned char ChannelEnable)
{
    unsigned char Status=0,len;
    if(RXBusy==0)
    {
        RXBusy=1;
        
        LT8960L_WriteReg(7,0,0);
        //LT8960L_Change_0x38();
        LT8960L_WriteReg(8,0x6c,0x90);
        LT8960L_WriteReg(52, 0x80, 0x80);
        
        
        //跳频/定频
        if(Rx_ChannelBLE==0)
        {
            if(ChannelEnable&0x01)
            {
                Rx_ChannelMAP = 1;//CH37
            }
            else if(ChannelEnable&0x02)
            {
                Rx_ChannelMAP = 2;//CH38
            }
            else if(ChannelEnable&0x04)
            {
                Rx_ChannelMAP = 4;//CH39
            }
            Rx_ChannelBLE=1;
        }
        else if(Rx_ChannelBLE==1)
        {
            if(ChannelEnable&0x02)
            {
                Rx_ChannelMAP = 2;
            }
            else if(ChannelEnable&0x04)
            {
                Rx_ChannelMAP = 4;
            }
            else if(ChannelEnable&0x01)
            {
                Rx_ChannelMAP = 1;
            }
            Rx_ChannelBLE=2;            
        }
        else if(Rx_ChannelBLE==2)
        {
            if(ChannelEnable&0x04)
            {
                Rx_ChannelMAP = 4;
            }
            else if(ChannelEnable&0x01)
            {
                Rx_ChannelMAP = 1;
            }
            else if(ChannelEnable&0x02)
            {
                Rx_ChannelMAP = 2;
            }
            Rx_ChannelBLE=0;            
        }                
        
        if(Rx_ChannelMAP&0x01)
        {
            LT8960L_WriteReg(46, 0x25, 0x00); //BLE 37
            LT8960L_WriteReg(7, 0x00,0|0x80); // RX=2402MHz            
        }
        if(Rx_ChannelMAP&0x02)
        {
            LT8960L_WriteReg(46, 0x26, 0x00); //BLE 38
            LT8960L_WriteReg(7, 0x00,24|0x80); // RX=2426MHz            
        }        
        if(Rx_ChannelMAP&0x04)
        {
            LT8960L_WriteReg(46, 0x27, 0x00); //BLE 39
            LT8960L_WriteReg(7, 0x00,78|0x80); // RX=2480MHz            
        }    

        
        return 0;
    }
    if(LT8960L_GetPKT() && RXBusy)
    {
        LT8960L_ReadReg(48);
        RXBusy=0; 
        if((LT8960L_RegH&0x80)==0)
        {
            len = LT8960L_ReadBUF_BLE(50,pBuf);
            
            if(len)
                Rx_TimeBLE=0;
            
            return len;
        }
    }    
    //Rx_TimeBLE++;            //1mS++ / 或者放在定时器中断+1
    if(Rx_TimeBLE>100)
    {
        Rx_TimeBLE=0;
        RXBusy=0;
        LT8960L_Change_Analog();
    }            
    return Status;    
    
}
unsigned char LT8960L_GetPKT(void)    //读取PKT复用输出
{

    if(DAT_IO_Type==0)
    {
        LT8960L_CLK_L; 
        LT8960L_DAT_H;        
        
        LT8960L_DAT_Input();    
    }
    if (LT8960L_DAT_read)
    {
        LT8960L_CLK_H; 
        return 1;
        
    }
    else
    {
        return 0;
    }
}






