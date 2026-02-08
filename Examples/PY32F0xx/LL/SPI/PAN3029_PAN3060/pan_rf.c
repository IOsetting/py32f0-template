/**
 * @file      pan_rf.c
 * @brief     PAN3029/PAN3060 driver implementation
 * @version   V1.0.1
 * @date      2025-08-18
 * @copyright Panchip Microelectronics Co., Ltd. All rights reserved.
 * @code
 *             ____              ____ _     _
 *            |  _ \ __ _ _ __  / ___| |__ (_)_ __
 *            | |_) / _` | '_ \| |   | '_ \| | '_ \
 *            |  __/ (_| | | | | |___| | | | | |_) |
 *            |_|   \__,_|_| |_|\____|_| |_|_| .__/
 *                                           |_|
 *            (C)2009-2025 PanChip
 * @endcode
 * @author    PanChip
 * @note      The encoding of this file is utf-8.
 */
#include <stdio.h>
#include <string.h>
#include "pan_param.h"
#include "pan_rf.h"

/**
 * @brief PAN3029/3060接收数据包结构体
 * @note 该结构体用于存储接收到的数据包，包括数据长度、数据缓冲区、SNR和RSSI等信息
 */
volatile RfRxPkt_t g_RfRxPkt = {0};

/**
 * @brief PAN3029/3060配置参数结构体
 * @note 该结构体用于存储PAN3029/3060的配置参数，包括发射功率、频率、扩频因子、带宽、编码率等
 */
static volatile RfConfig_t g_RfCfgParams = {0};

/**
 * @brief 保存当前的RF操作状态
 */
static volatile RfOpState_t g_RfOperatetate;

/**
 * @brief 微秒级延时函数
 * @param us 延时的微秒数
 * @note 该函数具体的实现需要根据实际硬件平台进行修改。
 * @note 须保证RF_DelayUs(1)大于等于1us。
 */
__attribute__((optimize("O0"))) void RF_DelayUs(uint32_t us)
{
    volatile int i, j;
    for (i = 0; i < us; i++)
    {
        for (j = 0; j < 12; j++)
        {
            __NOP(); /* 每个循环约1 cycle */
        }
    }
}

/**
 * @brief 毫秒级延时函数
 * @param ms 延时的毫秒数
 * @note 该函数具体的实现需要根据实际硬件平台进行修改。
 * @note 须保证RF_DelayMs(1)大于等于1ms。
 */
void RF_DelayMs(uint32_t ms)
{
    LL_mDelay(ms); /* 调用微秒延时函数 */
}

/**
 * @brief SPI 写入单个字节
 * @param Value 要写入的单字节数据
 * @note 此接口为软件SPI实现，具体实现方式可能因MCU平台而异。
 * @note 调用该函数前，必须先调用 SPI_CS_LOW() 函数拉低 CS 引脚。
 * @note 该函数会将 SPI_MOSI 引脚设置为输出模式，并在传输完成后将其设置为输入模式。
 * @note PAN3029/3060 SPI 接口使用的是 3 线 SPI 模式。
 * @note PAN3029/3060 SPI 配置为：
 *       时钟极性：低电平有效
 *       时钟相位：第一个边沿采样数据
 *       数据传输顺序：MSB优先
 * @note 以发送0xCC数据为例，时序图如下：
 *      SPI_CS：  ____________________________________________________
 *      SPI_CLK:  ____|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__
 *      SPI_MOSI: ___|‾‾‾‾‾‾‾‾‾‾|___________|‾‾‾‾‾‾‾‾‾‾‾|_____________
 *      BIT DATA:     1     1     0     0      1     1      0     0
 */
void SPI_WriteByte(uint8_t Value)
{
    /* TODO: 根据你的硬件实现，下面是软件SPI实现示例 */
#if INTERFACE_MODE == USE_SPI_4LINE
    SPI_TxRxByte(Value);
#elif INTERFACE_MODE == USE_SPI_3LINE
    unsigned char i;

    SPI_MOSI_OUTPUT(); /* 设置SPI_MOSI引脚为输出模式 */

    /* 以高比特优先MSB方式传输8位数据 */
    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW();    /* 拉低时钟线，准备传输数据 */
        if (Value & 0x80) /* 判断最高位数据 */
        {
            SPI_MOSI_HIGH(); /* 如果最高位为1，则发送高电平 */
        }
        else
        {
            SPI_MOSI_LOW(); /* 如果最高位为0，则发送低电平 */
        }

        Value <<= 1;    /* 左移一位，准备传输下一个比特 */
        SPI_SCK_HIGH(); /* 拉高时钟线，传输一位数据 */
    }

    SPI_MOSI_INPUT(); /* 设置SPI_MOSI引脚为输入模式 */
    SPI_SCK_LOW();    /* 结束传输，拉低时钟线，保证时钟线处于低电平 */
#endif
}

/**
 * @brief SPI 读取单个字节
 * @return unsigned char 读取的数据字节
 * @note 此接口为软件SPI实现，具体实现方式可能因MCU平台而异。
 * @note 调用该函数前，必须先调用 SPI_CS_LOW() 函数拉低 CS 引脚。
 * @note 该函数会将 SPI_MOSI 引脚设置为输出模式，并在传输完成后将其设置为输入模式。
 * @note PAN3029/3060 SPI 接口使用的是 3 线 SPI 模式。
 * @note PAN3029/3060 SPI 配置为：
 *       时钟极性：低电平有效
 *       时钟相位：第一个边沿采样数据
 *       数据传输顺序：MSB优先
 * @note 以发送0x33数据为例，时序图如下：
 *      SPI_CS：  ___________________________________________________
 *      SPI_CLK:  ___|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__
 *      SPI_MISO: _____________|‾‾‾‾‾‾‾‾‾‾‾|___________|‾‾‾‾‾‾‾‾‾‾‾‾‾
 *      BIT DATA:    0     0     1     1     0     0     1     1     
 */
unsigned char SPI_ReadByte(void)
{
    unsigned char readByte = 0;

    /* TODO: 根据你的硬件实现，下面是软件SPI实现示例 */
#if INTERFACE_MODE == USE_SPI_4LINE
    readByte = SPI_TxRxByte(0xFF);
#elif INTERFACE_MODE == USE_SPI_3LINE
    unsigned char i;
    unsigned char Value = 0;

    /* 以高比特优先MSB方式接收8位数据 */
    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW();  /* 拉低时钟线，准备接收数据 */
        Value <<= 1;  /* 左移一位，准备接收下一个比特 */
        SPI_SCK_HIGH(); /* 拉高时钟线，触发PAN3029 SPI发送数据 */
        if (SPI_MOSI_STATUS())
        {
            Value |= 0x01;
        }
    }
    SPI_SCK_LOW(); /* 结束传输，拉低时钟线，保证时钟线处于低电平 */
    readByte = Value; /* 将接收到的数据存储到readByte中 */
#endif
    return readByte;
}

/**
 * @brief Get the number of trailing zeros in a byte
 * @param Value The value to check
 * @return The number of trailing zeros
 */
uint8_t __ctz(uint8_t Value)
{
    int i;

    for (i = 0; i < 8; ++i)
    {
        if ((Value >> i) & 1)
            return (uint8_t)i;
    }
    
    return 0;
}

/**
 * @brief 从指定的寄存器读取单个字节
 * @param Addr 要读取的寄存器地址
 * @return uint8_t 从寄存器读取的值
 * @note 该函数的SPI_CS_LOW()、SPI_CS_HIGH()、SPI_WriteByte()
 *       和SPI_ReadByte()函数需要根据实际硬件实现进行修改。
 */
uint8_t RF_ReadReg(uint8_t Addr)
{
    uint8_t Temp;
    SPI_CS_LOW();                      /* 片选信号拉低，开始SPI传输 */
    SPI_WriteByte((Addr << 1) & 0xFE); /* Bit7:0为地址，Bit0为读写位，Bit0=0表示读操作 */
    Temp = SPI_ReadByte();             /* 读取寄存器值 */
    SPI_CS_HIGH();                     /* 片选信号拉高，结束SPI传输 */

    return Temp;
}

/**
 * @brief 写入单个字节到指定寄存器
 * @param Addr 要写入的寄存器地址
 * @param Value 要写入寄存器的单字节数据
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 * @note 该函数的SPI_CS_LOW()、SPI_CS_HIGH()、SPI_WriteByte()
 *       和SPI_ReadByte()函数需要根据实际硬件实现进行修改。
 */
RF_Err_t RF_WriteReg(uint8_t Addr, uint8_t Value)
{
    SPI_CS_LOW();                      /* 片选信号拉低，开始SPI传输 */
    SPI_WriteByte((Addr << 1) | 0x01); /* Bit7:1为地址，Bit0为读写位，Bit0=1表示写操作 */
    SPI_WriteByte(Value);              /* 写入寄存器值 */
    SPI_CS_HIGH();                     /* 片选信号拉高，结束SPI传输 */

#if USE_RF_REG_CHECK /* 是否使用寄存器回读确认功能 */
    /**
     * 该部分代码的作用是读取寄存器的值，并与写入的值进行比较，如果不相等则打印错误信息
     * 代码调试完成后可以注释掉或者删除掉。
     */
    {
        uint8_t Temp = RF_ReadReg(Addr);
        if (Temp == Value)
        {
            // printf("Write reg ok: 0x%02x, 0x%02x, 0x%02x\n", Addr, Value, Temp);
            return RF_OK; /* 写入的值与读取的值相等，返回操作成功 */
        }
        else
        {
            /* 读取的值与写入的值不相等，返回错误 */
            printf("Write reg fail: 0x%02x, 0x%02x, 0x%02x\r\n", Addr, Value, Temp);
            return RF_FAIL;
        }
    }
#else
    return RF_OK;
#endif
}

/**
 * @brief 连续写入多个字节到指定寄存器区
 * @param Addr 要写入的寄存器区的起始地址
 * @param Buffer 要写入寄存器的缓冲区指针
 * @param Size 要写入的字节数
 * @note 该函数的SPI_CS_LOW()、SPI_CS_HIGH()、SPI_WriteByte()函数需要根据实际硬件实现进行修改。
 */
void RF_WriteRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
    unsigned char i;
    SPI_CS_LOW();                      /** 片选信号拉低，开始SPI传输 */
    SPI_WriteByte((Addr << 1) | 0x01); /** Bit7:1为地址，Bit0为读写位，Bit0=1表示写操作 */
    for (i = 0; i < Size; i++)
    {
        SPI_WriteByte(Buffer[i]); /** 写入寄存器值 */
    }
    SPI_CS_HIGH(); /** 片选信号拉高，结束SPI传输 */
}

/**
 * @brief 从指定的寄存器连续读取多个字节
 * @param Addr 要读取的寄存器地址
 * @param Buffer 存储读取数据的缓冲区指针
 * @param Size 要读取的字节数
 * @note 该函数的SPI_CS_LOW()、SPI_CS_HIGH()、SPI_WriteByte()
 *       和SPI_ReadByte()函数需要根据实际硬件实现进行修改。
 */
void RF_ReadRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
    unsigned char i;
    SPI_CS_LOW();                     /* 片选信号拉低，开始SPI传输 */
    SPI_WriteByte((Addr << 1) & 0xFE); /* Bit7:0为地址，Bit0为读写位，Bit0=0表示读操作 */
    for (i = 0; i < Size; i++)
    {
        Buffer[i] = SPI_ReadByte(); /* 读取寄存器值 */
    }
    SPI_CS_HIGH(); /* 片选信号拉高，结束SPI传输 */
}

/**
 * @brief 选择寄存器页
 * @param Page 要选择的寄存器页，页范围0~3
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 * @note 如果当前页已经是所需页，则无需再配置寄存器
 */
RF_Err_t RF_SetPage(uint8_t Page)
{
    static uint8_t gCurrPage = 0xFF;
    if(gCurrPage == Page)
    {
        return RF_OK;
    }
    gCurrPage = Page;
    RF_ASSERT(RF_WriteReg(0x00, gCurrPage)); /* 选择寄存器页 */
    return RF_OK;
}

/**
 * @brief 写入单个字节到指定页的寄存器
 * @param Page 要写入的寄存器页，页范围0~3
 * @param Addr 要写入的寄存器地址
 * @param Value 要写入寄存器的单字节数据
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 */
RF_Err_t RF_WritePageReg(uint8_t Page, uint8_t Addr, uint8_t Value)
{
    RF_SetPage(Page);
    RF_WriteReg(Addr, Value);

    return RF_OK;
}

/**
 * @brief 写入多个字节到指定页的寄存器区间
 * @param Page 要写入的寄存器页，页范围0~3
 * @param Addr 要写入的寄存器地址
 * @param Buffer 要写入寄存器的缓冲区指针
 * @param Size 要写入的字节数
 */
void RF_WritePageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
    RF_SetPage(Page);                 /* 选择寄存器页 */
    RF_WriteRegs(Addr, Buffer, Size); /* 写入寄存器值 */
}

/**
 * @brief 从指定页的寄存器读取单个字节
 * @param Page 要读取的寄存器页，页范围0~3
 * @param Addr 要读取的寄存器地址
 * @return uint8_t 从寄存器读取的值
 */
uint8_t RF_ReadPageReg(uint8_t Page, uint8_t Addr)
{
    RF_SetPage(Page);
    return RF_ReadReg(Addr);
}

/**
 * @brief 从指定页的寄存器区间读取多个字节
 * @param Page 要读取的寄存器页，页范围0~3
 * @param Addr 要读取的寄存器地址
 * @param Buffer 存储读取数据的缓冲区指针
 * @param Size 要读取的字节数
 */
void RF_ReadPageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
    RF_SetPage(Page);                /* 选择寄存器页 */
    RF_ReadRegs(Addr, Buffer, Size); /* 读取寄存器值 */
}

/**
 * @brief 置位指定页的寄存器位
 * @param Page 要读取的寄存器页，页范围0~3
 * @param Addr 要设置的寄存器地址
 * @param Mask 要设置的位掩码
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 */
RF_Err_t RF_SetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask)
{
    uint8_t Temp;

    RF_SetPage(Page);
    Temp = RF_ReadReg(Addr);
    RF_WriteReg(Addr, Temp | Mask);

    return RF_OK;
}

/**
 * @brief 复位指定页的寄存器位
 * @param Page 要读取的寄存器页，页范围0~3
 * @param Addr 要重置的寄存器地址
 * @param Mask 要重置的位掩码
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 */
RF_Err_t RF_ResetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask)
{
    uint8_t Temp;

    RF_SetPage(Page);                  /* 选择寄存器页 */
    Temp = RF_ReadReg(Addr);           /* 读取寄存器值 */
    RF_WriteReg(Addr, Temp & (~Mask)); /* 清除寄存器中与掩码对应的位 */

    return RF_OK;
}

/**
 * @brief 写入指定页的寄存器位
 * @param Page 要读取的寄存器页，页范围0~3
 * @param Addr 要写入的寄存器地址
 * @param Value 要写入的值
 * @param Mask 要写入的位掩码
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 * @note 该函数会先清除寄存器中与掩码对应的位，然后再设置新的值
 * @note 比如要设置第1页0x08寄存器的第2位和第3位为0b10，其他位不变，可以调用
 *       RF_WritePageRegBits(1, 0x08, 0x02, 0x0C);
 *       其中Value = 0x02, Mask = 0x0C，Value不需要左移，因为掩码已经指定了要设置的位
 */
RF_Err_t RF_WritePageRegBits(uint8_t Page, uint8_t Addr, uint8_t Value, uint8_t Mask)
{
    uint8_t Temp;
    uint8_t shift = __ctz(Mask); /* 获取掩码的位移值 */

    Value <<= shift; /* 将值左移到掩码对应的位置 */
    Value &= Mask;   /* 将值与掩码进行与操作，确保只设置掩码对应的位 */

    RF_SetPage(Page);                            /* 选择寄存器页 */
    Temp = RF_ReadReg(Addr);                     /* 读取寄存器值 */
    RF_WriteReg(Addr, (Temp & (~Mask)) | Value); /* 清除寄存器中与掩码对应的位，然后设置新的值 */

    return RF_OK;
}

/**
 * @brief 配置GPIO模式
 * @param <GpioPin> 引脚号
 *        <GpioMode> GPIO模式
 *         - GPIO_MODE_INPUT: 输入模式
 *         - GPIO_MODE_OUTPUT: 输出模式
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 */
RF_Err_t RF_ConfigGpio(uint8_t GpioPin, uint8_t GpioMode)
{
    if(GpioMode == GPIO_MODE_INPUT)
    {
        if(GpioPin < 8)
        {
            RF_ASSERT(RF_SetPageRegBits(0, 0x63, (1 << GpioPin)));
        }
        else
        {
            RF_ASSERT(RF_SetPageRegBits(0, 0x64, (1 << (GpioPin - 8))));
        }
    }
    else if(GpioMode == GPIO_MODE_OUTPUT)
    {
        if(GpioPin < 8)
        {
            RF_SetPageRegBits(0, 0x65, (1 << GpioPin));
        }
        else
        {
            RF_SetPageRegBits(0, 0x66, (1 << (GpioPin - 8)));
        }
    }
    else
    {
        return RF_FAIL;
    }

    return RF_OK;
}

/**
 * @brief 控制GPIO输出电平
 * @param <GpioPin> 引脚号
 *        <Level>   GPIO电平
 *         - 0: 低电平
 *         - 1: 高电平
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 */
RF_Err_t RF_WriteGpioLevel(uint8_t GpioPin, uint8_t Level)
{
    if(GpioPin < 8)
    {
        RF_WritePageRegBits(0, 0x67, Level, (1 << GpioPin));
    }
    else
    {
        RF_WritePageRegBits(0, 0x68, Level, (1 << (GpioPin - 8)));
    }

    return RF_OK;
}

/**
 * @brief 读取GPIO电平
 * @param <GpioPin> 引脚号
 * @return 读取的GPIO电平
 *         - 0: 低电平
 *         - 1: 高电平
 */
uint8_t RF_ReadGpioLevel(uint8_t GpioPin)
{
    uint8_t Temp;

    if(GpioPin < 6)
    {
        Temp = RF_ReadPageReg(0, 0x74);
    }
    else
    {
        Temp = RF_ReadPageReg(0, 0x75);
        GpioPin -= 6;
    }

    return (bool)((Temp >> GpioPin) & 0x01);
}

/**
 * @brief 初始化PAN3029/3060的天线控制GPIO
 * @note 该函数用于初始化PAN3029/3060的天线控制GPIO，将其配置为输出模式，并设置初始电平为低
 * @note 如果使用MCU的GPIO控制天线开关时，则需要重新适配该函数
 */
void RF_InitAntGpio(void)
{
    RF_ConfigGpio(MODULE_GPIO_RX, GPIO_MODE_OUTPUT);
    RF_ConfigGpio(MODULE_GPIO_TX, GPIO_MODE_OUTPUT);

    RF_WriteGpioLevel(MODULE_GPIO_RX, 0);
    RF_WriteGpioLevel(MODULE_GPIO_TX, 0);
}

/**
 * @brief 打开PAN3029/3060的发射天线
 * @note 该函数用于打开PAN3029/3060的发射天线，将TX引脚设置为高电平，RX引脚设置为低电平
 * @note 如果使用MCU的GPIO控制天线开关时，则需要重新适配该函数
 */
void RF_TurnonTxAnt(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_RX, 0);
    RF_WriteGpioLevel(MODULE_GPIO_TX, 1);
}

/**
 * @brief 打开PAN3029/3060的接收天线
 * @note 该函数用于打开PAN3029/3060的接收天线，将RX引脚设置为高电平，TX引脚设置为低电平
 * @note 如果使用MCU的GPIO控制天线开关时，则需要重新适配该函数
 */
void RF_TurnonRxAnt(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_TX, 0);
    RF_WriteGpioLevel(MODULE_GPIO_RX, 1);
}

/**
 * @brief 关闭PAN3029/3060的天线
 * @note 该函数用于关闭PAN3029/3060的天线，将RX和TX引脚都设置为低电平
 * @note 如果使用MCU的GPIO控制天线开关时，则需要重新适配该函数
 */
void RF_ShutdownAnt(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_RX, 0);
    RF_WriteGpioLevel(MODULE_GPIO_TX, 0);
}

/**
 * @brief 初始化TCXO控制GPIO
 * @note 该函数用于初始化PAN3029/3060的TCXO控制GPIO，将其配置为输出模式，并设置初始电平为高
 * @note 如果使用MCU的GPIO控制TCXO开关时，则需要重新适配该函数
 */
void RF_InitTcxoGpio(void)
{
    RF_ConfigGpio(MODULE_GPIO_TCXO, GPIO_MODE_OUTPUT);
    RF_WriteGpioLevel(MODULE_GPIO_TCXO, 1);
}

/**
 * @brief 打开TCXO的供电电源
 * @note 该函数用于打开PAN3029/3060的TCXO，将TCXO引脚设置为高电平
 * @note 如果使用MCU的GPIO控制TCXO开关时，则需要重新适配该函数
 */
void RF_TurnonTcxo(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_TCXO, 1);
}

/**
 * @brief 关闭TCXO的供电电源
 * @note 该函数用于关闭PAN3029/3060的TCXO，将TCXO引脚设置为低电平
 * @note 如果使用MCU的GPIO控制TCXO开关时，则需要重新适配该函数
 */
void RF_TurnoffTcxo(void)
{
    RF_WriteGpioLevel(MODULE_GPIO_TCXO, 0);
}

/**
 * @brief Enable LDO PA
 */
void RF_TurnonLdoPA(void)
{    
    RF_SetPageRegBits(0, 0x4F, 0x08);
}

/*
 * @brief Disable LDO PA
 */
void RF_TurnoffLdoPA(void)
{
    RF_ResetPageRegBits(0, 0x4F, 0x08);
}

/**
 * @brief 关闭内部和外部PA
 */
void RF_TurnoffPA(void)
{
    RF_TurnoffLdoPA(); /* 关闭内部PA */
    RF_ShutdownAnt();  /* 关闭外部PA */
    /* 发射完成后，若配置为DCDC电源模式，则需要切换回DCDC电源模式 */
    if(g_RfCfgParams.RegulatorMode == USE_DCDC)
    {
        RF_WritePageReg(3, 0x24, 0x08);
    }
}

/**
 * @brief 打开内部和外部PA
 */
void RF_TurnonPA(void)
{
    /* 若当前为DCDC电源模式，发射前须切换至LDO电源模式 */
    if(g_RfCfgParams.RegulatorMode == USE_DCDC)
    {
        RF_WritePageReg(3, 0x24, 0x00);
    }
    RF_TurnonLdoPA(); /* 打开内部PA */
    RF_TurnonTxAnt(); /* 打开外部PA */
}

/**
 * @brief 设置芯片模式
 * @param <ChipMode> 芯片模式
 *        - CHIPMODE_MODE0
 *        - CHIPMODE_MODE1
 */
void RF_SetChipMode(RfChipMode_t ChipMode)
{
    if(ChipMode == CHIPMODE_MODE0)
    {
        /* Mode0 config */
        RF_WritePageRegBits(1, 0x25, 0, 0xF0);
        RF_WritePageRegBits(1, 0x25, 0, 0x08);
        RF_WritePageRegBits(3, 0x12, 1, 0x04);
        RF_WritePageRegBits(3, 0x12, 1, 0x10);
        RF_WritePageRegBits(0, 0x58, 1, 0x04); /* Enable crc interrupt */
    }
    else
    {
        /* Mode1 config */
        RF_WritePageRegBits(1, 0x25, 4, 0xF0);
        RF_WritePageRegBits(1, 0x25, 1, 0x08);
        RF_WritePageRegBits(3, 0x12, 0, 0x04);
        RF_WritePageRegBits(3, 0x12, 0, 0x10);
        RF_WritePageRegBits(0, 0x58, 0, 0x04); /* Disable crc interrupt */
    }
    g_RfCfgParams.ChipMode = ChipMode;
}

/**
 * @brief 获取芯片模式
 * @param -
 */
RfChipMode_t RF_GetChipMode(void)
{
    return g_RfCfgParams.ChipMode;
}

/**
 * @brief 从信息区读取字节
 * @param <Addr> 寄存器地址
 *        <Pattern> 模式匹配值
 *        <InfoAddr> 信息区地址
 * @return 从信息区读取的字节值
 */
uint8_t RF_ReadInfoByte(uint8_t Addr, uint16_t Pattern, uint8_t InfoAddr)
{
    uint8_t Value;
    uint8_t Buffer[3];
    uint16_t Timeout = 10000;

    Buffer[0] = Pattern >> 8;
    Buffer[1] = Pattern & 0xFF;
    Buffer[2] = InfoAddr << 1;

    RF_WritePageRegs(2, Addr, Buffer, sizeof(Buffer));
    do
    {
        if (RF_ReadPageReg(0, 0x6C) & 0x80)
        {
            break;
        }
    } while (Timeout--);

    Value = RF_ReadPageReg(2, Addr);

    return Value;
}

/**
 * @brief 校准RF相关参数
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 */
RF_Err_t RF_Calibrate(void)
{
    int i;
    uint8_t Temp[3] = {0};

    /* Temp[0]: efuse[0x1E] - DCDCIMAX
       Temp[1]: efuse[0x1F] - DCDCREF
       Temp[2]: efuse[0x20] - PABIAS */
    RF_ResetPageRegBits(2, 0x3E, 0x08); // Unlock info
    for (i = 0; i < sizeof(Temp); i++)
    {
        Temp[i] = RF_ReadInfoByte(0x3B, 0x5AA5, 0x1E + i);
    }

    if (RF_ReadInfoByte(0x3B, 0x5AA5, 0x1C) == 0x5A)
    {
        RF_WritePageReg(2, 0x3D, 0xFD);

        if (Temp[2] != 0)
        {
            /* Write PABIAS */
            RF_WritePageReg(0, 0x45, Temp[2]);
        }

        RF_WritePageReg(3, 0x1C, (0xC0 | (Temp[0] & 0x1F))); /* Write DCDCIMAX */
        RF_WritePageReg(3, 0x1D, Temp[1]);                   /* Write DCDCREF */
    }
    RF_ASSERT(RF_SetPageRegBits(2, 0x3E, 0x08)); // Lock info

    return RF_OK;
}

/**
 * @brief Configure AGC function
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 */
RF_Err_t RF_ConfigAgc(void)
{
    /* Enable AGC function
       - [Page2][0x06][Bit0] equal to 0 means enable AGC function
       - [Page2][0x06][Bit1] equal to 1 means disable AGC function */
    RF_ASSERT(RF_ResetPageRegBits(2, 0x06, 0x01));

#if REGION_DEFAULT == REGION_CN470_510
    RF_WritePageRegs(2, 0x0A, (uint8_t *)g_LowFreqAgcCfg, 40);
#elif REGION_DEFAULT == REGION_EU_863_870 || REGION_DEFAULT == REGION_US_902_928 
    RF_WritePageRegs(2, 0x0A, (uint8_t *)g_HighFreqAgcCfg, 40);
#endif

    RF_ASSERT(RF_WritePageReg(2, 0x34, 0xEF));

    return RF_OK;
}

/**
 * @brief 配置RF寄存器的默认参数
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 */
RF_Err_t RF_ConfigDefaultParams(void)
{
    int i;
    for(i = 0; i < sizeof(g_RfDefaultConfig)/sizeof(PAN_RegCfg_t); i++)
    {
        RF_WritePageReg(g_RfDefaultConfig[i].Page, g_RfDefaultConfig[i].Addr, g_RfDefaultConfig[i].Value);
    }
    return RF_OK;
}

/**
 * @brief 上电后初始化RF收发器到STB3状态
 * @return RF_Err_t 返回操作结果
 *         - RF_OK: 操作成功
 *         - RF_FAIL: 操作失败
 * @note 调用此函数前需要先配置好MCU的SPI和相关GPIO引脚
 */
RF_Err_t RF_Init(void)
{
#if USE_RF_RST_GPIO == 1
    RF_RESET_PIN_LOW();  /* 拉低芯片复位引脚，开始复位 */
    RF_DelayUs(100);     /* 保证实际延时在100us以上 */
    RF_RESET_PIN_High(); /* 拉高芯片复位引脚，释放复位 */
    RF_DelayUs(100);     /* 保证实际延时在100us以上 */
#endif
    /* [Pagex][0x04][BIT4]为复位控制，为0时复位芯片，为1时复位释放 */
    RF_WriteReg(0x04, 0x06); /* 开始POR复位芯片*/
    RF_DelayUs(100);         /* 保证实际延时在100us以上 */

#if INTERFACE_MODE == USE_SPI_4LINE
    RF_WriteReg(0x00, 0x03); /* 选择寄存器页3 */
    RF_WriteReg(0x1A, 0x03); /* 使能4line SPI */
#elif INTERFACE_MODE == USE_SPI_3LINE
    RF_WriteReg(0x00, 0x03); /* 选择寄存器页3 */
    RF_WriteReg(0x1A, 0x83); /* 使能3ine SPI */
#endif

    RF_SetPage(0);                                    /* 选择寄存器页0 */
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_DEEPSLEEP)); /* 进入deepsleep状态 */
    RF_DelayUs(10);                                   /* 保证实际延时在10us以上 */
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_SLEEP));     /* 进入sleep状态 */
    RF_DelayUs(10);                                   /* 保证实际延时在10us以上 */
    RF_ASSERT(RF_WritePageReg(3, 0x06, 0x20));        /* 使能ISO */
    RF_DelayUs(10);                                   /* 保证实际延时在10us以上 */
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_STB1));      /* 进入stb1状态 */
    RF_DelayUs(10);                                   /* 保证实际延时在10us以上 */
#if USE_ACTIVE_CRYSTAL == 1                           /* 如果使用有源晶振，则需要配置TCXO GPIO引脚 */
    RF_ASSERT(RF_WritePageReg(3, 0x26, 0xA0));        /* 使能内核电源，并打开有源晶振通道 */
    RF_DelayUs(100);                                  /* 保证实际延时在100us以上 */
    RF_ASSERT(RF_WriteReg(0x04, 0x36));               /* 使能LFT并释放POR复位 */
    RF_DelayMs(1);                                    /* 保证实际延时在1ms以上 */
    RF_InitTcxoGpio();                                /* 初始化TCXO GPIO引脚 */
#else
    RF_ASSERT(RF_WritePageReg(3, 0x26, 0x20));        /* 使能内核电源 */
    RF_DelayUs(100);                                  /* 保证实际延时在100us以上 */
    RF_ASSERT(RF_WriteReg(0x04, 0x36));               /* 使能LFT并释放POR复位 */
    RF_DelayMs(1);                                    /* 保证实际延时在1ms以上 */
#endif
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_STB2));      /* 进入stb2状态 */
    RF_DelayMs(1);                                    /* 保证实际延时在1ms以上 */
    RF_ASSERT(RF_WriteReg(0x02, RF_STATE_STB3));      /* 进入stb3状态 */
    RF_DelayUs(100);                                  /* 保证实际延时在100us以上 */
    RF_ASSERT(RF_ConfigDefaultParams());              /* 配置RF寄存器的默认参数 */
    RF_ASSERT(RF_Calibrate());                        /* 校准RF相关参数 */
    RF_ASSERT(RF_ConfigAgc());                        /* 配置AGC功能 */
    RF_InitAntGpio();                                 /* 初始化天线GPIO引脚 */
    g_RfOperatetate = RF_STATE_STB3;                  /* 设置当前工作状态为STB3 */

    return RF_OK;
}

/**
 * @brief 配置RF芯片的用户参数
 */
void RF_ConfigUserParams(void)
{
    RF_SetTxPower(22);                    /* 设置功率档位 */
    RF_SetFreq(RF_FREQ_DEFAULT);          /* 设置频率 */
    RF_SetBW(RF_BW_DEFAULT);              /* 设置带宽 */
    RF_SetSF(RF_SF_DEFAULT);              /* 设置扩频因子 */
    RF_SetCR(RF_CR_DEFAULT);              /* 设置信道编码率 */
    RF_SetCRC(RF_CRC_DEFAULT);            /* 设置CRC校验 */
    RF_SetLDR(RF_LDR_DEFAULT);            /* 设置低速率模式 */
    RF_SetPreamLen(RF_PREAMBLE_DEFAULT);  /* 设置前导码长度 */
    RF_SetInvertIQ(RF_IQ_INVERT_DEFAULT); /* 设置IQ不反转 */
    RF_SetRegulatorMode(USE_LDO);         /* 设置芯片为LDO电源模式 */
    RF_SetChipMode(CHIPMODE_MODE0);       /* 设置芯片模式为MODE0 */
}

/**
 * @brief 软件复位RF芯片控制逻辑
 */
void RF_ResetLogic(void)
{
    RF_WriteReg(0x00, 0x80);
    RF_WriteReg(0x00, 0x00);

    (void)RF_ReadReg(0x00); /* 需要空读一次寄存器0x00，才能使复位生效 */ 
}

/**
 * @brief 获取RF芯片的工作状态
 * @return RfOpState_t 当前工作状态
 *         - RF_STATE_SLEEP: 芯片处于睡眠模式
 *         - RF_STATE_STB3:  芯片处于待机模式
 *         - RF_STATE_TX:    芯片处于发射模式
 *         - RF_STATE_RX:    芯片处于接收模式
 */
RfOpState_t RF_GetOperateState(void)
{
    return g_RfOperatetate;
}

/**
 * @brief 设置RF芯片的工作状态
 * @param <RfState> 工作状态
 *         - RF_STATE_SLEEP: 芯片处于睡眠模式
 *         - RF_STATE_STB3: 芯片处于待机模式
 *         - RF_STATE_TX: 芯片处于发射模式
 *         - RF_STATE_RX: 芯片处于接收模式
 */
void RF_SetOperateState(RfOpState_t RfState)
{
    g_RfOperatetate = RfState;
}

/**
 * @brief 设置RF芯片的工作状态
 * @param <RfState>
 *        - RF_STATE_DEEPSLEEP
 *        - RF_STATE_SLEEP
 *        - RF_STATE_STB3
 *        - RF_STATE_TX
 *        - RF_STATE_RX
 */
void RF_SetRfState(uint8_t RfState)
{
    RF_WriteReg(0x02, RfState);
    g_RfOperatetate = (RfOpState_t)RfState;
}

/**
 * @brief 进入深度睡眠模式
 * @note 该函数用于将RF芯片置于深度睡眠模式，关闭天线供电和TCXO电源
 * @note 该函数会将芯片的工作状态设置为MODE_DEEPSLEEP
 * @note 执行该函数后，如需唤醒RF芯片，需要调用RF_Init()函数唤醒芯片
 */
void RF_EnterDeepsleepState(void)
{
    RF_ShutdownAnt();                      /* 关闭天线 */
    RF_WriteReg(0x02, RF_STATE_STB3);      /* 进入STB3状态 */
    RF_DelayUs(150);                       /* 保证实际延时在150us以上 */
    RF_WriteReg(0x02, RF_STATE_STB2);      /* 进入STB2状态 */
    RF_DelayUs(10);                        /* 保证实际延时在10us以上 */
    RF_WriteReg(0x02, RF_STATE_STB1);      /* 进入STB1状态 */
    RF_DelayUs(10);                        /* 保证实际延时在10us以上 */
#if USE_ACTIVE_CRYSTAL == 1                /* 如果使用有源晶振，则需要关闭TCXO电源 */
    RF_TurnoffTcxo();                      /* 关闭TCXO电源 */
#endif
    RF_WriteReg(0x04, 0x06);               /* 关闭LFT */
    RF_DelayUs(10);                        /* 保证实际延时在10us以上 */
    RF_WriteReg(0x02, RF_STATE_SLEEP);     /* 进入SLEEP状态 */
    RF_DelayUs(10);                        /* 保证实际延时在10us以上 */
    RF_WritePageReg(3, 0x06, 0x00);        /* 关闭ISO */
    RF_DelayUs(10);                        /* 保证实际延时在10us以上 */
    RF_WritePageReg(3, 0x26, 0x00);        /* 关闭内部电源 */
    RF_DelayUs(10);                        /* 保证实际延时在10us以上 */
    RF_WriteReg(0x02, RF_STATE_DEEPSLEEP); /* 进入DEEPSLEEP状态 */

    g_RfOperatetate = RF_STATE_DEEPSLEEP;
}

/**
 * @brief 进入睡眠模式
 * @note 该函数用于将RF芯片置于睡眠模式，关闭天线供电和TCXO电源
 * @note 该函数会将芯片的工作状态设置为MODE_SLEEP
 * @note 执行该函数后，如需唤醒RF芯片，需要调用RF_ExitSleepState()函数
 */
void RF_EnterSleepState(void)
{
    RF_ShutdownAnt();                   /* 关闭天线 */
    RF_WriteReg(0x02, RF_STATE_STB3);   /* 进入STB3状态 */
    RF_DelayUs(150);                    /* 保证实际延时在150us以上 */
    RF_WriteReg(0x02, RF_STATE_STB2);   /* 进入STB2状态 */
    RF_DelayUs(10);                     /* 保证实际延时在10us以上 */
    RF_WriteReg(0x02, RF_STATE_STB1);   /* 进入STB1状态 */
    RF_DelayUs(10);                     /* 保证实际延时在10us以上 */
#if USE_ACTIVE_CRYSTAL == 1             /* 如果使用有源晶振，则需要关闭TCXO电源 */
    RF_TurnoffTcxo();                   /* 关闭TCXO电源 */
#endif
    RF_WriteReg(0x04, 0x16);            /* 关闭LFT */
    RF_DelayUs(10);                     /* 保证实际延时在10us以上 */
    RF_WriteReg(0x02, RF_STATE_SLEEP);  /* 进入SLEEP状态 */
    RF_DelayUs(10);                     /* 保证实际延时在10us以上 */
    RF_ResetPageRegBits(3, 0x06, 0x20); /* 关闭ISO */
    RF_DelayUs(10);                     /* 保证实际延时在10us以上 */
    RF_WritePageReg(3, 0x26, 0x00);     /* 关闭内部电源 */

    g_RfOperatetate = RF_STATE_SLEEP;
}

/**
 * @brief 退出睡眠模式
 * @note 该函数用于将RF芯片退出睡眠模式，打开天线供电和TCXO电源
 * @note 该函数会将芯片的工作状态设置为MODE_STDBY
 */
void RF_ExitSleepState(void)
{
    RF_SetPageRegBits(3, 0x06, 0x20); /* 使能ISO */
    RF_DelayUs(10);                   /* 保证实际延时在10us以上 */
    RF_WriteReg(0x02, RF_STATE_STB1); /* 进入STB1状态 */
    RF_DelayUs(10);                   /* 保证实际延时在10us以上 */
#if USE_ACTIVE_CRYSTAL == 1
    RF_WritePageReg(3, 0x26, 0xA0);   /* 使能内核电源，并打开有源晶振通道 */
    RF_DelayUs(100);                  /* 保证实际延时在100us以上 */
    RF_WriteReg(0x04, 0x36);          /* 使能LFT */
    RF_DelayUs(100);                  /* 保证实际延时在100us以上 */
    RF_TurnonTcxo();                  /* 打开TCXO */
#else
    RF_WritePageReg(3, 0x26, 0x20);   /* 使能内核电源 */
    RF_DelayUs(100);                  /* 保证实际延时在100us以上 */
    RF_WriteReg(0x04, 0x36);          /* 使能LFT */
    RF_DelayUs(100);                  /* 保证实际延时在100us以上 */
#endif
    RF_WriteReg(0x02, RF_STATE_STB2); /* 进入STB2状态 */
    RF_DelayMs(1);                    /* 保证实际延时在1ms以上 */
    RF_WriteReg(0x02, RF_STATE_STB3); /* 进入STB3状态 */
    RF_DelayUs(100);                  /* 保证实际延时在100us以上 */

    g_RfOperatetate = RF_STATE_STB3;
}

/**
 * @brief 进入待机模式
 * @note 该函数会将芯片的工作状态设置为MODE_STDBY
 */
void RF_EnterStandbyState(void)
{
    RF_SetRfState(RF_STATE_STB3);
    RF_SetOperateState(RF_STATE_STB3);
}

/**
 * @brief 检查RF芯片是否处于休眠状态
 * @note 该函数用于检查RF芯片是否处于休眠状态，
 * @note 如果处于睡眠状态，则退出睡眠状态进入待机状态
 * @note 该函数会将芯片的工作状态设置为MODE_STDBY
 */
void RF_CheckDeviceReady(void)
{
    if (RF_GetOperateState() == RF_STATE_SLEEP)
    {
        RF_ExitSleepState();
    }
}

/**
 * @brief 设置芯片的供电模式
 * @param <RegulatorMode> 供电模式
 *         - USE_LDO: 使用LDO供电
 *         - USE_DCDC: 使用DCDC供电
 * @note 在发射状态下，RF必须使用LDO电源模式，
 *       其它状态下可任意选择电源供电模式。
 */
void RF_SetRegulatorMode(RfRegulatorMode_t RegulatorMode)
{
    RF_WritePageReg(3, 0x24, (RegulatorMode == USE_DCDC) ? 0x08 : 0x00);
    g_RfCfgParams.RegulatorMode = RegulatorMode;
}

/**
 * @brief 设置RF芯片的频率
 * @param <Frequency> 通信频率(Hz)
 * @note  支持的频率范围为：
 *        低频段：
 *         - 138.33MHz ~ 282.5MHz
 *         - 405.00MHz ~ 565.00MHz
 *        高频段：
 *         - 810.00MHz ~ 1080.00MHz
 */
RF_Err_t RF_SetFreq(uint32_t Frequency)
{
    int i;
    uint32_t Fa, Fb;
    uint32_t Temp = 0;
    uint32_t IntegerPart;
    uint8_t FreqReg[4], Fab[3];
    uint32_t FreqTableNum = (sizeof(g_RfFreqTable) / sizeof(RadioFreqTable_t));
   
    if (Frequency < g_RfFreqTable[0].StartFreq || Frequency > g_RfFreqTable[FreqTableNum - 1].StopFreq)
    {
        return RF_FAIL;
    }
    
    /* 遍历频率表，找到匹配的频率段 */
    for (i = 0; i < FreqTableNum; i++)
    {
        if (Frequency > g_RfFreqTable[i].StartFreq && Frequency <= g_RfFreqTable[i].StopFreq)
        {
            uint8_t LoMux = (g_RfFreqTable[i].LoParam & 0x70) >> 4;
            Temp = Frequency * g_VcoDivTable[LoMux];
            RF_WritePageRegs(0, 0x40, (uint8_t *)&g_RfFreqTable[i].VcoParam, 2);
            RF_WriteReg(0x3D, g_RfFreqTable[i].LoParam);
            break;
        }
    }

    /* No frequency range matched */
    if (i >= FreqTableNum)
    {
        return RF_FAIL;
    }
    
    IntegerPart = Temp / 32000000;
    Fa = IntegerPart - 20;
    Fb = (Temp % 32000000) / 40000;

    FreqReg[0] = (uint8_t)Frequency;
    FreqReg[1] = (uint8_t)(Frequency >> 8);
    FreqReg[2] = (uint8_t)(Frequency >> 16);
    FreqReg[3] = (uint8_t)(Frequency >> 24);
    RF_WritePageRegs(3, 0x09, FreqReg, 4);

    Fab[0] = (uint8_t)(Fa);
    Fab[1] = (uint8_t)(Fb);
    Fab[2] = (uint8_t)((Fb >> 8) & 0x0F);
    RF_WritePageRegs(3, 0x15, Fab, 3);

    g_RfCfgParams.Frequency = Frequency;

    return RF_OK;
}

/**
 * @brief 设置IQ反转
 * @param <NewState> 使能或禁用IQ反转
 *         - true: 使能IQ反转
 *         - false: 禁用IQ反转
 */
void RF_SetInvertIQ(bool NewState)
{
    if (NewState)
    {
        /*
         * BIT6 = 0: invert rx IQ
         * BIT5 = 1: invert tx IQ
         */
        RF_WritePageRegBits(1, 0x0E, 0x01, 0x40 | 0x20);
        g_RfCfgParams.InvertIQ = RF_IQ_INVERTED;
    }
    else
    {
        /*
         * BIT6 = 1: non-invert rx IQ
         * BIT5 = 0: non-invert tx IQ
         */
        RF_WritePageRegBits(1, 0x0E, 0x02, 0x40 | 0x20);
        g_RfCfgParams.InvertIQ = RF_IQ_NORMAL;
    }
}

/**
 * @brief 设置前导码长度
 * @param <PreamLen> 前导码长度值
 *         范围为4 - 65535
 */
void RF_SetPreamLen(uint16_t PreamLen)
{
    uint8_t Temp[2] = {(uint8_t)(PreamLen), (uint8_t)((PreamLen >> 8))};
    RF_WritePageRegs(3, 0x13, Temp, 2);
    g_RfCfgParams.PreambleLen = PreamLen;
}

/**
 * @brief 设置同步字
 * @param <syncWord> 同步字值
 * @note PAN3029/3060支持的同步字大小为1字节
 * @note 同步字用于接收数据包时的同步检测，通常在发送和接收数据包时需要设置相同的同步字
 *       例如，如果发送数据包时设置了同步字为0x12，则在接收数据包时也需要设置同步字为0x12
 */
void RF_SetSyncWord(uint8_t SyncWord)
{
    RF_WritePageReg(3, 0x0F, SyncWord);
    g_RfCfgParams.SyncWord = SyncWord;
}

/**
 * @brief 设置发射功率
 * @param <TxPower> 发射档位，档位范围：1-22
 * @note 功率档位对应的功率值如下表所示：
 *|------|------------------|------------------|------------------|------------------|
 *| 档位 | 410MHz 功率(dBm) | 430MHz 功率(dBm)  | 450MHz 功率(dBm) | 460MHz 功率(dBm) |
 *|------|------------------|------------------|------------------|------------------|
 *|  1   |       -18.7      |      -18.2       |       -18.8      |       -19.7      |
 *|  2   |       -8.3       |      -7.9        |       -8.6       |       -9.5       |
 *|  3   |        1.6       |       1.7        |        0.9       |       -0.1       |
 *|  4   |        3.9       |       4.5        |        4.1       |        3.3       |
 *|  5   |        4.9       |       5.3        |        4.8       |        3.9       |
 *|  6   |        5.4       |       5.5        |        4.8       |        3.8       |
 *|  7   |        7.1       |       7.6        |        7.3       |        6.8       |
 *|  8   |        7.7       |       8.2        |        8.0       |        7.5       |
 *|  9   |        8.6       |       9.2        |        8.8       |        8.1       |
 *|  10  |        9.0       |       9.3        |        9.1       |        8.9       |
 *|  11  |        10.3      |       10.7       |        10.6      |        10.3      |
 *|  12  |        10.8      |       11.0       |        10.8      |        10.6      |
 *|  13  |        11.6      |       11.8       |        11.6      |        11.5      |
 *|  14  |        12.8      |       13.2       |        13.1      |        12.9      |
 *|  15  |        13.8      |       14.3       |        14.2      |        14.0      |
 *|  16  |        14.9      |       15.4       |        15.4      |        15.0      |
 *|  17  |        15.4      |       16.0       |        15.8      |        15.3      |
 *|  18  |        16.1      |       16.6       |        16.5      |        16.3      |
 *|  19  |        16.5      |       17.0       |        16.9      |        16.7      |
 *|  20  |        17.4      |       17.9       |        17.8      |        17.6      |
 *|  21  |        18.2      |       18.3       |        18.0      |        17.7      |
 *|  22  |        19.4      |       19.4       |        19.1      |        18.7      |
 *|------|------------------|------------------|------------------|------------------|
 */
void RF_SetTxPower(uint8_t TxPower)
{
    int Index;
    uint8_t Temp1, Temp2;
    static bool PaBiasReadFlag = false; // for read efuse only once
    static uint8_t PaBiasVal = 0;
    
    TxPower = (TxPower > RF_MAX_RAMP ? RF_MAX_RAMP : TxPower);
    TxPower = (TxPower < RF_MIN_RAMP ? RF_MIN_RAMP : TxPower);
    
    Index = TxPower - 1;

    /* Modulate wave ramp mode */
    RF_WritePageReg(3, 0x22, g_RfPowerRampCfg[Index].Ldo & 0x01);
    RF_WritePageReg(0, 0x1E, g_RfPowerRampCfg[Index].Ramp);
    RF_WritePageReg(0, 0x4B, g_RfPowerRampCfg[Index].Ldo >> 4);
    
    if (g_RfPowerRampCfg[Index].PAbias != 0x70)
    {
        RF_SetPageRegBits(0, 0x46, 0x04); // page0, reg0x46, bit2=1
    }
    else
    {
        RF_ResetPageRegBits(0, 0x46, 0x04); // page0, reg0x46, bit2=0
    }
    
    if(!PaBiasReadFlag)
    {
        PaBiasReadFlag = true;

        RF_ResetPageRegBits(2, 0x3E, 0x08); /* RF unlock info */
        PaBiasVal = RF_ReadInfoByte(0x3B, 0x5AA5, 0x20);
        RF_SetPageRegBits(2, 0x3E, 0x08);  /* RF lock info */

        if (PaBiasVal == 0)
        {
            PaBiasVal = 8;
        }
    }

    Temp1 = PaBiasVal - (g_RfPowerRampCfg[Index].PAbias & 0x0F);
    Temp2 = (g_RfPowerRampCfg[Index].PAbias & 0xF0) | Temp1;
    RF_WritePageReg(0, 0x45, Temp2);

    g_RfCfgParams.TxPower = TxPower; /* save current TxPower value */
}

/**
 * @brief 设置调制带宽
 * @param <BandWidth> 调制带宽值
 *         - RF_BW_062K / RF_BW_125K / RF_BW_250K / RF_BW_500K
 * @note 调制带宽越大，数据速率越高，但传输距离越短
 * @note PAN3029芯片的调制带宽范围为RF_BW_062K - RF_BW_500K
 * @note PAN3060芯片的调制带宽范围为RF_BW_125K - RF_BW_500K
 */
void RF_SetBW(uint8_t BandWidth)
{
    /* Page 3, Reg 0x0D, Bit[7:4] = BandWidth */
    RF_WritePageRegBits(3, 0x0D, BandWidth, 0xF0);

    if (BandWidth != RF_BW_500K)
    {
        RF_SetPageRegBits(2, 0x3F, 0x02);
    }
    else
    {
        RF_ResetPageRegBits(2, 0x3F, 0x02);
    }

    g_RfCfgParams.Bandwidth = (RfBandwidths_t)BandWidth; // save current BW value
}

/**
 * @brief 设置扩频因子
 * @param <SpreadFactor> 扩频因子值
 *         - RF_SF5 / RF_SF6 / RF_SF7 / RF_SF8 / RF_SF9 / RF_SF10 / RF_SF11 / RF_SF12
 * @note 扩频因子越大，传输距离越远，但数据速率越低
 * @note PAN3029芯片的扩频因子范围为RF_SF5 - RF_SF12
 * @note PAN3060芯片的扩频因子范围为RF_SF5 - RF_SF9
 */
void RF_SetSF(uint8_t SpreadFactor)
{
    /* Page 3, Reg 0x0E, Bit[7:4] = SpreadFactor */
    RF_WritePageRegBits(3, 0x0E, SpreadFactor, 0xF0);

    g_RfCfgParams.SpreadingFactor = (RfSpreadFactor_t)SpreadFactor; // save current SF value
}

/**
 * @brief 设置信道编码率
 * @param <CodingRate> 信道编码率值
 *         - RF_CR_4_5 / RF_CR_4_6 / RF_CR_4_7 / RF_CR_4_8
 */
void RF_SetCR(uint8_t CodingRate)
{
    /* Page 3, Reg 0x0D, Bit[3:1] = CodingRate */
    RF_WritePageRegBits(3, 0x0D, CodingRate, 0x0E);

    g_RfCfgParams.CodingRate = (RfCodingRates_t)CodingRate; // save current CR value
}

/**
 * @brief 设置CRC校验
 * @param <CrcMode> 使能或禁用CRC校验
 *        - RF_CRC_ON: 使能CRC校验
 *        - RF_CRC_OFF: 禁用CRC校验
 */
void RF_SetCRC(uint8_t CrcMode)
{
    /* Page 3, Reg 0x0D, Bit[0] = CRC */
    RF_WritePageRegBits(3, 0x0E, CrcMode, 0x08);

    g_RfCfgParams.CrcMode = (RfCrcModes_t)CrcMode; // save current CRC value
}

/**
 * @brief 设置低速率模式
 * @param <LdrMode> 低速率模式值
 *         - RF_LDR_ON: 使能低速率模式
 *         - RF_LDR_OFF：禁用低速率模式
 */
void RF_SetLDR(uint8_t LdrMode)
{
    /* Page 3, Reg 0x12, Bit[3] = LDR */
    RF_WritePageRegBits(3, 0x12, LdrMode, 0x08);

    g_RfCfgParams.LowDatarateOptimize = LdrMode; // save current LDR value
}

/**
 * @brief Set modem mode
 * @param <modem_mode>
 *         - MODEM_MODE_NORMAL
 *         - MODEM_MODE_MULTI_SECTOR
 * @note This function should be called after RF_SetSF(uint8_t SpreadFactor)
 */
void RF_SetModemMode(uint8_t ModemMode)
{
    if (ModemMode == MODEM_MODE_NORMAL)
    {
        RF_WritePageReg(1, 0x0B, 0x08);
    }
    else if (ModemMode == MODEM_MODE_MULTI_SECTOR)
    {
        RF_WritePageReg(1, 0x0B, 0x18);
        if( g_RfCfgParams.SpreadingFactor <= RF_SF6 )
        {
            RF_WritePageReg(1, 0x2F, 0x74);
            RF_WritePageReg(1, 0x30, 0x01);
        }
        else
        {
            RF_WritePageReg(1, 0x2F, 0x54);
            RF_WritePageReg(1, 0x30, 0x40);
        }
    }
}

/**
 * @brief 设置发送模式
 * @param Buffer 要发送的数据缓冲区
 * @param Size 要发送的数据字节数
 * @note 保证在调用此函数前，RF已经处于待机(STB3)模式
 * @note 此函数为单次发送模式，发送完成后会自动进入待机(STB3)模式
 * @note 发送完成后会触发TX_DONE中断
 */
void RF_SetTx(uint8_t *Buffer, uint8_t Size)
{
    RF_TxSinglePkt(Buffer, Size);
    g_RfOperatetate = RF_STATE_TX;
}

/**
 * @brief 设置接收模式
 * @param TimeoutMs 接收超时时间，单位为毫秒
 *        - 0: 连续接收模式
 *        - >0: 单次接收模式，超时后报超时中断并自动进入待机(STB3)模式
 * @note 保证在调用此函数前，RF已经处于待机(STB3)模式
 */
void RF_SetRx(uint32_t TimeoutMs)
{
    if (TimeoutMs == 0)
    {
        RF_EnterContinousRxState();
    }
    else
    {
        RF_EnterSingleRxWithTimeout(TimeoutMs);
    }

    g_RfOperatetate = RF_STATE_RX;
}

/**
 * @brief 启动CAD检测
 * @param <Threshold>
           - RF_CAD_THRESHOLD_0A
           - RF_CAD_THRESHOLD_10
           - RF_CAD_THRESHOLD_15
           - RF_CAD_THRESHOLD_20
          <Chirps>
           - RF_CAD_01_SYMBOL
           - RF_CAD_02_SYMBOL
           - RF_CAD_03_SYMBOL
           - RF_CAD_04_SYMBOL
 */
void RF_StartCad(uint8_t Threshold, uint8_t Chirps)
{
    /* Configure GPIO11 as output for CAD indication */
    RF_ConfigGpio(MODULE_GPIO_CAD_IRQ, GPIO_MODE_OUTPUT);

    /* [Page0][Reg0x5E][Bit6] = 0, enable GPIO11 CAD indication */
    RF_ResetPageRegBits(0, 0x5E, 0x40);

    RF_WritePageReg(1, 0x0F, Threshold);            /* [Page1][Reg0x0F] = Threshold */
    RF_WritePageRegBits(1, 0x25, Chirps - 1, 0x03); /* [Page1][Reg0x25][Bit[1:0]] = Chirps - 1 */
    RF_WritePageReg(1, 0x35, 0xFE);                 /* [Page1][Reg0x35] payload cad config */
    RF_EnterContinousRxState();                     /* Enter continous RX state */
}

/**
 * @brief 设置CAD检测阈值
 * @param <Threshold> CAD检测阈值
 *         - RF_CAD_THRESHOLD_0A
 *         - RF_CAD_THRESHOLD_10
 *         - RF_CAD_THRESHOLD_15
 *         - RF_CAD_THRESHOLD_20
 */
void RF_SetCadThreshold(uint8_t Threshold)
{
    RF_WritePageReg(1, 0x0F, Threshold);
}

/**
 * @brief 设置CAD检测符号数
 * @param <Chirps> CAD检测符号数
 *         - RF_CAD_01_SYMBOL
 *         - RF_CAD_02_SYMBOL
 *         - RF_CAD_03_SYMBOL
 *         - RF_CAD_04_SYMBOL
 */
void RF_SetCadChirps(uint8_t Chirps)
{
    RF_WritePageRegBits(1, 0x25, Chirps - 1, 0x03); /* [Page1][Reg0x25][Bit[1:0]] = Chirps */
}

/**
 * @brief 停止CAD检测
*/
void RF_StopCad(void)
{
    RF_SetPageRegBits(0, 0x5E, 0x40); /* [Page0][Reg0x5E][Bit6] = 1, disable GPIO11 CAD indication */
    RF_WritePageReg(1, 0x0F, 0x0A);   /* Reset CAD threshold */
    RF_WritePageReg(1, 0x35, 0xF4);   /* Reset payload cad config */
    RF_SetRfState(RF_STATE_STB3);     /* Enter standby state */
    RF_ResetLogic();                  /* Soft reset the RF chip, clear cad state */
    g_RfOperatetate = RF_STATE_STB3;
}

/**
 * @brief 设置发射模式
 * @param <TxMode>
 *         - RF_TX_SINGLE：单次发送模式
 *         - RF_TX_CONTINOUS：连续发送模式
 * @note 该函数仅用于设置发射模式，并不改变芯片的工作状态
 */
void RF_SetTxMode(uint8_t TxMode)
{
    RF_WritePageRegBits(3, 0x06, TxMode, 0x04); /* [Page3][Reg0x06][Bit[2]] = TxMode */
}

/**
 * @brief 发送单个数据包
 * @param <Buffer> 要发送的数据缓冲区
 * @param <Size> 要发送的数据字节数
 * @note 发送完成后会触发TX_DONE中断
 * @note 在发送完成中断中，需调用RF_TurnoffPA()函数来关闭芯片内部和外部PA
 */
void RF_TxSinglePkt(uint8_t *Buffer, uint8_t Size)
{
    RF_WriteReg(0x02, RF_STATE_STB3); /* 进入待机状态 */
    RF_SetTxMode(RF_TX_MODE_SINGLE);  /* 设置单次发送模式 */
    RF_WritePageReg(1, 0x0C, Size);   /* 设置发送数据长度 */
    RF_TurnonPA();                    /* 发射数据前须打开PA */
    RF_SetRfState(RF_STATE_TX);       /* 设置芯片为发射状态 */
    RF_WriteRegs(0x01, Buffer, Size); /* 写入数据到FIFO, 写完数据后，芯片开始发射数据, 其中0x01为FIFO寄存器地址 */
}

/**
 * @brief 设置接收模式
 * @param <RxMode>
 *         - RF_RX_SINGLE: 单次接收模式,接收到一包数据后自动进入待机模式
 *         - RF_RX_SINGLE_TIMEOUT：单次带超时接收模式,超时后自动进入待机模式
 *         - RF_RX_CONTINOUS：连续接收模式,接收到一包数据后继续接收
 * @note 该函数仅用于设置接收模式，并不改变芯片的工作状态
 */
void RF_SetRxMode(uint8_t RxMode)
{
    RF_WritePageRegBits(3, 0x06, RxMode, 0x03); /* [Page3][Reg0x06][Bit[1:0]] = RxMode */
}

/**
 * @brief 让芯片进入连续接收状态
 * @note 调用此函数后，芯片会进入连续接收状态
 * @note 该函数会将芯片的工作状态设置为MODE_RX
 */
void RF_EnterContinousRxState(void)
{
    RF_SetRfState(RF_STATE_STB3);       /* 进入待机状态 */
    RF_TurnonRxAnt();                   /* 打开接收天线开关 */
    RF_TurnoffLdoPA();                  /* 关闭内部PA */
    RF_SetRxMode(RF_RX_MODE_CONTINOUS); /* 设置接收模式为连续接收 */
    RF_SetRfState(RF_STATE_RX);         /* 进入接收状态 */
}

/**
 * @brief 设置接收超时时间
 * @param <TimeoutMs> 超时时间，单位为ms
 *         超时时间范围：0~65535ms
 * @note 该函数仅用于设置接收超时时间，并不改变芯片的工作状态
 */
void RF_SetRxTimeout(uint16_t TimeoutMs)
{
    uint8_t Temp[2] = {(uint8_t)TimeoutMs, (uint8_t)(TimeoutMs >> 8)};
    RF_WritePageRegs(3, 0x07, Temp, 2);
}

/**
 * @brief 让芯片进入带超时的单次接收状态
 * @param <TimeoutMs> 超时时间，单位为ms
 *            超时时间范围：1~65535ms
 * @note 调用此函数后，芯片会进入带超时的单次接收状态
 * @note 该函数会将芯片的工作状态设置为MODE_RX
 */
void RF_EnterSingleRxWithTimeout(uint16_t TimeoutMs)
{
    RF_SetRfState(RF_STATE_STB3);            /* 进入待机状态 */
    RF_TurnonRxAnt();                        /* 打开接收天线开关 */
    RF_TurnoffLdoPA();                       /* 关闭内部PA */
    RF_SetRxTimeout(TimeoutMs);              /* 设置接收超时时间 */
    RF_SetRxMode(RF_RX_MODE_SINGLE_TIMEOUT); /* 设置接收模式为单次接收模式 */
    RF_SetRfState(RF_STATE_RX);              /* 进入接收状态 */
}

/**
 * @brief 获取接收数据长度
 * @return 接收数据长度
 * @note 此函数须在接收中断被清除前调用，因为清除接收中断会将接收长度寄存器清零
 */
uint8_t RF_GetRxPayloadLen(void)
{
    return RF_ReadPageReg(1, 0x7D);
}

/**
 * @brief 函数用于获取接收数据长度及内容
 * @param *Buffer 待接收数据区指针地址
 * @return 接收到的数据长度
 * @note 此函数须在接收中断被清除前调用，因为清除接收中断会将接收长度寄存器清零
 */
uint8_t RF_GetRecvPayload(uint8_t *Buffer)
{
    uint8_t Size;
    Size = RF_GetRxPayloadLen();     /* 获取接收数据长度 */
    RF_ReadRegs(0x01, Buffer, Size); /* 从FIFO中读取接收到的数据, 其中0x01为FIFO寄存器地址 */
    return Size;
}

/**
 * @brief 获取接收数据包的RSSI值
 * @return RSSI值
 * @note RSSI值范围：-125~-10，单位dBm，RSSI值小于等于灵敏度值
 * @note 此函数须在接收中断被清除前调用，因为清除接收中断会将信号强度寄存器清零
 */
int8_t RF_GetPktRssi(void)
{
    return RF_ReadPageReg(1, 0x7F);
}

/**
 * @brief 获取实时RSSI值
 * @return RSSI值
 * @note RSSI值范围：-125~-10，单位dBm
 * @note 调用此函数前需须保证RF处于接收状态
 */
int8_t RF_GetRealTimeRssi(void)
{
    /* 将寄存器0x06的Bit[2]清零再置为1，即可更新[Page1][Reg0x7E]的值 */
    RF_ResetPageRegBits(2, 0x06, 0x04);
    RF_SetPageRegBits(2, 0x06, 0x04);

    return (int8_t)RF_ReadPageReg(1, 0x7E);
}

/**
 * @brief 获取接收数据包的SNR值
 * @return SNR值
 * @note 此函数须在接收中断被清除前调用，因为清除接收中断会将SNR寄存器清零
 * @note SNR值范围：-20~10，单位dB
 */
int32_t RF_GetPktSnr(void)
{
    int32_t PktSnr = 0, SnrVal;
    uint8_t i, Temp[6];
    uint32_t NoiseStrength;
    uint32_t SingalStrength;

    RF_ReadPageRegs(2, 0x71, &Temp[0], 3); // Noise strength
    RF_ReadPageRegs(1, 0x74, &Temp[3], 3); // Singal strength

    SingalStrength = (((uint32_t)Temp[5] << 16) | ((uint32_t)Temp[4] << 8) | Temp[3]);
    NoiseStrength = (((uint32_t)Temp[2] << 16) | ((uint32_t)Temp[1] << 8) | Temp[0]);

    if (NoiseStrength == 0)
    {
        NoiseStrength = 1;
    }

    if(g_RfCfgParams.SpreadingFactor <= 9)
    {
        SnrVal = (SingalStrength << (9 - g_RfCfgParams.SpreadingFactor)) / NoiseStrength;
    }
    else
    {
        SnrVal = (SingalStrength >> (g_RfCfgParams.SpreadingFactor - 9)) / NoiseStrength;
    }

    for (i = 0; i < 31; i++)
    {
        if (SnrVal <= g_SnrLog10Talbe[i])
        {
            PktSnr = (int32_t)i - (int32_t)20;
            break;
        }
    }

    return PktSnr;
}

/**
 * @brief 获取中断标志位
 * @return IRQ标志位
 *         - 0x00: 无中断
 *         - 0x01: RF_IRQ_TX_DONE
 *         - 0x02: RF_IRQ_RX_TIMEOUT
 *         - 0x04: RF_IRQ_CRC_ERR
 *         - 0x08: RF_IRQ_RX_DONE
 *         - 0x40: RF_IRQ_MAPM_DONE
 */
uint8_t RF_GetIRQFlag(void)
{
    return (RF_ReadPageReg(0, 0x6C) & 0x7F);
}

/**
 * @brief 清中断标志位
 * @param <IRQFlag> 中断标志位
 */
void RF_ClrIRQFlag(uint8_t IRQFlag)
{
    RF_WritePageReg(0, 0x6C, IRQFlag);
}

/**
 * * @brief 获取当前的频率设置
 */
uint32_t RF_GetFreq(void)
{
    return g_RfCfgParams.Frequency;
}

/**
 * * @brief 获取当前的IQ反转值
 */
RfIQModes_t RF_GetInvertIQ(void)
{
    return g_RfCfgParams.InvertIQ;
}

/**
 * @brief 获取当前的前导码长度设置
 */
uint16_t RF_GetPreamLen(void)
{
    return g_RfCfgParams.PreambleLen;
}

/**
 * @brief 获取当前的发射功率设置
 */
uint8_t RF_GetTxPower(void)
{
    return g_RfCfgParams.TxPower;
}

/**
 * @brief 获取当前的调制带宽设置
 */
uint8_t RF_GetBandWidth(void)
{
    return g_RfCfgParams.Bandwidth;
}

/**
 * @brief 获取当前的扩频因子设置
 */
uint8_t RF_GetSF(void)
{
    return g_RfCfgParams.SpreadingFactor;
}

/**
 * @brief 获取当前的CRC校验设置
 */
uint8_t RF_GetCRC(void)
{
    return g_RfCfgParams.CrcMode;
}

/**
 * @brief 获取当前的编码率设置
 */
uint8_t RF_GetCR(void)
{
    return g_RfCfgParams.CodingRate;
}

/**
 * @brief 获取当前的同步字设置
 */
uint8_t RF_GetSyncWord(void)
{
    return g_RfCfgParams.SyncWord;
}

/**
 * @brief 获取当前的发射模式设置
 */
uint8_t RF_GetLDR(void)
{
    return g_RfCfgParams.LowDatarateOptimize;
}

/**
 * @brief 获取单个符号的时间
 * @param <bw> 带宽
 *           - RF_BW_062K / RF_BW_125K / RF_BW_250K / RF_BW_500K
 * @param <sf> 扩频因子
 *           - RF_SF5 / RF_SF6 / RF_SF7 / RF_SF8 / RF_SF9 / RF_SF10 / RF_SF11 / RF_SF12
 * @return 单个符号的时间，单位为us
 * @note 该函数用于计算单个符号的时间
 */
uint32_t RF_GetOneSymbolTime(uint8_t bw, uint8_t sf)
{
    const uint32_t BwTable[4] = {62500, 125000, 250000, 500000};

    if(bw < RF_BW_062K || bw > RF_BW_500K)
    {
        return 0;
    }

    return (1000000 * (1 << sf) / BwTable[bw - RF_BW_062K]);
}

/**
 * @brief 计算发送数据包的时间
 * @param <Size> 发送数据包的大小，单位为字节
 * @return 发送数据包的时间，单位为ms
 */
uint32_t RF_GetTxTimeMs(uint8_t Size)
{
    uint8_t sf, cr, bw, ldr;
    uint16_t PreambleLen; /* Preamble length */
    float SymbolTime;     /* Symbol time:ms */
    float PreambleTime;   /* Preamble time:ms */
    float PayloadTime;    /* Payload time:ms */
    float TotalTime;      /* Total time:ms */
    const float BwTable[4] = {62.5, 125, 250, 500};

    sf = RF_GetSF();
    cr = RF_GetCR();
    bw = RF_GetBandWidth();
    ldr = RF_GetLDR();
    PreambleLen = RF_GetPreamLen();
    
    SymbolTime = (float)(1 << sf) / BwTable[bw - RF_BW_062K]; /* Symbol time: ms */

    if (sf < 7)
    {
        PreambleTime = (PreambleLen + 6.25f) * SymbolTime;
        PayloadTime = ceil((float)(Size * 8 - sf * 4 + 36) / ((sf - ldr * 2) * 4));
    }
    else
    {
        PreambleTime = (PreambleLen + 4.25f) * SymbolTime;
        PayloadTime = ceil((float)(Size * 8 - sf * 4 + 44) / ((sf - ldr * 2) * 4));
    }

    TotalTime = PreambleTime + (PayloadTime * (cr + 4) + 8) * SymbolTime;

    if(TotalTime < 1)
    {
        TotalTime = 1; /* 小于1ms时，按1ms处理 */
    }

    return (uint32_t)TotalTime;
}

/**
 * @brief 启动mapm模式
 */
void RF_EnableMapm(void)
{
    RF_SetPageRegBits(1, 0x38, 0x01);      /* Enable mapm mode */
    RF_WritePageRegBits(0, 0x58, 0, 0x40); /* Enable mapm interrupt */
}

/**
 * @brief 关闭mapm模式
 */
void RF_DisableMapm(void)
{
    RF_ResetPageRegBits(1, 0x38, 0x01);    /* Disable mapm mode */
    RF_WritePageRegBits(0, 0x58, 1, 0x40); /* Disable mapm interrupt */
}

/**
 * @brief 配置mapm相关参数
 * @param <pMapmCfg>
 *         fn: mapm中的field个数，总共发送fn*(2^fnm)个field（通常fnm=0）
 *         fnm: 同一个field重复发送的次数配置(默认fnm=0)
 *              - fnm=0时，表示同一个field发送1次，总共发送fn*1个fields
 *              - fnm=1时，表示同一个field发送2次，总共发送fn*2个fields
 *              - fnm=2时，表示同一个field发送4次，总共发送fn*4个fields
 *              - fnm=3时，表示同一个field发送8次，总共发送fn*8个fields
 *         gfs: 每个field中最后一个group的载荷功能选择
 *              - gfs=0时，表示最后一个group的载荷功能为地址
 *              - gfs=1时，表示最后一个group的载荷功能为剩余field的个数(包含当前field)
 *         gn: 一个field中group的个数
 *         pg1: field中第一个group的前导码个数,范围为8~255
 *              - pg1=8时，表示第一个group中有8个前导码
 *              - pg1=200时，表示第一个group中有200个前导码
 *         pgn: field中除第一个group外的其它group的前导码个数,范围为0~255
 *              - pgn=8时，表示第一个group中有8个前导码
 *              - pgn=200时，表示第一个group中有200个前导码
 *         pn: 数据包的前导码个数,范围为1~65535
 * @note group中的地址或计数值占用2个chirp.
 */
void RF_ConfigMapm(RF_MapmCfg_t *pMapmCfg)
{
    uint8_t reg_fn, fn_h, fn_l;

    fn_h = pMapmCfg->fn / 15 + 1;
    fn_l = pMapmCfg->fn % 15 + 1;
    reg_fn = (fn_h << 4) + fn_l;
    RF_WritePageReg(1, 0x3D, reg_fn);                            /* set the number of fields */
    RF_WritePageRegBits(1, 0x37, pMapmCfg->fnm, 0x80 | 0x40);    /* set the unit code word of the field counter represents several fields */
    RF_WritePageRegBits(1, 0x38, pMapmCfg->gfs, 0x02);           /* set the last group function selection */
    RF_WritePageRegBits(1, 0x38, pMapmCfg->gn - 1, 0x08 | 0x04); /* set the number of groups in Field */
    RF_WritePageReg(1, 0x3B, pMapmCfg->pg1);                     /* set the number of Preambles in first groups */

    /* set the number of preambles for groups other than the first group */
    RF_WritePageReg(1, 0x3C, pMapmCfg->pgn);

    /* set the number of preamble between the last group and the sync word */
    RF_WritePageRegBits(1, 0x39, (uint8_t)(pMapmCfg->pn >> 8), 0x0F);
    RF_WritePageReg(1, 0x3A, (uint8_t)(pMapmCfg->pn));
}

/**
 * @brief 设置mapm模式下的组地址
 * @param <MapmAddr> mapm组地址
 *        <AddrWidth> 地址宽度,范围为1~4
 * @note 接收端的MapmAddr[0]须与发送端的MapmAddr[0]一致，
 *       否则接收端不会触发mapm中断。
 * @note Mapm地址寄存器说明：
 *       [Page1][Reg0x3E]为MapmAddr[0]
 *       [Page1][Reg0x3F]为MapmAddr[1]
 *       [Page1][Reg0x40]为MapmAddr[2]
 *       [Page1][Reg0x41]为MapmAddr[3]
 */
void RF_SetMapmAddr(uint8_t *MapmAddr, uint8_t AddrWidth)
{
    RF_WritePageRegs(1, 0x3E, MapmAddr, AddrWidth);
}

/**
 * @brief 计算1个field花费的时间(ms)
 * @param <pMapmCfg> mapm配置参数
 *        <SymbolTime> 单个symbol(chirp)时间
 * @note Group1中chirp的数量为(pg1 + 2)，其中pg1为Group1中前导码的个数，2为Group1中地址占用的chirp数。
 * @note 其它Group中chirp的数量为(pgn + 2)*(gn-1)，其中pgn为其它单个Group中前导码的个数，
 *       2为其它单个Group中地址(或计数值)占用的chirp数，(gn-1)为除去Group1后剩余的Group数。
 */
uint32_t RF_GetMapmOneFieldTime(RF_MapmCfg_t *pMapmCfg, uint32_t SymbolTime)
{
    uint8_t pgn = pMapmCfg->pgn;
    uint8_t pg1 = pMapmCfg->pg1;
    uint8_t gn = pMapmCfg->gn;

    uint16_t ChirpNumInOneField = (pg1 + 2) + (pgn + 2) * (gn - 1);
    return ChirpNumInOneField * SymbolTime / 1000;
}

/**
 * @brief 获取mapm模式下的剩余Mapm时间
 * @param <pMapmCfg> mapm配置参数
 *        <SymbolTime> 单个symbol(chirp)时间
 * @return 剩余Mapm时间，单位为ms
 * @note 剩余Mapm时间为从当前时刻到剩余field发送完成的时间
 * @note 剩余filed计数包括当前filed在内
 */
uint32_t RF_GetLeftMapmTime(RF_MapmCfg_t *pMapmCfg, uint32_t SymbolTime)
{
    uint8_t fnm, gn, pgn, pg1, fn;
    uint16_t ChirpNumInOneField;
    uint16_t NumberOfLeftChirps;
    uint32_t LeftMapmTime;

    pgn = pMapmCfg->pgn;
    pg1 = pMapmCfg->pg1;
    gn = pMapmCfg->gn;
    fnm = pMapmCfg->fnm;
    fn = pMapmCfg->fn;
    
    /**
     * @brief 计算1个field中chirp的数量
     * @note Group1中chirp的数量为(pg1 + 2)，其中pg1为Group1中前导码的个数，2为Group1中地址占用的chirp数。
     * @note 其它Group中chirp的数量为(pgn + 2)*(gn-1)，其中pgn为其它单个Group中前导码的个数，
     *       (gn-1)为除去Group1后剩余的Group数，2为其它单个Group中地址(或计数值)占用的chirp数。
     */
    ChirpNumInOneField = (pg1 + 2) + (pgn + 2) * (gn - 1);
    
    /**
     * @brief 计算剩余chirp的数量
     * @note fn为mapm中field的数量,如果fnm > 0时，实际发送的field的数量为fn*(2^fnm)。
     * @note pn为field与sync word之间的前导码个数，空中对应有pn个chirp。
     * @note 减掉1个field的chirp数是为了减去自身filed花费的时间。
     */
    NumberOfLeftChirps = (1 << fnm) * fn * ChirpNumInOneField - ChirpNumInOneField;
    
    /* 剩余时间为剩余chirp数乘以单个chirp的时间 */
    LeftMapmTime = SymbolTime * NumberOfLeftChirps;

    return LeftMapmTime / 1000; /* 微秒转毫秒 */
}

/**
 * @brief 开始发送连续载波
 * @note 调用此函数前须设置好发射功率和频率
 * @note 调用此函数后，芯片会一直处于发射状态，直到调用RF_StopTxContinuousWave()函数停止发射
 */
void RF_StartTxContinuousWave(void)
{
    RF_WriteReg(0x02, RF_STATE_STB3);   /* 设置芯片为空闲状态 */
    RF_WritePageReg(0, 0x58, 0x00);     /* 关闭所有RF中断 */
    RF_SetTxMode(RF_TX_MODE_CONTINOUS); /* 设置连续发送模式 */
    RF_WritePageReg(1, 0x0C, 1);        /* 设置发送数据长度为1字节 */
    RF_TurnonPA();                      /* 发射数据前须打开PA */
    RF_WriteReg(0x02, RF_STATE_TX);     /* 设置芯片为发射状态 */
    RF_WriteReg(0x01, 0xFF);            /* 其中0x01为FIFO寄存器地址, 写完数据后在CS上升沿开始发送载波 */
    g_RfOperatetate = RF_STATE_TX;
}

/**
 * @brief 停止发送连续载波
 * @note 调用此函数后，芯片会停止发射，并进入待机状态
 */
void RF_StopTxContinuousWave(void)
{
    RF_WriteReg(0x02, RF_STATE_STB3); /* 设置芯片为空闲状态 */
    RF_TurnoffPA();                   /* 发送完成后须关闭PA */
    RF_WritePageReg(0, 0x58, 0x0F);   /* 恢复RF默认中断 */
    g_RfOperatetate = RF_STATE_STB3;
}

/**
 * @brief 处理RF中断事件
 * @note 此函数可以放在中断服务函数中调用；
 *       也可以用于轮询方式调用此函数来处理RF中断事件。
 */
void RF_IRQ_Process(void)
{
    if (CHECK_RF_IRQ()) /* 检测到RF中断，高电平表示有中断 */
    {
        uint8_t IRQFlag;

        IRQFlag = RF_GetIRQFlag();    /* 获取中断标志位 */
        if (IRQFlag & RF_IRQ_TX_DONE) /* 发送完成中断 */
        {
            RF_TurnoffPA();                /* 发送完成后须关闭PA */
            RF_ClrIRQFlag(RF_IRQ_TX_DONE); /* 清除发送完成中断标志位 */
            IRQFlag &= ~RF_IRQ_TX_DONE;
        }
        if (IRQFlag & RF_IRQ_RX_DONE) /* 接收完成中断 */
        {
            g_RfRxPkt.Snr = RF_GetPktSnr();   /* 获取接收数据包的SNR值 */
            g_RfRxPkt.Rssi = RF_GetPktRssi(); /* 获取接收数据包的RSSI值 */
            
            /* 获取接收数据和长度 */
            g_RfRxPkt.RxLen = RF_GetRecvPayload((uint8_t *)g_RfRxPkt.RxBuf);

            RF_ClrIRQFlag(RF_IRQ_RX_DONE); /* 清除接收完成中断标志位 */
            IRQFlag &= ~RF_IRQ_RX_DONE;
        }
        if (IRQFlag & RF_IRQ_MAPM_DONE) /* mapm接收完成中断 */
        {
            uint8_t MapmAddr = RF_ReadPageReg(0, 0x6E);
            g_RfRxPkt.MapmRxBuf[g_RfRxPkt.MapmRxIndex++] = MapmAddr;

            RF_ClrIRQFlag(RF_IRQ_MAPM_DONE); /* 清除mapm接收完成中断标志位 */
            IRQFlag &= ~RF_IRQ_MAPM_DONE;
        }
        if (IRQFlag & RF_IRQ_CRC_ERR) /* CRC错误中断 */
        {
            RF_ClrIRQFlag(RF_IRQ_CRC_ERR); /* 清除CRC错误中断标志位 */
            IRQFlag &= ~RF_IRQ_CRC_ERR;
        }
        if (IRQFlag & RF_IRQ_RX_TIMEOUT) /* 接收超时中断 */
        {
            /* rf_refresh(); */
            IRQFlag &= ~RF_IRQ_RX_TIMEOUT;
            RF_ClrIRQFlag(RF_IRQ_RX_TIMEOUT); /* 清除接收超时中断标志位 */
        }

        if (IRQFlag)
        {
            RF_ClrIRQFlag(IRQFlag); /* 清除未处理的中断标志位 */
        }
    }
}
