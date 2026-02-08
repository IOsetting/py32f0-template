/**
 * @file      pan_rf.h
 * @brief     PAN3029/PAN3060 driver implementation
 * @version   V1.0.1
 * @date      2025-08-18
 * @copyright Panchip Microelectronics Co., Ltd. All rights reserved.
 * @code
 *              ____              ____ _     _
 *             |  _ \ __ _ _ __  / ___| |__ (_)_ __
 *             | |_) / _` | '_ \| |   | '_ \| | '_ \
 *             |  __/ (_| | | | | |___| | | | | |_) |
 *             |_|   \__,_|_| |_|\____|_| |_|_| .__/
 *                                            |_|
 *              (C)2009-2025 PanChip
 * @endcode
 * @author    PanChip
 * @addtogroup ChirpIOT
 * @{
 * @defgroup  PAN3029/3060 Radio Driver API
 * @{
 */
#ifndef __PAN_RF_H__
#define __PAN_RF_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"


/** TODO: 根据您的硬件做调整：
 * @note 当使用4线SPI时，本SDK使用硬件SPI接口进行数据传输。
 *       - SPI_CS引脚配置为普通GPIO功能，模式为推挽输出
 *       - SPI_SCK引脚配置为SPI功能，模式为推挽输出
 *       - SPI_MOSI引脚配置为SPI功能，模式为推挽输出
 *       - SPI_MISO引脚配置为SPI功能，模式为输入
 * @note 当使用3线SPI时，本SDK使用软件SPI接口进行数据传输。
 *       - SPI_CS引脚配置普通GPIO功能，为推挽输出
 *       - SPI_SCK引脚配置为普通GPIO功能，推挽输出
 *       - SPI_MOSI引脚配置为普通GPIO功能，推挽输出或输入模式，
 *         低功耗模式下须配置为输入带上拉电阻
 */
#define SPI_CS_HIGH()      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)     /* 将SPI_CS引脚拉高 */
#define SPI_CS_LOW()       LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)   /* 将SPI_CS引脚拉低 */

#if INTERFACE_MODE == USE_SPI_3LINE /* 使用3线SPI */
#define SPI_SCK_HIGH()     PORT_SetBits(SPI_SCK_PORT, SPI_SCK_PIN)     /* 将SPI_SCK引脚拉高 */
#define SPI_SCK_LOW()      PORT_ResetBits(SPI_SCK_PORT, SPI_SCK_PIN)   /* 将SPI_SCK引脚拉低 */
#define SPI_MOSI_HIGH()    PORT_SetBits(SPI_MOSI_PORT, SPI_MOSI_PIN)   /* 将SPI_MOSI引脚拉高 */
#define SPI_MOSI_LOW()     PORT_ResetBits(SPI_MOSI_PORT, SPI_MOSI_PIN) /* 将SPI_MOSI引脚拉低 */
#define SPI_MOSI_STATUS()  PORT_GetBit(SPI_MOSI_PORT, SPI_MOSI_PIN)    /* 检测SPI_MOSI引脚状态 */
#define SPI_MOSI_OUTPUT()  BSP_SetGpioMode(SPI_MOSI_PORT, SPI_MOSI_PIN, Pin_Mode_Out)  /* 设置SPI_MOSI引脚为输出模式 */
#define SPI_MOSI_INPUT()   BSP_SetGpioMode(SPI_MOSI_PORT, SPI_MOSI_PIN, Pin_Mode_In)   /* 设置SPI_MOSI引脚为输入模式 */
#endif

/** PA4   ------> IRQ
 * @brief 定义PAN3029/3060中断引脚电平读取宏
 * @note 该宏用于读取PAN3029/3060的中断引脚电平状态，高电平表示中断触发，低电平表示中断未触发
 */
#define CHECK_RF_IRQ()  LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)

/** PB0   ------> CAD
 * @brief 定义PAN3029/3060 CAD引脚电平读取宏
 * @note 该宏用于读取PAN3029/3060的CAD引脚电平状态，高电平表示信道忙，低电平表示信道空闲
 * @note 不使用CAD功能时，此宏可以不适配，保持PAN3029/3060的CAD(GPIO11)引脚上拉或者悬空即可
 */
#define CHECK_RF_CAD()  LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0)

/** PA5   ------> RESET
 * @brief 定义PAN3029/3060复位引脚宏
 * @note RF_RESET_PIN_LOW()用于将复位引脚设置为低电平，复位芯片
 * @note RF_RESET_PIN_High()用于释放复位控制信号
 * @note 不用硬件复位时，此宏可以不适配，保持PAN3029/3060的复位引脚上拉或者悬空即可
 */
#define RF_RESET_PIN_LOW()   LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5) /* 设置复位引脚为低电平 */
#define RF_RESET_PIN_High()  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5)   /* 设置复位引脚为高电平 */

/**TODO: 根据您的硬件做调整：
 * @brief PAN3029/3060 GPIO引脚定义
 */
#define MODULE_GPIO_TX             0  /* PAN3029/3060 天线发射控制引脚，高电平时表示打开发射天线，低电平表示关闭发射天线 */
#define MODULE_GPIO_RX             10 /* PAN3029/3060 天线接收控制引脚，高电平时表示打开接收天线，低电平表示关闭接收天线 */
#define MODULE_GPIO_TCXO           3  /* PAN3029/3060 TCXO控制引脚，用于打开或关闭TCXO电源 */
#define MODULE_GPIO_CAD_IRQ        11 /* PAN3029/3060 CAD中断引脚，检测CAD信号时输出高电平，否则输出低电平 */

/**
 * @brief PAN3029/3060的频段定义
 */
#define REGION_CN470_510           0x00 /* 中国470-510MHz频段 */
#define REGION_EU_863_870          0x01 /* 欧洲863-870MHz频段 */
#define REGION_US_902_928          0x02 /* 美国902-928MHz频段 */

#define REGION_DEFAULT             REGION_CN470_510 /* 默认频段为中国470-510MHz频段 */

/**
 * @brief 定义PAN3029/3060的默认参数，用户根据需要修改这些RF参数
 */
#define RF_FREQ_DEFAULT            490000000   /* 默认频率配置 */
#define RF_SF_DEFAULT              RF_SF7      /* 默认扩频因子配置 */
#define RF_BW_DEFAULT              RF_BW_500K  /* 默认带宽配置 */
#define RF_CR_DEFAULT              RF_CR_4_5   /* 默认信道编码率配置 */
#define RF_CRC_DEFAULT             RF_CRC_ON   /* 默认CRC校验配置 */
#define RF_LDR_DEFAULT             RF_LDR_OFF  /* 默认低速率优化配置 */
#define RF_PREAMBLE_DEFAULT        8           /* 默认前导码长度配置 */
#define RF_IQ_INVERT_DEFAULT       FALSE       /* 默认IQ调制配置 */

/**
 * @brief PAN3029/3060的外接晶振和复位引脚配置
 */
#define USE_ACTIVE_CRYSTAL         0 /* 是否使用外部有源晶振(0:不使用, 1: 使用) */
#define USE_RF_RST_GPIO            1 /* 是否使用MCU引脚控制RF复位功能(0:不使用, 1: 使用) */

/**
 * @brief PAN3029/3060的寄存器回读确认功能开关
 * @note 该功能用于在写入寄存器后，通过读取寄存器值来确认写入是否成功
 */
#define USE_RF_REG_CHECK           0 /* 是否使用寄存器回读确认功能(0:不使用, 1: 使用) */
#if USE_RF_REG_CHECK
#define RF_ASSERT(fn)        \
    do                       \
    {                        \
        if (RF_OK != fn)     \
        {                    \
            return RF_FAIL;  \
        }                    \
    } while (0);
#else
#define RF_ASSERT(fn)  fn
#endif

/**
 * @brief PAN3029/3060 中断标志位定义
 */
#define RF_IRQ_TX_DONE                  0x01 /* 发送完成中断标志 */
#define RF_IRQ_RX_TIMEOUT               0x02 /* 单次接收超时中断标志 */
#define RF_IRQ_CRC_ERR                  0x04 /* CRC错误中断标志 */
#define RF_IRQ_RX_DONE                  0x08 /* 接收完成中断标志 */
#define RF_IRQ_MAPM_DONE                0x40 /* MAPM中断标志 */

/**
 * @brief PAN3029/3060 调制模式定义
 */
#define MODEM_MODE_NORMAL               0x00 /* 普通调制模式 */
#define MODEM_MODE_MULTI_SECTOR         0x01 /* 多段调制模式 */

/**
 * @brief PAN3029/3060同步字定义
 * @note PAN3029/3060的同步字为单字节，范围为0x00-0xFF
 */
#define RF_MAC_PRIVATE_SYNCWORD         0x12 /* 私有网络同步字，默认值使用0x12 */
#define RF_MAC_PUBLIC_SYNCWORD          0x34 /* 公有网络同步字 */

/**
 * @brief PAN3029/3060 功率档位定义
 */
#define RF_MIN_RAMP                     1  /* 最小功率档位 */
#define RF_MAX_RAMP                     22 /* 最大功率档位 */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**
 * @brief RF相关操作的返回值定义
 * - RF_OK: 操作成功
 * - RF_FAIL: 操作失败
 */
typedef enum
{
    RF_OK,   //!< Operate ok
    RF_FAIL, //!< Operate fail
} RF_Err_t;

/**
 * @brief PAN3029/3060的工作状态定义
 */
typedef enum
{
    RF_STATE_DEEPSLEEP = 0x00, //!< The radio is in deep sleep mode
    RF_STATE_SLEEP = 0x01,     //!< The radio is in sleep mode
    RF_STATE_STB1 = 0x02,      //!< The radio is in standby mode 1
    RF_STATE_STB2 = 0x03,      //!< The radio is in standby mode 2
    RF_STATE_STB3 = 0x04,      //!< The radio is in standby mode 3
    RF_STATE_TX = 0x05,        //!< The radio is in transmit mode
    RF_STATE_RX = 0x06,        //!< The radio is in receive mode
} RfOpState_t;

/**
 * @brief PAN3029/3060的GPIO模式定义
 *        - GPIO_MODE_INPUT: GPIO输入模式
 *        - GPIO_MODE_OUTPUT: GPIO输出模式
 */
typedef enum
{
    GPIO_MODE_INPUT = 0x00, //!< GPIO input mode
    GPIO_MODE_OUTPUT = 0x01,//!< GPIO output mode
} RfGpioMode_t;

/**
 * @brief PAN3029/3060的发射模式定义
 *        - RF_TX_MODE_SINGLE: 单次发送模式,大多数情况下使用此模式
 *        - RF_TX_MODE_CONTINOUS: 连续发送模式，常用于载波信号发射
 */
typedef enum
{
    RF_TX_MODE_SINGLE = 0x00,    //!< Single transmission mode
    RF_TX_MODE_CONTINOUS = 0x01, //!< Continuous transmission mode
} RfTxMode_t;

/**
 * @brief PAN3029/3060的接收模式定义
 *        - RF_RX_MODE_SINGLE: 单次接收模式,接收到一包数据后自动进入待机模式，一般不用此模式
 *        - RF_RX_MODE_SINGLE_TIMEOUT: 单次带超时接收模式,超时后自动进入待机模式
 *        - RF_RX_MODE_CONTINOUS: 连续接收模式,接收到一包数据后继续接收
 */
typedef enum
{
    RF_RX_MODE_SINGLE = 0x00,         //!< Single reception mode
    RF_RX_MODE_SINGLE_TIMEOUT = 0x01, //!< Single reception with timeout mode
    RF_RX_MODE_CONTINOUS = 0x02,      //!< Continuous reception mode
} RfRxMode_t;

/**
 * @brief PAN3029/3060的供电模式定义
 *        - USE_LDO: 使用LDO供电，电流略大一点，但接收灵敏度略好
 *        - USE_DCDC: 使用DCDC供电，电流略小一点，但接收灵敏度略差1~2dB
 */
typedef enum
{
    USE_LDO = 0x00,  //!< Use LDO for power regulation
    USE_DCDC = 0x01, //!< Use DCDC for power regulation
} RfRegulatorMode_t;

/**
 * @brief PAN3029/3060的芯片模式定义
 *        - CHIPMODE_MODE0
 *        - CHIPMODE_MODE1
 */
typedef enum
{
    CHIPMODE_MODE0 = 0, //!< Mode 0
    CHIPMODE_MODE1 = 1, //!< Mode 1
} RfChipMode_t;

/**
 * @brief CAD检测门限值配置定义
 *        - 门限值越小，可检测到更弱的信号，但误报率会增加
 *        - 门限值越大，误报率会降低，但可检测到的信号强度范围会减小
 */
typedef enum
{
    RF_CAD_THRESHOLD_0A = 0x0A,
    RF_CAD_THRESHOLD_10 = 0x10,
    RF_CAD_THRESHOLD_15 = 0x15,
    RF_CAD_THRESHOLD_20 = 0x20,
} RfCadThreshold_t;

/**
 * @brief CAD检测符号数配置定义
 */
typedef enum
{
    RF_CAD_01_SYMBOL = 0x01, //!< 实际检测时间可能会大于1个符号的时间
    RF_CAD_02_SYMBOL = 0x02, //!< 实际检测时间可能会大于2个符号的时间
    RF_CAD_03_SYMBOL = 0x03, //!< 实际检测时间可能会大于3个符号的时间
    RF_CAD_04_SYMBOL = 0x04, //!< 实际检测时间可能会大于4个符号的时间
} RfCadSymbols_t;

/**
 * @brief Represents the possible spreading factor values in ChirpIot packet types
 */
typedef enum
{
    RF_SF5 = 0x05,  //!< 5 spreading factor
    RF_SF6 = 0x06,  //!< 6 spreading factor
    RF_SF7 = 0x07,  //!< 7 spreading factor
    RF_SF8 = 0x08,  //!< 8 spreading factor
    RF_SF9 = 0x09,  //!< 9 spreading factor
    RF_SF10 = 0x0A, //!< 10 spreading factor
    RF_SF11 = 0x0B, //!< 11 spreading factor
    RF_SF12 = 0x0C, //!< 12 spreading factor
} RfSpreadFactor_t;

/**
 * @brief Represents the bandwidth values for ChirpIot packet type
 */
typedef enum
{
    RF_BW_062K = 6, //!< 62.5KHz bandwidth
    RF_BW_125K = 7, //!< 125KHz bandwidth
    RF_BW_250K = 8, //!< 250KHz bandwidth
    RF_BW_500K = 9, //!< 500KHz bandwidth
} RfBandwidths_t;

/**
 * @brief Represents the coding rate values for ChirpIot packet type
 * @note  The coding rate is expressed as 4/x where x is the value below
 */
typedef enum
{
    RF_CR_4_5 = 0x01, //!< 4/5 coding rate
    RF_CR_4_6 = 0x02, //!< 4/6 coding rate
    RF_CR_4_7 = 0x03, //!< 4/7 coding rate
    RF_CR_4_8 = 0x04, //!< 4/8 coding rate
} RfCodingRates_t;

/**
 * @brief CRC模式
 * @note  RF_CRC_ON: 开启CRC校验
 * @note  RF_CRC_OFF: 关闭CRC校验，需要用户自己校验数据完整性
 */
typedef enum
{
    RF_CRC_OFF = 0x00, //!< CRC not used
    RF_CRC_ON = 0x01,  //!< CRC activated
} RfCrcModes_t;

/**
 * @brief 低速率优化模式
 * @note  RF_LDR_ON: Low data rate optimization activated
 * @note  RF_LDR_OFF: Low data rate optimization not used
 */
typedef enum
{
    RF_LDR_OFF = 0x00, //!< Low data rate optimization not used
    RF_LDR_ON = 0x01,  //!< Low data rate optimization activated
} RfLdr_t;

/**
 * @brief Represents the IQ mode for ChirpIot packet type
 */
typedef enum
{
    RF_IQ_NORMAL = 0x00,   //!< Normal IQ mode,default
    RF_IQ_INVERTED = 0x01, //!< Inverted IQ mode
} RfIQModes_t;

typedef enum
{
    RF_MAPM_GRP_ADDR = 0x00,    //!< Address group
    RF_MAPM_GRP_COUNTER = 0x01, //!< Counter group
} RfMapmGrpType_t;

/**
 * @brief  RF configuration structure of mapm
 * @member Addr: the address of mapm, which is 4 bytes
 * @member fn: the number of field before the standard preamble
 * @member fnm: the clone number of one field, which can be 0, 1, 2, 3, 
 *              fnm=0 indicates clone the field 1 times
 *              fnm=1 indicates clone the field 2 times
 *              fnm=2 indicates clone the field 4 times
 *              fnm=3 indicates clone the field 8 times
 * @member gfs: the function select of last address, which can be 0 or 1
 *              0: the last address is ordinary address
 *              1: the last address is field counter
 * @member gn: the number of group in one filed, which can be 1, 2, 3, 4
 *              gn=1 indicates 1 group(when gn=1, the field has only one group, fgs must be 0)
 *              gn=2 indicates 2 groups
 *              gn=3 indicates 3 groups
 *              gn=4 indicates 4 groups
 * @member pg1: the number of preamble in the first group, which can be 8~255
 *              pg1=8 indicates 8 chirps in the first group
 *              pg1=255 indicates 255 chirps in the first group
 * @member pgn: the number of preamble in the other group, which can be 0~255
 *              pgn=0 indicates 0 chirp in the other group, addr2~addr3 will be combined to addr1
 *              pgn=255 indicates 255 chirps in the other group
 * @member pn: the number of preamble between fields and syncword, which can be 1~65535
 * @note   the total numer of chirp before syncword: Pl=(pg1+1+pgn+1)*gn*fn*Fmux+pn
 * @note The mapm frame structure is as follows:
 *       1. The number of fields is fn, so N in the following figure equals fn;
 *       2. Preamble length in the figure is pn which is equal to the preamble length of normal
 *          ChirpIot packet;
 * |<---------------------------------- Mapm Frame------------------------------------------>|
 * | Field1 | Field2 | Field3 | ... | FieldN | Preamble | SyncWord | Header | Payload | CRC  |
 * |-----------------------------------------------------------------------------------------|
 * 
 * @note The mapm field structure is as follows(gn=4 && gfs=1):
 *       1. Each field has 4 groups, the preamble len of the fist group must be bigger than 8 
 *          and less than 255; the preamble len of the other groups can be 0~255;
 *       2. Each group carries 1byte payload, the type of first three groups is address, and
 *          the type of last(fourth) group is counter which indicates the number of left fields;
 *       3. The coding rate of the address and counter is 4/8, so the number of chirps in the 
 *          first group is pg1 + 2, and the number of chirps in the other groups is pgn + 2;
 * |<---------------------------------- Field ---------------------------------------------->|
 * |        Group1       |         Group2       |         Group3       |        Group4       |
 * |-----------------------------------------------------------------------------------------|
 * | Preamble | Address1 | Preamble | Address2  | Preamble | Address3  | Preamble | Counter  |
 * |-----------------------------------------------------------------------------------------|
 * 
 * @note The mapm field structure is as follows(gn=3 && gfs=1):
 *       1. Each field has 4 groups, the preamble len of the fist group must be bigger than 8 
 *          and less than 255; the preamble len of the other groups can be 0~255;
 *       2. Each group carries 1byte payload, the type of first two groups is address, and
 *          the type of last(third) group is counter which indicates the number of left fields;
 *       3. The coding rate of the address and counter is 4/8, so the number of chirps in the 
 *          first group is pg1 + 2, and the number of chirps in the other groups is pgn + 2;
 * |<---------------------------------- Field ----------------------->|
 * |        Group1       |         Group2       |        Group3       |
 * |------------------------------------------------------------------|
 * | Preamble | Address1 | Preamble | Address2  | Preamble | Counter  |
 * |------------------------------------------------------------------|
 */
typedef struct
{
    uint8_t Addr[4];      //!< the address of mapm, which is 4 bytes
    uint8_t fn;           //!< the number of field before the standard preamble
    uint8_t fnm;          //!< the clone number of one field, which can be 0, 1, 2, 3
    uint8_t gn;           //!< the number of group in one filed
    uint8_t gfs;          //!< the function select of last address
    uint8_t pg1;          //!< the number of preamble in the first group
    uint8_t pgn;          //!< the number of preamble in the other group
    uint16_t pn;          //!< the number of preamble between fields and syncword
} RF_MapmCfg_t;

/**
 * @brief Represents the possible operating states of the radio
 */
typedef struct
{
    uint8_t RxLen;          //!< Size of the payload in the ChirpIot packet
    uint8_t RxBuf[255];     //!< Buffer to store the received payload
    uint8_t MapmRxIndex;    //!< Index of the received payload in the ChirpIot packet
    uint8_t MapmRxBuf[16];  //!< Buffer to store the received payload in mapm mode
    int8_t Rssi;            //!< The RSSI of the received packet
    int8_t Snr;             //!< The SNR of the received packet
} RfRxPkt_t;

/**
 * @brief PAN3029/3060的配置参数结构体
 * @note  方便用户快速获取之前设置的参数
 */
typedef struct
{
    uint8_t TxPower;                  //!< The power level to be used for transmission
    uint32_t Frequency;               //!< The frequency to be used for transmission and reception
    RfSpreadFactor_t SpreadingFactor; //!< Spreading Factor for the ChirpIot modulation
    RfBandwidths_t Bandwidth;         //!< Bandwidth for the ChirpIot modulation
    RfCodingRates_t CodingRate;       //!< Coding rate for the ChirpIot modulation
    RfCrcModes_t CrcMode;             //!< Size of CRC block in ChirpIot packet
    RfIQModes_t InvertIQ;             //!< Allows to swap IQ for ChirpIot packet
    uint8_t SyncWord;                 //!< Sync word byte
    uint8_t LowDatarateOptimize;      //!< Indicates if the modem uses the low datarate optimization
    uint16_t PreambleLen;             //!< The preamble length is the number of ChirpIot symbols in the preamble
    RfChipMode_t ChipMode;            //!< The Chip for Communication with This Device
    RfRegulatorMode_t RegulatorMode;  //!< The regulator mode for the radio
} RfConfig_t;

/**
 * @brief PAN3029/3060接收数据包结构体
 * @note 该结构体用于存储接收到的数据包，包括数据长度、数据缓冲区、SNR和RSSI等信息
 */
extern volatile RfRxPkt_t g_RfRxPkt;

/**
 * RF definitions
 */

/**
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */

 
/**
 * @brief 微秒级延时函数
 * @param us 延时的微秒数
 */
void RF_DelayUs(uint32_t us);

/**
 * @brief 毫秒级延时函数
 * @param ms 延时的毫秒数
 */
void RF_DelayMs(uint32_t ms);

/**
 * @brief 写入单个字节到指定寄存器
 * @param Addr 要写入的寄存器地址
 * @param Value 要写入寄存器的单字节数据
 * @return RF_Err_t 返回操作结果
 */
RF_Err_t RF_WriteReg(uint8_t Addr, uint8_t Value);

/**
 * @brief 从指定的寄存器读取单个字节
 * @param Addr 要读取的寄存器地址
 * @return uint8_t 从寄存器读取的值
 */
uint8_t RF_ReadReg(uint8_t Addr);

/**
 * @brief 连续写入多个字节到指定寄存器区
 * @param Addr 要写入的寄存器区的起始地址
 * @param Buffer 要写入寄存器的缓冲区指针
 * @param Size 要写入的字节数
 */
void RF_WriteRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size);

/**
 * @brief 从指定的寄存器连续读取多个字节
 * @param Addr 要读取的寄存器地址
 * @param Buffer 存储读取数据的缓冲区指针
 * @param Size 要读取的字节数
 */
void RF_ReadRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size);

/**
 * @brief 选择寄存器页
 * @param Page 要选择的寄存器页
 * @return RF_Err_t 返回操作结果
 */
RF_Err_t RF_SetPage(uint8_t Page);

/**
 * @brief 写入单个字节到指定页的寄存器
 * @param Page 要写入的寄存器页
 * @param Addr 要写入的寄存器地址
 * @param Value 要写入寄存器的单字节数据
 * @return RF_Err_t 返回操作结果
 */
RF_Err_t RF_WritePageReg(uint8_t Page, uint8_t Addr, uint8_t Value);

/**
 * @brief 写入多个字节到指定页的寄存器区间
 * @param Page 要写入的寄存器页
 * @param Addr 要写入的寄存器地址
 * @param Buffer 要写入寄存器的缓冲区指针
 * @param Size 要写入的字节数
 */
void RF_WritePageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size);

/**
 * @brief 从指定页的寄存器读取单个字节
 * @param Page 要读取的寄存器页
 * @param Addr 要读取的寄存器地址
 * @return uint8_t 从寄存器读取的值
 */
uint8_t RF_ReadPageReg(uint8_t Page, uint8_t Addr);

/**
 * @brief 从指定页的寄存器区间读取多个字节
 * @param Page 要读取的寄存器页
 * @param Addr 要读取的寄存器地址
 * @param Buffer 存储读取数据的缓冲区指针
 * @param Size 要读取的字节数
 */
void RF_ReadPageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size);

/**
 * @brief 置位指定页的寄存器位
 * @param Page 要设置的寄存器页
 * @param Addr 要设置的寄存器地址
 * @param Mask 要设置的位掩码
 * @return RF_Err_t 返回操作结果
 */
RF_Err_t RF_SetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask);

/**
 * @brief 复位指定页的寄存器位
 * @param Page 要重置的寄存器页
 * @param Addr 要重置的寄存器地址
 * @param Mask 要重置的位掩码
 * @return RF_Err_t 返回操作结果
 */
RF_Err_t RF_ResetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask);

/**
 * @brief 写入指定页的寄存器位
 * @param Page 要写入的寄存器页
 * @param Addr 要写入的寄存器地址
 * @param Value 要写入的值
 * @param Mask 要写入的位掩码
 * @return RF_Err_t 返回操作结果
 */
RF_Err_t RF_WritePageRegBits(uint8_t Page, uint8_t Addr, uint8_t Value, uint8_t Mask);

/**
 * @brief 配置GPIO模式
 * @param[in] <GpioPin> 引脚号
 * @param[in] <GpioMode> GPIO模式
 * @return RF_Err_t 返回操作结果
 */
RF_Err_t RF_ConfigGpio(uint8_t GpioPin, uint8_t GpioMode);

/**
 * @brief 控制GPIO输出电平
 * @param[in] <GpioPin> 引脚号
 * @param[in] <Level> GPIO电平
 * @return RF_Err_t 返回操作结果
 */
RF_Err_t RF_WriteGpioLevel(uint8_t GpioPin, uint8_t Level);

/**
 * @brief 读取GPIO电平
 * @param[in] <GpioPin> 引脚号
 * @return 读取的GPIO电平
 */
uint8_t RF_ReadGpioLevel(uint8_t GpioPin);

/**
 * @brief 初始化PAN3029/3060的天线控制GPIO
 */
void RF_InitAntGpio(void);

/**
 * @brief 打开PAN3029/3060的发射天线
 */
void RF_TurnonTxAnt(void);

/**
 * @brief 打开PAN3029/3060的接收天线
 */
void RF_TurnonRxAnt(void);

/**
 * @brief 关闭PAN3029/3060的天线
 */
void RF_ShutdownAnt(void);

/**
 * @brief 初始化TCXO控制GPIO
 */
void RF_InitTcxoGpio(void);

/**
 * @brief 打开TCXO的供电电源
 */
void RF_TurnonTcxo(void);

/**
 * @brief 关闭TCXO的供电电源
 */
void RF_TurnoffTcxo(void);

/**
 * @brief Turnon LDO PA
 */
void RF_TurnonLdoPA(void);

/**
 * @brief Turnoff LDO PA
 */
void RF_TurnoffLdoPA(void);

/**
 * @brief 打开内部和外部PA
 */
void RF_TurnonPA(void);

/**
 * @brief 关闭内部和外部PA
 */
void RF_TurnoffPA(void);

/**
 * @brief 设置芯片模式
 * @param[in] <ChipMode> 芯片模式
 */
void RF_SetChipMode(RfChipMode_t ChipMode);

/**
 * @brief 获取芯片模式
 * @return RfChipMode_t 当前芯片模式
 */
RfChipMode_t RF_GetChipMode(void);

/**
 * @brief 上电后初始化RF收发器到STB3状态
 * @return 初始化是否成功，0表示成功，1表示失败
 */
RF_Err_t RF_Init(void);

/**
 * @brief 配置RF芯片的用户参数
 */
void RF_ConfigUserParams(void);

/**
 * @brief 软件复位RF芯片控制逻辑
 */
void RF_ResetLogic(void);

/**
 * @brief 获取RF芯片的工作状态
 * @return RfOpState_t 当前工作状态
 */
RfOpState_t RF_GetOperateState(void);

/**
 * @brief 设置RF芯片的工作状态
 * @param[in] <RfState> 工作状态
 */
void RF_SetOperateState(RfOpState_t RfState);

/**
 * @brief 设置RF芯片的工作状态
 * @param[in] <RfState> 工作状态
 */
void RF_SetRfState(uint8_t RfState);

/**
 * @brief 进入深度睡眠模式
 * @note 该函数用于将RF芯片置于深度睡眠模式，关闭天线供电和TCXO电源
 * @note 该函数会将芯片的工作状态设置为MODE_DEEPSLEEP
 * @note 执行该函数后，如需唤醒RF芯片，需要调用RF_Init()函数唤醒芯片
 */
void RF_EnterDeepsleepState(void);

/**
 * @brief 进入睡眠模式
 * @note 该函数用于将RF芯片置于睡眠模式，关闭天线供电和TCXO电源
 * @note 该函数会将芯片的工作状态设置为MODE_SLEEP
 * @note 执行该函数后，如需唤醒RF芯片，需要调用RF_ExitSleepState()函数
 */
void RF_EnterSleepState(void);

/**
 * @brief 退出睡眠模式
 */
void RF_ExitSleepState(void);

/**
 * @brief 进入待机模式
 */
void RF_EnterStandbyState(void);

/**
 * @brief 检查RF芯片是否处于休眠状态
 */
void RF_CheckDeviceReady(void);

/**
 * @brief 设置芯片的供电模式
 * @param[in] <RegulatorMode> 供电模式
 */
void RF_SetRegulatorMode(RfRegulatorMode_t RegulatorMode);

/**
 * @brief 设置RF芯片的频率
 * @param[in] <Frequency> 频率值
 */
RF_Err_t RF_SetFreq(uint32_t Frequency);

/**
 * @brief 设置IQ反转
 * @param[in] <enable> 使能或禁用IQ反转
 */
void RF_SetInvertIQ(bool NewState);

/**
 * @brief 设置前导码长度
 * @param[in] <PreamLen> 前导码长度值
 */
void RF_SetPreamLen(uint16_t PreamLen);

/**
 * @brief 设置同步字
 * @param[in] <syncWord> 同步字值
 */
void RF_SetSyncWord(uint8_t SyncWord);

/**
 * @brief 设置发射功率
 * @param[in] <TxPower> 发射档位
 */
void RF_SetTxPower(uint8_t TxPower);

/**
 * @brief 设置调制带宽
 * @param[in] <BandWidth> 调制带宽值
 *            - RF_BW_062K / RF_BW_125K / RF_BW_250K / RF_BW_500K
 * @note 调制带宽越大，数据速率越高，但传输距离越短
 * @note PAN3029芯片的调制带宽范围为RF_BW_062K - RF_BW_500K
 * @note PAN3060芯片的调制带宽范围为RF_BW_125K - RF_BW_500K
 */
void RF_SetBW(uint8_t BandWidth);

/**
 * @brief 设置扩频因子
 * @param[in] <SpreadFactor> 扩频因子值
 *          - RF_SF5 / RF_SF6 / RF_SF7 / RF_SF8 / RF_SF9 / RF_SF10 / RF_SF11 / RF_SF12
 * @note 扩频因子越大，传输距离越远，但数据速率越低
 * @note PAN3029芯片的扩频因子范围为RF_SF5 - RF_SF12
 * @note PAN3060芯片的扩频因子范围为RF_SF5 - RF_SF9
 */
void RF_SetSF(uint8_t SpreadFactor);

/**
 * @brief 设置信道编码率
 * @param[in] <CodingRate> 信道编码率值
 */
void RF_SetCR(uint8_t CodingRate);

/**
 * @brief 设置CRC校验
 * @param[in] <NewState> 使能或禁用CRC校验
 */
void RF_SetCRC(uint8_t CrcMode);

/**
 * @brief 设置低速率模式
 * @param[in] <LdrMode> 低速率模式值
 */
void RF_SetLDR(uint8_t LdrMode);

/**
 * @brief set modem mode
 * @param[in] <modem_mode>
 */
void RF_SetModemMode(uint8_t ModemMode);

/**
 * @brief 设置发送模式
 * @param Buffer 要发送的数据缓冲区
 * @param Size 要发送的数据字节数
 */
void RF_SetTx(uint8_t *Buffer, uint8_t Size);

/**
 * @brief 设置接收模式
 * @param TimeoutMs 接收超时时间
 */
void RF_SetRx(uint32_t TimeoutMs);

/**
 * @brief CAD function enable
 * @param[in] <Threshold>
 */
void RF_StartCad(uint8_t Threshold, uint8_t Chirps);

/**
 * @brief 设置CAD检测阈值
 * @param[in] <Threshold> CAD检测阈值
 *            - RF_CAD_THRESHOLD_0A
 *            - RF_CAD_THRESHOLD_10
 *            - RF_CAD_THRESHOLD_15
 *            - RF_CAD_THRESHOLD_20
 */
void RF_SetCadThreshold(uint8_t Threshold);

/**
 * @brief 设置CAD检测符号数
 * @param[in] <Chirps> CAD检测符号数
 *            - RF_CAD_01_SYMBOL
 *            - RF_CAD_02_SYMBOL
 *            - RF_CAD_03_SYMBOL
 *            - RF_CAD_04_SYMBOL
 */
void RF_SetCadChirps(uint8_t Chirps);

/**
 * @brief CAD function disable
 */
void RF_StopCad(void);

/**
 * @brief 设置发射模式
 * @param[in] <TxMode>
 */
void RF_SetTxMode(uint8_t TxMode);

/**
 * @brief 发送单个数据包
 * @param[in] <Buffer> 要发送的数据缓冲区
 * @param[in] <Size> 要发送的数据字节数
 */
void RF_TxSinglePkt(uint8_t *Buffer, uint8_t Size);

/**
 * @brief 设置接收模式
 * @param[in] <RxMode> 接收模式
 */
void RF_SetRxMode(uint8_t RxMode);

/**
 * @brief 让芯片进入连续接收状态
 */
void RF_EnterContinousRxState(void);

/**
 * @brief 设置接收超时时间
 * @param[in] <TimeoutMs> 超时时间
 */
void RF_SetRxTimeout(uint16_t TimeoutMs);

/**
 * @brief 让芯片进入带超时的单次接收状态
 * @param[in] <TimeoutMs> 超时时间
 */
void RF_EnterSingleRxWithTimeout(uint16_t TimeoutMs);

/**
 * @brief 获取接收数据长度
 * @return 接收数据长度
 */
uint8_t RF_GetRxPayloadLen(void);

/**
 * @brief 函数用于获取接收数据长度及内容
 * @param *Buffer 待接收数据区指针地址
 * @return 接收到的数据长度
 */
uint8_t RF_GetRecvPayload(uint8_t *Buffer);

/**
 * @brief 获取接收数据包的RSSI值
 * @return RSSI值
 */
int8_t RF_GetPktRssi(void);

/**
 * @brief 获取实时RSSI值
 * @return RSSI值
 */
int8_t RF_GetRealTimeRssi(void);

/**
 * @brief 获取接收数据包的SNR值
 * @return SNR值
 */
int32_t RF_GetPktSnr(void);

/**
 * @brief 获取中断标志位
 * @return IRQ标志位
 */
uint8_t RF_GetIRQFlag(void);

/**
 * @brief 清中断标志位
 * @param[in] <IRQFlag> 中断标志位
 */
void RF_ClrIRQFlag(uint8_t IRQFlag);

/**
 * @brief 获取当前的频率设置
 */
uint32_t RF_GetFreq(void);

/**
 * @brief 获取IQ反转值
 */
RfIQModes_t RF_GetInvertIQ(void);

/**
 * @brief 获取当前的前导码长度设置
 */
uint16_t RF_GetPreamLen(void);

/**
 * @brief 获取当前的发射功率设置
 */
uint8_t RF_GetTxPower(void);

/**
 * @brief 获取当前的调制带宽设置
 */
uint8_t RF_GetBandWidth(void);

/**
 * @brief 获取当前的扩频因子设置
 */
uint8_t RF_GetSF(void);

/**
 * @brief 获取当前的CRC校验设置
 */
uint8_t RF_GetCRC(void);

/**
 * @brief 获取当前的编码率设置
 */
uint8_t RF_GetCR(void);

/**
 * @brief 获取当前的同步字设置
 */
uint8_t RF_GetSyncWord(void);

/**
 * @brief 获取当前的发射模式设置
 */
uint8_t RF_GetLDR(void);

/**
 * @brief 获取单个符号的时间
 * @param[in] <bw> 带宽
 *           - RF_BW_062K / RF_BW_125K / RF_BW_250K / RF_BW_500K
 * @param[in] <sf> 扩频因子
 *           - RF_SF5 / RF_SF6 / RF_SF7 / RF_SF8 / RF_SF9 / RF_SF10 / RF_SF11 / RF_SF12
 * @return 单个符号的时间，单位为us
 * @note 该函数用于计算单个符号的时间
 */
uint32_t RF_GetOneSymbolTime(uint8_t bw, uint8_t sf);

/**
 * @brief 计算发送数据包的时间
 * @param[in] <Size> 发送数据包的大小，单位为字节
 * @return 发送数据包的时间，单位为ms
 */
uint32_t RF_GetTxTimeMs(uint8_t Size);

/**
 * @brief 启动mapm模式
 */
void RF_EnableMapm(void);

/**
 * @brief 关闭mapm模式
 */
void RF_DisableMapm(void);

/**
 * @brief 配置mapm相关参数
 * @param[in] <pMapmCfg>
 */
void RF_ConfigMapm(RF_MapmCfg_t *pMapmCfg);

/**
 * @brief 设置mapm模式下的组地址
 * @param[in] <MapmAddr> mapm组地址
 *            <AddrWidth> 地址宽度,范围为1~4
 * @note 接收端的MapmAddr[0]须与发送端的MapmAddr[0]一致，
 *       否则接收端不会触发mapm中断。
 */
void RF_SetMapmAddr(uint8_t *MapmAddr, uint8_t AddrWidth);

/**
 * @brief 计算1个field花费的时间(ms)
 * @param[in] <pMapmCfg> mapm配置参数
 *            <SymbolTime> 单个symbol(chirp)时间
 * @note Group1中chirp的数量为(pg1 + 2)，其中pg1为Group1中前导码的个数，2为Group1中地址占用的chirp数。
 * @note 其它Group中chirp的数量为(pgn + 2)*(gn-1)，其中pgn为其它单个Group中前导码的个数，
 *       2为其它单个Group中地址(或计数值)占用的chirp数，(gn-1)为除去Group1后剩余的Group数。
 */
uint32_t RF_GetMapmOneFieldTime(RF_MapmCfg_t *pMapmCfg, uint32_t SymbolTime);

/**
 * @brief 获取mapm模式下的剩余Mapm时间
 * @param[in] <pMapmCfg> mapm配置参数
 * @param[in] <SynbolTime> 单个chirp时间
 * @return 剩余Mapm时间，单位为ms
 */
uint32_t RF_GetLeftMapmTime(RF_MapmCfg_t *pMapmCfg, uint32_t SynbolTime);

/**
 * @brief 开始发送连续载波
 */
void RF_StartTxContinuousWave(void);

/**
 * @brief 停止发送连续载波
 */
void RF_StopTxContinuousWave(void);

/**
 * @brief 处理RF中断
 * @note 此函数会在中断服务函数中调用
 */
void RF_IRQ_Process(void);

/** \} defgroup PAN3029/3060 */
/** \} addtogroup ChirpIOT */

#endif // __PAN_RF_H__
