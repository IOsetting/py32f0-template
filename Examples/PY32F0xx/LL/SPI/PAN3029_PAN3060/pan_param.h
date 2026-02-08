/**
 ******************************************************************************
 * @file    pan_param.h
 * @brief   RF parameter tables for PAN3029/PAN3060 (power ramp, defaults, AGC, SNR LUT).
 * @note    These are static configuration tables used by pan_rf.c during initialization
 *          and runtime parameter conversions. No functions are defined here.
 * @history - V1.0.1, 2025-08-18
 ******************************************************************************
 */

#ifndef  _RF_PARAMS_H_
#define  _RF_PARAMS_H_

#include <stdint.h>
#include "pan_rf.h"

/*
 * TX power ramp configuration entry.
 * - Ramp:   PA ramp timing/shape configuration register value
 * - Ldo:    PA LDO related configuration register value
 * - PAbias: PA bias setting
 */
typedef struct
{
    uint8_t Ramp;   //!< Ramp timing/shape register value
    uint8_t Ldo;    //!< PA LDO configuration
    uint8_t PAbias; //!< PA bias configuration
} RF_PowerRampCfg_t;

/* Power levels 1..RF_MAX_RAMP. Indexed by desired power step. */
static const RF_PowerRampCfg_t g_RfPowerRampCfg[RF_MAX_RAMP + 1] =
{
    {0x01, 0x01, 0x00}, //!< 功率档位1
    {0x03, 0x01, 0x01}, //!< 功率档位2
    {0x03, 0xF0, 0x30}, //!< 功率档位3
    {0x05, 0x01, 0x81}, //!< 功率档位4
    {0x05, 0xA1, 0x81}, //!< 功率档位5
    {0x05, 0xF0, 0x81}, //!< 功率档位6
    {0x07, 0x01, 0x81}, //!< 功率档位7
    {0x05, 0x01, 0x80}, //!< 功率档位8
    {0x05, 0x31, 0x80}, //!< 功率档位9
    {0x07, 0x01, 0x80}, //!< 功率档位10
    {0x07, 0x21, 0x80}, //!< 功率档位11
    {0x0B, 0x11, 0x80}, //!< 功率档位12
    {0x0B, 0x21, 0x80}, //!< 功率档位13
    {0x0B, 0x41, 0x80}, //!< 功率档位14
    {0x0B, 0x61, 0x80}, //!< 功率档位15
    {0x0B, 0x91, 0x80}, //!< 功率档位16
    {0x0B, 0xB1, 0x80}, //!< 功率档位17
    {0x0D, 0xB1, 0x80}, //!< 功率档位18
    {0x0F, 0xB1, 0x80}, //!< 功率档位19
    {0x11, 0x50, 0x80}, //!< 功率档位20
    {0x15, 0x30, 0x20}, //!< 功率档位21
    {0x15, 0x50, 0x70}, //!< 功率档位22
};

/* Single register write item (page, address, value). */
typedef struct
{
    uint8_t Page;  //!< Register page index
    uint8_t Addr;  //!< Register address within the page
    uint8_t Value; //!< Register value to write
} PAN_RegCfg_t;

/**
 * @brief PAN3029/3060 推荐的默认寄存器配置
 * @note 该配置参数用于在芯片上电后，初始化RF收发器的寄存器值
 */
static const PAN_RegCfg_t g_RfDefaultConfig[] = 
{
    {0, 0x03, 0x1B},
    {0, 0x04, 0x76},
    {0, 0x06, 0x01},
    {0, 0x0B, 0x04},
    {0, 0x13, 0x04},
    {0, 0x11, 0x20},
    {0, 0x12, 0x10},
    {0, 0x1F, 0x07},
    {0, 0x20, 0x07},
    {0, 0x24, 0x03},
    {0, 0x46, 0x03},
    {0, 0x25, 0x00},
    {0, 0x21, 0x07},
    {0, 0x22, 0x07},
    {0, 0x15, 0x21},
    {0, 0x31, 0xD0},
    {0, 0x36, 0x66},
    {0, 0x37, 0x6B},
    {0, 0x38, 0xCC},
    {0, 0x39, 0x09},
    {0, 0x3C, 0xB4},
    {0, 0x3E, 0x42},
    {0, 0x40, 0x6A},
    {0, 0x41, 0x06},
    {0, 0x42, 0xAA},
    {0, 0x48, 0x77},
    {0, 0x49, 0x77},
    {0, 0x4A, 0x77},
    {0, 0x4B, 0x05},
    {0, 0x4F, 0x04},
    {0, 0x50, 0xD2},
    {0, 0x5E, 0x80},
    {1, 0x03, 0x1B},
    {1, 0x04, 0x76},
    {1, 0x0B, 0x08},
    {1, 0x0F, 0x0A},
    {1, 0x19, 0x00},
    {1, 0x2F, 0xD0},
    {1, 0x43, 0xDA},
    {2, 0x03, 0x1B},
    {2, 0x04, 0x76},
    {2, 0x2C, 0xC0},
    {2, 0x2D, 0x27},
    {2, 0x2E, 0x09},
    {2, 0x2F, 0x00},
    {2, 0x30, 0x10},
    {3, 0x03, 0x1B},
    {3, 0x04, 0x76},
    {3, 0x0A, 0x0E},
    {3, 0x0B, 0xCF},
    {3, 0x0C, 0x19},
    {3, 0x0D, 0x98},
    {3, 0x12, 0x16},
    {3, 0x13, 0x14},
    {3, 0x16, 0xF4},
    {3, 0x17, 0x01},
    {3, 0x1F, 0xD9},
};

/* Frequency band parameters lookup, selected by user-set frequency in Hz. */
typedef struct
{
    uint32_t StartFreq;  //!< Start frequency in Hz (inclusive)
    uint32_t StopFreq;   //!< Stop  frequency in Hz (exclusive)
    uint8_t  VcoParam;   //!< VCO configuration parameter
    uint8_t  FreqFactor; //!< Frequency factor parameter
    uint8_t  LoParam;    //!< LO parameter
} RadioFreqTable_t;

/**
 * @brief PAN3029/3060 频率参数表
 * @note 用户在设置不同频率时，需要用到该参数表，
 *       不同的频率对应不同的vco参数、频率因子和lo参数
 * @note 该表支持的频率范围为：
 *       低频段：
 *        - 138.33MHz ~ 180.00MHz
 *        - 207.00MHz ~ 270.00MHz
 *        - 282.50MHz ~ 360.00MHz
 *        - 408.00MHz ~ 540.00MHz
 *       高频段：
 *        - 810.00MHz ~ 1080.00MHz
 */
static const RadioFreqTable_t g_RfFreqTable[] = 
{
    // Fstart    Fstop       0x40  0x41  0x3D        min     mid     max
    {138330000,  143330000,  0x2A, 0x0C, 0xC0}, //!< 131.6M  140.2M  148.8M
    {143330000,  148330000,  0x3A, 0x0C, 0xC0}, //!< 135.1M  144.5M  153.8M
    {148330000,  155000000,  0x4A, 0x0C, 0xC0}, //!< 138.8M  149.2M  159.5M
    {155000000,  161000000,  0x5A, 0x0C, 0xC0}, //!< 142.9M  154.3M  165.7M
    {161000000,  168000000,  0x6A, 0x0C, 0xC0}, //!< 147.4M  160.1M  172.7M
    {168000000,  180000000,  0x7A, 0x0C, 0xC0}, //!< 152.3M  166.4M  180.5M
    {207500000,  215000000,  0x2A, 0x08, 0xB0}, //!< 197.4M  210.3M  223.3M
    {215000000,  222500000,  0x3A, 0x08, 0xB0}, //!< 202.6M  216.8M  230.8M
    {222500000,  232500000,  0x4A, 0x08, 0xB0}, //!< 208.3M  223.8M  239.3M
    {232500000,  242500000,  0x5A, 0x08, 0xB0}, //!< 214.4M  231.5M  248.5M
    {242500000,  252500000,  0x6A, 0x08, 0xB0}, //!< 221.1M  240.1M  259.0M
    {252500000,  270000000,  0x7A, 0x08, 0xB0}, //!< 228.5M  249.6M  270.8M
    {282500000,  287000000,  0x2A, 0x06, 0xA0}, //!< 263.2M  280.3M  297.7M
    {287000000,  297000000,  0x3A, 0x06, 0xA0}, //!< 270.2M  289.0M  307.7M
    {297000000,  310000000,  0x4A, 0x06, 0xA0}, //!< 277.7M  298.3M  319.0M
    {310000000,  323000000,  0x5A, 0x06, 0xA0}, //!< 285.8M  308.7M  331.3M
    {323000000,  337000000,  0x6A, 0x06, 0xA0}, //!< 294.8M  320.2M  345.3M
    {337000000,  360000000,  0x7A, 0x06, 0xA0}, //!< 304.7M  332.8M  361.0M
    {408000000,  415000000,  0x1A, 0x06, 0x90}, //!< 385.0M  408.8M  432.8M
    {415000000,  430000000,  0x2A, 0x06, 0x90}, //!< 394.8M  420.5M  446.5M
    {430000000,  445000000,  0x3A, 0x06, 0x90}, //!< 405.3M  433.5M  461.5M
    {445000000,  465000000,  0x4A, 0x06, 0x90}, //!< 416.5M  447.5M  478.5M
    {465000000,  485000000,  0x5A, 0x06, 0x90}, //!< 428.8M  463.0M  497.0M
    {485000000,  505000000,  0x6A, 0x06, 0x90}, //!< 442.3M  480.3M  518.0M
    {505000000,  540000000,  0x7A, 0x06, 0x90}, //!< 457.0M  499.3M  541.5M
    {810000000,  830000000,  0x1A, 0x02, 0x80}, //!< 770.0M  817.5M  865.5M
    {830000000,  860000000,  0x2A, 0x02, 0x80}, //!< 789.5M  841.0M  893.0M
    {860000000,  890000000,  0x3A, 0x02, 0x80}, //!< 810.5M  867.0M  923.0M
    {890000000,  930000000,  0x4A, 0x02, 0x80}, //!< 833.0M  895.0M  957.0M
    {930000000,  970000000,  0x5A, 0x02, 0x80}, //!< 857.5M  926.0M  994.0M
    {970000000,  1010000000, 0x6A, 0x02, 0x80}, //!< 884.5M  960.5M  1036.0M
    {1010000000, 1080000000, 0x7A, 0x02, 0x80}, //!< 914.0M  998.5M  1083.0M
};

/**
 * @brief VCO分频系数表
 * @note 该表用于存储VCO的分频系数
 */
static const uint8_t g_VcoDivTable[] =
{
    2, 4, 6, 8, 12, 16, 4, 4,
};

/**
 * @brief PAN3029/3060 低频段AGC配置参数
 * @note 该配置参数用于设置低频段的AGC（自动增益控制）相关寄存器的值
 * @note 该参数支持的频段：138.33MHz ~ 282.5MHz，405.00MHz ~ 565.00MHz
 */
static const uint8_t g_LowFreqAgcCfg[40] = 
{
    0x06, 0x00, 0xf8, 0x06, 0x06, 0x00, 0xf8, 0x06,
    0x06, 0x00, 0xf8, 0x06, 0x06, 0x00, 0xf8, 0x06,
    0x14, 0xc0, 0xf9, 0x14, 0x22, 0xd4, 0xf9, 0x22,
    0x30, 0xd8, 0xf9, 0x30, 0x3e, 0xde, 0xf9, 0x3e,
    0x0e, 0xff, 0x80, 0x4f, 0x12, 0x80, 0x38, 0x01
};

/**
 * @brief PAN3029/3060 低频段AGC配置参数
 * @note 该配置参数用于设置低频段的AGC（自动增益控制）相关寄存器的值
 * @note 该参数支持的频段：810.00MHz ~ 1080.00MHz
 */
static const uint8_t g_HighFreqAgcCfg[40] = 
{
    0x09, 0x80, 0xf3, 0x09, 0x09, 0x80, 0xf3, 0x09,
    0x09, 0x80, 0xf3, 0x09, 0x09, 0x80, 0xf3, 0x09,
    0x14, 0x06, 0xf0, 0x14, 0x22, 0xc6, 0xf1, 0x22,
    0x31, 0x73, 0xf0, 0x31, 0x3f, 0xde, 0xf1, 0x3f, 
    0x0e, 0xff, 0xe0, 0x32, 0x29, 0x80, 0x38, 0x01
};

/**
 * @brief 该表用于将信噪比(SNR)转换为线性值
 * @note SNR的范围是-10到20dB，对应的线性值范围是10到10240
 * @note 为了减少MCU的计算负担，使用了对数表来进行转换
 */
const uint32_t g_SnrLog10Talbe[31] = 
{
    10,   12,   15,   19,   26,   32,   40,   51,   65,   81,
    102,  128,  162,  204,  257,  324,  407,  513,  645,  813,
    1024, 1288, 1623, 2043, 2572, 3237, 4078, 5131, 6462, 8130,
    10240
};

#endif // ! RF_PARAMS_H
