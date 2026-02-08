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


/** TODO: Adjust according to your hardware:
 * @note When using 4-wire SPI, this SDK uses hardware SPI interface for data transmission.
 *       - SPI_CS pin configured as general GPIO function, push-pull output mode
 *       - SPI_SCK pin configured as SPI function, push-pull output mode
 *       - SPI_MOSI pin configured as SPI function, push-pull output mode
 *       - SPI_MISO pin configured as SPI function, input mode
 * @note When using 3-wire SPI, this SDK uses software SPI interface for data transmission.
 *       - SPI_CS pin configured as general GPIO function, push-pull output
 *       - SPI_SCK pin configured as general GPIO function, push-pull output
 *       - SPI_MOSI pin configured as general GPIO function, push-pull output or input mode,
 *         must be configured as input with pull-up resistor in low power mode
 */
#define SPI_CS_HIGH()      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6)     /* Pull SPI_CS pin high */
#define SPI_CS_LOW()       LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6)   /* Pull SPI_CS pin low */

#if INTERFACE_MODE == USE_SPI_3LINE /* Use 3-wire SPI */
#define SPI_SCK_HIGH()     PORT_SetBits(SPI_SCK_PORT, SPI_SCK_PIN)     /* Pull SPI_SCK pin high */
#define SPI_SCK_LOW()      PORT_ResetBits(SPI_SCK_PORT, SPI_SCK_PIN)   /* Pull SPI_SCK pin low */
#define SPI_MOSI_HIGH()    PORT_SetBits(SPI_MOSI_PORT, SPI_MOSI_PIN)   /* Pull SPI_MOSI pin high */
#define SPI_MOSI_LOW()     PORT_ResetBits(SPI_MOSI_PORT, SPI_MOSI_PIN) /* Pull SPI_MOSI pin low */
#define SPI_MOSI_STATUS()  PORT_GetBit(SPI_MOSI_PORT, SPI_MOSI_PIN)    /* Detect SPI_MOSI pin status */
#define SPI_MOSI_OUTPUT()  BSP_SetGpioMode(SPI_MOSI_PORT, SPI_MOSI_PIN, Pin_Mode_Out)  /* Set SPI_MOSI pin to output mode */
#define SPI_MOSI_INPUT()   BSP_SetGpioMode(SPI_MOSI_PORT, SPI_MOSI_PIN, Pin_Mode_In)   /* Set SPI_MOSI pin to input mode */
#endif

/** PA4   ------> IRQ
 * @brief Define PAN3029/3060 interrupt pin level read macro
 * @note This macro is used to read the interrupt pin level state of PAN3029/3060, high level indicates interrupt triggered, low level indicates interrupt not triggered
 */
#define CHECK_RF_IRQ()  LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4)

/** PB0   ------> CAD
 * @brief Define PAN3029/3060 CAD pin level read macro
 * @note This macro is used to read the CAD pin level state of PAN3029/3060, high level indicates channel busy, low level indicates channel idle
 * @note When CAD function is not used, this macro can be not adapted, keep PAN3029/3060 CAD(GPIO11) pin pulled up or floating
 */
#define CHECK_RF_CAD()  LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0)

/** PA5   ------> RESET
 * @brief Define PAN3029/3060 reset pin macro
 * @note RF_RESET_PIN_LOW() is used to set the reset pin to low level to reset the chip
 * @note RF_RESET_PIN_High() is used to release the reset control signal
 * @note When hardware reset is not used, this macro can be not adapted, keep PAN3029/3060 reset pin pulled up or floating
 */
#define RF_RESET_PIN_LOW()   LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5) /* Set reset pin to low level */
#define RF_RESET_PIN_High()  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5)   /* Set reset pin to high level */

/**TODO: Adjust according to your hardware:
 * @brief PAN3029/3060 GPIO pin definitions
 */
#define MODULE_GPIO_TX             0  /* PAN3029/3060 antenna transmit control pin, high level indicates transmit antenna enabled, low level indicates transmit antenna disabled */
#define MODULE_GPIO_RX             10 /* PAN3029/3060 antenna receive control pin, high level indicates receive antenna enabled, low level indicates receive antenna disabled */
#define MODULE_GPIO_TCXO           3  /* PAN3029/3060 TCXO control pin, used to enable or disable TCXO power */
#define MODULE_GPIO_CAD_IRQ        11 /* PAN3029/3060 CAD interrupt pin, outputs high level when CAD signal is detected, otherwise outputs low level */

/**
 * @brief PAN3029/3060 frequency band definitions
 */
#define REGION_CN470_510           0x00 /* China 470-510MHz band */
#define REGION_EU_863_870          0x01 /* Europe 863-870MHz band */
#define REGION_US_902_928          0x02 /* United States 902-928MHz band */

#define REGION_DEFAULT             REGION_CN470_510 /* Default band is China 470-510MHz band */

/**
 * @brief Define default parameters for PAN3029/3060, users can modify these RF parameters as needed
 */
#define RF_FREQ_DEFAULT            490000000   /* Default frequency configuration */
#define RF_SF_DEFAULT              RF_SF7      /* Default spreading factor configuration */
#define RF_BW_DEFAULT              RF_BW_500K  /* Default bandwidth configuration */
#define RF_CR_DEFAULT              RF_CR_4_5   /* Default channel coding rate configuration */
#define RF_CRC_DEFAULT             RF_CRC_ON   /* Default CRC check configuration */
#define RF_LDR_DEFAULT             RF_LDR_OFF  /* Default low data rate optimization configuration */
#define RF_PREAMBLE_DEFAULT        8           /* Default preamble length configuration */
#define RF_IQ_INVERT_DEFAULT       FALSE       /* Default IQ modulation configuration */

/**
 * @brief PAN3029/3060 external crystal oscillator and reset pin configuration
 */
#define USE_ACTIVE_CRYSTAL         0 /* Whether to use external active crystal oscillator(0: not use, 1: use) */
#define USE_RF_RST_GPIO            1 /* Whether to use MCU pin to control RF reset function(0: not use, 1: use) */

/**
 * @brief PAN3029/3060 register read-back confirmation function switch
 * @note This function is used to confirm whether the write is successful by reading the register value after writing to the register
 */
#define USE_RF_REG_CHECK           0 /* Whether to use register read-back confirmation function(0: not use, 1: use) */
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
 * @brief PAN3029/3060 interrupt flag definitions
 */
#define RF_IRQ_TX_DONE                  0x01 /* Transmit complete interrupt flag */
#define RF_IRQ_RX_TIMEOUT               0x02 /* Single reception timeout interrupt flag */
#define RF_IRQ_CRC_ERR                  0x04 /* CRC error interrupt flag */
#define RF_IRQ_RX_DONE                  0x08 /* Receive complete interrupt flag */
#define RF_IRQ_MAPM_DONE                0x40 /* MAPM interrupt flag */

/**
 * @brief PAN3029/3060 modulation mode definitions
 */
#define MODEM_MODE_NORMAL               0x00 /* Normal modulation mode */
#define MODEM_MODE_MULTI_SECTOR         0x01 /* Multi-sector modulation mode */

/**
 * @brief PAN3029/3060 sync word definitions
 * @note The sync word of PAN3029/3060 is a single byte, range 0x00-0xFF
 */
#define RF_MAC_PRIVATE_SYNCWORD         0x12 /* Private network sync word, default value 0x12 */
#define RF_MAC_PUBLIC_SYNCWORD          0x34 /* Public network sync word */

/**
 * @brief PAN3029/3060 power level definitions
 */
#define RF_MIN_RAMP                     1  /* Minimum power level */
#define RF_MAX_RAMP                     22 /* Maximum power level */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**
 * @brief RF related operation return value definitions
 * - RF_OK: Operation successful
 * - RF_FAIL: Operation failed
 */
typedef enum
{
    RF_OK,   //!< Operate ok
    RF_FAIL, //!< Operate fail
} RF_Err_t;

/**
 * @brief PAN3029/3060 operating state definitions
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
 * @brief PAN3029/3060 GPIO mode definitions
 *        - GPIO_MODE_INPUT: GPIO input mode
 *        - GPIO_MODE_OUTPUT: GPIO output mode
 */
typedef enum
{
    GPIO_MODE_INPUT = 0x00, //!< GPIO input mode
    GPIO_MODE_OUTPUT = 0x01,//!< GPIO output mode
} RfGpioMode_t;

/**
 * @brief PAN3029/3060 transmission mode definitions
 *        - RF_TX_MODE_SINGLE: Single transmission mode, used in most cases
 *        - RF_TX_MODE_CONTINOUS: Continuous transmission mode, commonly used for carrier signal transmission
 */
typedef enum
{
    RF_TX_MODE_SINGLE = 0x00,    //!< Single transmission mode
    RF_TX_MODE_CONTINOUS = 0x01, //!< Continuous transmission mode
} RfTxMode_t;

/**
 * @brief PAN3029/3060 reception mode definitions
 *        - RF_RX_MODE_SINGLE: Single reception mode, automatically enters standby mode after receiving a packet, generally not used
 *        - RF_RX_MODE_SINGLE_TIMEOUT: Single reception with timeout mode, automatically enters standby mode after timeout
 *        - RF_RX_MODE_CONTINOUS: Continuous reception mode, continues to receive after receiving a packet
 */
typedef enum
{
    RF_RX_MODE_SINGLE = 0x00,         //!< Single reception mode
    RF_RX_MODE_SINGLE_TIMEOUT = 0x01, //!< Single reception with timeout mode
    RF_RX_MODE_CONTINOUS = 0x02,      //!< Continuous reception mode
} RfRxMode_t;

/**
 * @brief PAN3029/3060 power supply mode definitions
 *        - USE_LDO: Use LDO for power supply, current is slightly larger, but receive sensitivity is slightly better
 *        - USE_DCDC: Use DCDC for power supply, current is slightly smaller, but receive sensitivity is slightly worse by 1~2dB
 */
typedef enum
{
    USE_LDO = 0x00,  //!< Use LDO for power regulation
    USE_DCDC = 0x01, //!< Use DCDC for power regulation
} RfRegulatorMode_t;

/**
 * @brief PAN3029/3060 chip mode definitions
 */
typedef enum
{
    CHIPMODE_MODE0 = 0, //!< Mode 0
    CHIPMODE_MODE1 = 1, //!< Mode 1
} RfChipMode_t;

/**
 * @brief CAD detection threshold configuration definitions
 *        - The smaller the threshold value, the weaker the signal that can be detected, but the false alarm rate will increase
 *        - The larger the threshold value, the lower the false alarm rate, but the detectable signal strength range will decrease
 */
typedef enum
{
    RF_CAD_THRESHOLD_0A = 0x0A,
    RF_CAD_THRESHOLD_10 = 0x10,
    RF_CAD_THRESHOLD_15 = 0x15,
    RF_CAD_THRESHOLD_20 = 0x20,
} RfCadThreshold_t;

/**
 * @brief CAD detection symbol number configuration definitions
 */
typedef enum
{
    RF_CAD_01_SYMBOL = 0x01, //!< Actual detection time may be longer than 1 symbol time
    RF_CAD_02_SYMBOL = 0x02, //!< Actual detection time may be longer than 2 symbol time
    RF_CAD_03_SYMBOL = 0x03, //!< Actual detection time may be longer than 3 symbol time
    RF_CAD_04_SYMBOL = 0x04, //!< Actual detection time may be longer than 4 symbol time
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
 * @brief CRC mode
 * @note  RF_CRC_ON: Enable CRC check
 * @note  RF_CRC_OFF: Disable CRC check, user needs to verify data integrity themselves
 */
typedef enum
{
    RF_CRC_OFF = 0x00, //!< CRC not used
    RF_CRC_ON = 0x01,  //!< CRC activated
} RfCrcModes_t;

/**
 * @brief Low data rate optimization mode
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
 * @brief PAN3029/3060 configuration parameter structure
 * @note  Convenient for users to quickly obtain previously set parameters
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
 * @brief PAN3029/3060 received data packet structure
 * @note This structure is used to store received data packets, including data length, data buffer, SNR and RSSI information, etc.
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
 * @brief Microsecond delay function
 * @param us Number of microseconds to delay
 */
void RF_DelayUs(uint32_t us);

/**
 * @brief Millisecond delay function
 * @param ms Number of milliseconds to delay
 */
void RF_DelayMs(uint32_t ms);

/**
 * @brief Write a single byte to the specified register
 * @param Addr Register address to write to
 * @param Value Single byte data to write to the register
 * @return RF_Err_t Return operation result
 */
RF_Err_t RF_WriteReg(uint8_t Addr, uint8_t Value);

/**
 * @brief Read a single byte from the specified register
 * @param Addr Register address to read from
 * @return uint8_t Value read from the register
 */
uint8_t RF_ReadReg(uint8_t Addr);

/**
 * @brief Continuously write multiple bytes to the specified register area
 * @param Addr Start address of the register area to write to
 * @param Buffer Pointer to the buffer to write to the register
 * @param Size Number of bytes to write
 */
void RF_WriteRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size);

/**
 * @brief Continuously read multiple bytes from the specified register
 * @param Addr Register address to read from
 * @param Buffer Pointer to the buffer to store read data
 * @param Size Number of bytes to read
 */
void RF_ReadRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size);

/**
 * @brief Select register page
 * @param Page Register page to select
 * @return RF_Err_t Return operation result
 */
RF_Err_t RF_SetPage(uint8_t Page);

/**
 * @brief Write a single byte to the register of the specified page
 * @param Page Register page to write to
 * @param Addr Register address to write to
 * @param Value Single byte data to write to the register
 * @return RF_Err_t Return operation result
 */
RF_Err_t RF_WritePageReg(uint8_t Page, uint8_t Addr, uint8_t Value);

/**
 * @brief Write multiple bytes to the register area of the specified page
 * @param Page Register page to write to
 * @param Addr Register address to write to
 * @param Buffer Pointer to the buffer to write to the register
 * @param Size Number of bytes to write
 */
void RF_WritePageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size);

/**
 * @brief Read a single byte from the register of the specified page
 * @param Page Register page to read from
 * @param Addr Register address to read from
 * @return uint8_t Value read from the register
 */
uint8_t RF_ReadPageReg(uint8_t Page, uint8_t Addr);

/**
 * @brief Read multiple bytes from the register area of the specified page
 * @param Page Register page to read from
 * @param Addr Register address to read from
 * @param Buffer Pointer to the buffer to store read data
 * @param Size Number of bytes to read
 */
void RF_ReadPageRegs(uint8_t Page, uint8_t Addr, uint8_t *Buffer, uint8_t Size);

/**
 * @brief Set bits of the register of the specified page
 * @param Page Register page to set
 * @param Addr Register address to set
 * @param Mask Bit mask to set
 * @return RF_Err_t Return operation result
 */
RF_Err_t RF_SetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask);

/**
 * @brief Reset bits of the register of the specified page
 * @param Page Register page to reset
 * @param Addr Register address to reset
 * @param Mask Bit mask to reset
 * @return RF_Err_t Return operation result
 */
RF_Err_t RF_ResetPageRegBits(uint8_t Page, uint8_t Addr, uint8_t Mask);

/**
 * @brief Write bits to the register of the specified page
 * @param Page Register page to write to
 * @param Addr Register address to write to
 * @param Value Value to write
 * @param Mask Bit mask to write
 * @return RF_Err_t Return operation result
 */
RF_Err_t RF_WritePageRegBits(uint8_t Page, uint8_t Addr, uint8_t Value, uint8_t Mask);

/**
 * @brief Configure GPIO mode
 * @param[in] <GpioPin> Pin number
 * @param[in] <GpioMode> GPIO mode
 * @return RF_Err_t Return operation result
 */
RF_Err_t RF_ConfigGpio(uint8_t GpioPin, uint8_t GpioMode);

/**
 * @brief Write GPIO output level
 * @param[in] <GpioPin> GPIO pin number
 * @param[in] <Level> GPIO output level
 * @return RF_Err_t Return operation result
 */
RF_Err_t RF_WriteGpioLevel(uint8_t GpioPin, uint8_t Level);

/**
 * @brief Read GPIO input level
 * @param[in] <GpioPin> GPIO pin number
 * @return GPIO input level
 */
uint8_t RF_ReadGpioLevel(uint8_t GpioPin);

/**
 * @brief Initialize PAN3029/3060 antenna control GPIO
 */
void RF_InitAntGpio(void);

/**
 * @brief Enable PAN3029/3060 transmit antenna
 */
void RF_TurnonTxAnt(void);

/**
 * @brief Enable PAN3029/3060 receive antenna
 */
void RF_TurnonRxAnt(void);

/**
 * @brief Disable PAN3029/3060 antenna
 */
void RF_ShutdownAnt(void);

/**
 * @brief Initialize TCXO control GPIO
 */
void RF_InitTcxoGpio(void);

/**
 * @brief Enable TCXO power supply
 */
void RF_TurnonTcxo(void);

/**
 * @brief Disable TCXO power supply
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
 * @brief Enable internal and external PA
 */
void RF_TurnonPA(void);

/**
 * @brief Disable internal and external PA
 */
void RF_TurnoffPA(void);

/**
 * @brief Set chip mode
 * @param[in] <ChipMode> Chip mode
 */
void RF_SetChipMode(RfChipMode_t ChipMode);

/**
 * @brief Get chip mode
 * @return RfChipMode_t Current chip mode
 */
RfChipMode_t RF_GetChipMode(void);

/**
 * @brief Initialize RF transceiver to STB3 state after power-on
 * @return Whether initialization is successful, 0 indicates success, 1 indicates failure
 */
RF_Err_t RF_Init(void);

/**
 * @brief Configure user parameters for RF chip
 */
void RF_ConfigUserParams(void);

/**
 * @brief Software reset RF chip control logic
 */
void RF_ResetLogic(void);

/**
 * @brief Get RF chip operating state
 * @return RfOpState_t Current operating state
 */
RfOpState_t RF_GetOperateState(void);

/**
 * @brief Set RF chip operating state
 * @param[in] <RfState> Operating state
 */
void RF_SetOperateState(RfOpState_t RfState);

/**
 * @brief Set RF chip operating state
 * @param[in] <RfState> Operating state
 */
void RF_SetRfState(uint8_t RfState);

/**
 * @brief Enter deep sleep mode
 * @note This function is used to put the RF chip into deep sleep mode, disable antenna power supply and TCXO power
 * @note This function sets the chip's operating state to MODE_DEEPSLEEP
 * @note After executing this function, if you need to wake up the RF chip, you need to call RF_Init() function to wake up the chip
 */
void RF_EnterDeepsleepState(void);

/**
 * @brief Enter sleep mode
 * @note This function is used to put the RF chip into sleep mode, disable antenna power supply and TCXO power
 * @note This function sets the chip's operating state to MODE_SLEEP
 * @note After executing this function, if you need to wake up the RF chip, you need to call RF_ExitSleepState() function
 */
void RF_EnterSleepState(void);

/**
 * @brief Exit sleep mode
 * @note This function is used to wake up the RF chip from sleep mode
 * @note This function sets the chip's operating state to MODE_STANDBY
 * @note After executing this function, if you need to put the RF chip into sleep mode, you need to call RF_EnterSleepState() function
 */
void RF_ExitSleepState(void);

/**
 * @brief Enter standby mode
 * @note This function is used to put the RF chip into standby mode, enable antenna power supply and TCXO power
 * @note This function sets the chip's operating state to MODE_STANDBY
 * @note After executing this function, if you need to put the RF chip into sleep mode, you need to call RF_EnterSleepState() function
 */
void RF_EnterStandbyState(void);

/**
 * @brief Check if the RF chip is in sleep mode
 * @return Whether the RF chip is in sleep mode, 0 indicates not in sleep mode, 1 indicates in sleep mode
 */
void RF_CheckDeviceReady(void);

/**
 * @brief Set regulator mode
 * @param[in] <RegulatorMode> Regulator mode
 */
void RF_SetRegulatorMode(RfRegulatorMode_t RegulatorMode);

/**
 * @brief Set RF chip frequency
 * @param[in] <Frequency> Frequency value
 */
RF_Err_t RF_SetFreq(uint32_t Frequency);

/**
 * @brief Set IQ inversion
 * @param[in] <NewState> Enable or disable IQ inversion
 */
void RF_SetInvertIQ(bool NewState);

/**
 * @brief Set preamble length
 * @param[in] <PreamLen> Preamble length value
 */
void RF_SetPreamLen(uint16_t PreamLen);

/**
 * @brief Set sync word
 * @param[in] <syncWord> Sync word value
 */
void RF_SetSyncWord(uint8_t SyncWord);

/**
 * @brief Set transmit power
 * @param[in] <TxPower> Transmit power value
 */
void RF_SetTxPower(uint8_t TxPower);

/**
 * @brief Set modulation bandwidth
 * @param[in] <BandWidth> Modulation bandwidth value
 *            - RF_BW_062K / RF_BW_125K / RF_BW_250K / RF_BW_500K
 * @note The larger the modulation bandwidth, the higher the data rate, but the shorter the transmission distance
 * @note The modulation bandwidth range of PAN3029 is RF_BW_062K - RF_BW_500K
 * @note The modulation bandwidth range of PAN3060 is RF_BW_125K - RF_BW_500K
 */
void RF_SetBW(uint8_t BandWidth);

/**
 * @brief Set spreading factor
 * @param[in] <SpreadFactor> Spreading factor value
 *          - RF_SF5 / RF_SF6 / RF_SF7 / RF_SF8 / RF_SF9 / RF_SF10 / RF_SF11 / RF_SF12
 * @note The larger the spreading factor, the longer the transmission distance, but the lower the data rate
 * @note The spreading factor range of PAN3029 is RF_SF5 - RF_SF12
 * @note The spreading factor range of PAN3060 is RF_SF5 - RF_SF9
 */
void RF_SetSF(uint8_t SpreadFactor);

/**
 * @brief Set channel coding rate
 * @param[in] <CodingRate> Channel coding rate value
 */
void RF_SetCR(uint8_t CodingRate);

/**
 * @brief Set CRC check
 * @param[in] <NewState> Enable or disable CRC check
 */
void RF_SetCRC(uint8_t CrcMode);

/**
 * @brief Set low data rate mode
 * @param[in] <LdrMode> Low data rate mode value
 */
void RF_SetLDR(uint8_t LdrMode);

/**
 * @brief set modem mode
 * @param[in] <modem_mode>
 */
void RF_SetModemMode(uint8_t ModemMode);

/**
 * @brief Set transmit mode
 * @param Buffer Data buffer to be sent
 * @param Size Number of data bytes to be sent
 */
void RF_SetTx(uint8_t *Buffer, uint8_t Size);

/**
 * @brief Set receive mode
 * @param TimeoutMs Receive timeout time
 */
void RF_SetRx(uint32_t TimeoutMs);

/**
 * @brief CAD function enable
 * @param[in] <Threshold>
 */
void RF_StartCad(uint8_t Threshold, uint8_t Chirps);

/**
 * @brief Set CAD detection threshold
 * @param[in] <Threshold> CAD detection threshold
 *            - RF_CAD_THRESHOLD_0A
 *            - RF_CAD_THRESHOLD_10
 *            - RF_CAD_THRESHOLD_15
 *            - RF_CAD_THRESHOLD_20
 */
void RF_SetCadThreshold(uint8_t Threshold);

/**
 * @brief Set CAD detection symbol number
 * @param[in] <Chirps> CAD detection symbol number
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
 * @brief Set transmit mode
 * @param[in] <TxMode>
 */
void RF_SetTxMode(uint8_t TxMode);

/**
 * @brief Send a single data packet
 * @param[in] <Buffer> Data buffer to be sent
 * @param[in] <Size> Number of data bytes to be sent
 */
void RF_TxSinglePkt(uint8_t *Buffer, uint8_t Size);

/**
 * @brief Set receive mode
 * @param[in] <RxMode> Receive mode
 */
void RF_SetRxMode(uint8_t RxMode);

/**
 * @brief Make the chip enter continuous reception state
 */
void RF_EnterContinousRxState(void);

/**
 * @brief Set receive timeout time
 * @param[in] <TimeoutMs> Timeout time
 */
void RF_SetRxTimeout(uint16_t TimeoutMs);

/**
 * @brief Make the chip enter single reception with timeout state
 * @param[in] <TimeoutMs> Timeout time
 */
void RF_EnterSingleRxWithTimeout(uint16_t TimeoutMs);

/**
 * @brief Get receive data length
 * @return Receive data length
 */
uint8_t RF_GetRxPayloadLen(void);

/**
 * @brief Function used to get receive data length and content
 * @param *Buffer Pointer address of the data area to receive data
 * @return Length of received data
 */
uint8_t RF_GetRecvPayload(uint8_t *Buffer);

/**
 * @brief Get RSSI value of received data packet
 * @return RSSI value
 */
int8_t RF_GetPktRssi(void);

/**
 * @brief Get real-time RSSI value
 * @return RSSI value
 */
int8_t RF_GetRealTimeRssi(void);

/**
 * @brief Get SNR value of received data packet
 * @return SNR value
 */
int32_t RF_GetPktSnr(void);

/**
 * @brief Get interrupt flag bits
 * @return IRQ flag bits
 */
uint8_t RF_GetIRQFlag(void);

/**
 * @brief Clear interrupt flag bits
 * @param[in] <IRQFlag> Interrupt flag bits
 */
void RF_ClrIRQFlag(uint8_t IRQFlag);

/**
 * @brief Get current frequency setting
 */
uint32_t RF_GetFreq(void);

/**
 * @brief Get current IQ inversion setting
 */
RfIQModes_t RF_GetInvertIQ(void);

/**
 * @brief Get current preamble length setting
 */
uint16_t RF_GetPreamLen(void);

/**
 * @brief Get current transmit power setting
 */
uint8_t RF_GetTxPower(void);

/**
 * @brief Get current bandwidth setting
 */
uint8_t RF_GetBandWidth(void);

/**
 * @brief Get current spreading factor setting
 */
uint8_t RF_GetSF(void);

/**
 * @brief Get current CRC check setting
 */
uint8_t RF_GetCRC(void);

/**
 * @brief Get current coding rate setting
 */
uint8_t RF_GetCR(void);

/**
 * @brief Get current sync word setting
 */
uint8_t RF_GetSyncWord(void);

/**
 * @brief Get current low data rate mode setting
 */
uint8_t RF_GetLDR(void);

/**
 * @brief Get the time of a single symbol
 * @param[in] <bw> Bandwidth
 *           - RF_BW_062K / RF_BW_125K / RF_BW_250K / RF_BW_500K
 * @param[in] <sf> Spreading factor
 *           - RF_SF5 / RF_SF6 / RF_SF7 / RF_SF8 / RF_SF9 / RF_SF10 / RF_SF11 / RF_SF12
 * @return Time of a single symbol, unit is us
 * @note This function is used to calculate the time of a single symbol
 */
uint32_t RF_GetOneSymbolTime(uint8_t bw, uint8_t sf);

/**
 * @brief Calculate the time to send a data packet
 * @param[in] <Size> Size of the data packet to be sent, unit is bytes
 * @return Time to send the data packet, unit is ms
 */
uint32_t RF_GetTxTimeMs(uint8_t Size);

/**
 * @brief Enable mapm mode
 */
void RF_EnableMapm(void);

/**
 * @brief Disable mapm mode
 */
void RF_DisableMapm(void);

/**
 * @brief Configure mapm related parameters
 * @param[in] <pMapmCfg>
 */
void RF_ConfigMapm(RF_MapmCfg_t *pMapmCfg);

/**
 * @brief Set group address in mapm mode
 * @param[in] <MapmAddr> mapm group address
 *            <AddrWidth> Address width, range is 1~4
 * @note The MapmAddr[0] of the receiving end must be consistent with the MapmAddr[0] of the transmitting end,
 *       otherwise the receiving end will not trigger mapm interrupt.
 */
void RF_SetMapmAddr(uint8_t *MapmAddr, uint8_t AddrWidth);

/**
 * @brief Calculate the time spent on 1 field (ms)
 * @param[in] <pMapmCfg> mapm configuration parameters
 *            <SymbolTime> Time of a single symbol (chirp)
 * @note The number of chirps in Group1 is (pg1 + 2), where pg1 is the number of preambles in Group1, and 2 is the number of chirps occupied by the address in Group1.
 * @note The number of chirps in other Groups is (pgn + 2)*(gn-1), where pgn is the number of preambles in other single Groups,
 *       2 is the number of chirps occupied by the address (or count value) in other single Groups, and (gn-1) is the number of remaining Groups after removing Group1.
 */
uint32_t RF_GetMapmOneFieldTime(RF_MapmCfg_t *pMapmCfg, uint32_t SymbolTime);

/**
 * @brief Get remaining Mapm time in mapm mode
 * @param[in] <pMapmCfg> mapm configuration parameters
 * @param[in] <SynbolTime> Time of a single chirp
 * @return Remaining Mapm time, unit is ms
 */
uint32_t RF_GetLeftMapmTime(RF_MapmCfg_t *pMapmCfg, uint32_t SynbolTime);

/**
 * @brief Start transmitting continuous carrier
 */
void RF_StartTxContinuousWave(void);

/**
 * @brief Stop transmitting continuous carrier
 */
void RF_StopTxContinuousWave(void);

/**
 * @brief Process RF interrupts
 * @note This function will be called in the interrupt service function
 */
void RF_IRQ_Process(void);

/** \} defgroup PAN3029/3060 */
/** \} addtogroup ChirpIOT */

#endif // __PAN_RF_H__
