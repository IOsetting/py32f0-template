/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bmp180.h
* @date 10/01/2020
* @version  2.2.5
*
*/

/** \file bmp180.h
 * \brief Header file for all define constants and function prototypes
 */
#ifndef __BMP180_H__
#define __BMP180_H__

/*!
 * @brief The following definition uses for define the data types
 *
 * @note While porting the API please consider the following
 * @note Please check the version of C standard
 * @note Are you using Linux platform
 */

/*!
 * @brief For the Linux platform support
 * Please use the types.h for your data types definitions
 */
#ifdef  __KERNEL__

#include <linux/types.h>

/* singed integer type*/
typedef int8_t s8; /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */
typedef u_int8_t u8; /**< used for unsigned 8bit */
typedef u_int16_t u16; /**< used for unsigned 16bit */
typedef u_int32_t u32; /**< used for unsigned 32bit */
typedef u_int64_t u64; /**< used for unsigned 64bit */

#else /* ! __KERNEL__ */

/**********************************************************
 * These definition uses for define the C
 * standard version data types
 ***********************************************************/
# if !defined(__STDC_VERSION__)

/************************************************
* compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>

/************************************************/
/*unsigned integer types*/
typedef uint8_t u8; /**< used for unsigned 8bit */
typedef uint16_t u16; /**< used for unsigned 16bit */
typedef uint32_t u32; /**< used for unsigned 32bit */
typedef uint64_t u64; /**< used for unsigned 64bit */
/*signed integer types*/
typedef int8_t s8; /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */

/************************************************
* compiler is C99 C standard
************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
 *  which is used to fixed the integer size*/

/************************************************/
#include <stdint.h>

/************************************************/
/*unsigned integer types*/
typedef uint8_t u8; /**< used for unsigned 8bit */
typedef uint16_t u16; /**< used for unsigned 16bit */
typedef uint32_t u32; /**< used for unsigned 32bit */
typedef uint64_t u64; /**< used for unsigned 64bit */
/*signed integer types*/
typedef int8_t s8; /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */

/************************************************
* compiler is C89 or other C standard
************************************************/

#else /*  !defined(__STDC_VERSION__) */

/*!
 * @brief By default it is defined as 32 bit machine configuration
 *  define your data types based on your
 *  machine/compiler/controller configuration
 */
#define  MACHINE_32_BIT

/*! @brief
 *  If your machine support 16 bit
 *  define the MACHINE_16_BIT
 */
#ifdef MACHINE_16_BIT
#include <limits.h>

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed long int s32; /**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int s64; /**< used for signed 64bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int s64; /**< used for signed 64bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
    could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
    please set s64 manually.
#endif

/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned long int u32; /**< used for unsigned 32bit */

/* If your machine support 32 bit
 * define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32; /**< used for signed 32bit */
typedef signed long long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32; /**< used for unsigned 32bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */

/* If your machine support 64 bit
 * define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32; /**< used for signed 32bit */
typedef signed long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32; /**< used for unsigned 32bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
    define the data types manually
#endif
#endif

/*** This else will execute for the compilers
 *  which are not supported the C standards
 *  Like C89/C99/C11***/
#else

/*!
 * @brief By default it is defined as 32 bit machine configuration
 *  define your data types based on your
 *  machine/compiler/controller configuration
 */
#define  MACHINE_32_BIT

/* If your machine support 16 bit
 * define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed long int s32; /**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int s64; /**< used for signed 64bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int s64; /**< used for signed 64bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
    could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
    please set s64 manually.
#endif

/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned long int u32; /**< used for unsigned 32bit */

/*! @brief If your machine support 32 bit
 * define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed int s16; /**< used for signed 16bit */
typedef signed long int s32; /**< used for signed 32bit */
typedef signed long long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned int u16; /**< used for unsigned 16bit */
typedef unsigned long int u32; /**< used for unsigned 32bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */

/* If your machine support 64 bit
 * define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT

/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32; /**< used for signed 32bit */
typedef signed long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32; /**< used for unsigned 32bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
    define the data types manually
#endif
#endif
#endif

/***************************************************************/
/**\name    FUNCTION DEFINITIONS      */
/***************************************************************/
#define bmp180_calc_temperature(ut) \
    bmp180_get_temperature(ut)

#define bmp180_calc_pressure(up) \
    bmp180_get_pressure(up)

#define bmp180_read_ut() \
    bmp180_get_ut()

#define bmp180_read_up() \
    bmp180_get_up()

#define bmp180_read_cal_param() \
    bmp180_get_cal_param()

#define smd500_read_cal_param() \
    smd500_get_cal_param()

/***************************************************************/
/**\name    BUS READ AND WRITE FUNCTION        */
/***************************************************************/

/*!
 * @brief Define the calling convention of YOUR bus communication routine.
 * @note This includes types of parameters. This example shows the
 * configuration for an SPI bus link.
 *
 * If your communication function looks like this:
 *
 * write_my_bus_xy(u8 device_addr, u8 register_addr,
 * u8 * data, u8 length);
 *
 * The BMP180_WR_FUNC_PTR would equal:
 *
 * BMP180_WR_FUNC_PTR s8 (* bus_write)(u8,
 * u8, u8 *, u8)
 *
 * Parameters can be mixed as needed refer to the
 * refer BMP180_BUS_WRITE_FUNC  macro.
 *
 *
 */
#define BMP180_BUS_WR_RETURN_TYPE s8

/** defines the calling parameter types of the BMP180_WR_FUNCTION
 *
 */
#define BMP180_BUS_WR_PARAM_TYPES u8, u8, \
    u8 *, u8

/** links the order of parameters defined in
 * BMP180_BUS_WR_PARAM_TYPE to function calls used inside the API
 *
 */
#define BMP180_BUS_WR_PARAM_ORDER (device_addr, register_addr, \
                                   register_data, write_length)

/* never change this line */
#define BMP180_BUS_WRITE_FUNC(device_addr, register_addr, \
                              register_data, write_length) \
    bus_write(device_addr, register_addr, register_data, write_length)

/*!
 * @brief link macro between API function calls and bus read function
 * @note The bus write function can change since this is a
 * system dependant issue.
 *
 * If the bus_read parameter calling order is like: reg_addr,
 * reg_data, wr_len it would be as it is here.
 *
 * If the parameters are differently ordered or your communication
 * function like I2C need to know the device address,
 * you can change this macro accordingly.
 *
 *
 * BMP180_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
 * bus_read(dev_addr, reg_addr, reg_data, wr_len)
 *
 * This macro lets all API functions call YOUR communication routine in a
 * way that equals your definition in the
 * refer BMP180_WR_FUNC_PTR definition.
 *
 * @note: this macro also includes the "MSB='1'
 * for reading BMP180 addresses.
 *
 */

/** defines the return parameter type of the BMP180_WR_FUNCTION
 */
#define BMP180_BUS_RD_RETURN_TYPE s8

/** defines the calling parameter types of the BMP180_WR_FUNCTION
 */
#define BMP180_BUS_RD_PARAM_TYPES (u8, \
                                   u8, u8 *, u8)

/** links the order of parameters defined in
 * BMP180_BUS_WR_PARAM_TYPE to function calls used inside the API
 *
 */
#define BMP180_BUS_RD_PARAM_ORDER (device_addr, \
                                   register_addr, register_data, read_length)

/* never change this line */
#define BMP180_BUS_READ_FUNC(device_addr, register_addr, \
                             register_data, read_length) \
    bus_read(device_addr, register_addr, register_data, read_length)

/***************************************************************/
/**\name    BUS READ AND WRITE FUNCTION POINTERS  DEFINITIONS      */
/***************************************************************/
#define BMP180_WR_FUNC_PTR \
    s8 (*bus_write)(u8, u8, u8 *, u8)

#define BMP180_RD_FUNC_PTR \
    s8 (*bus_read)(u8, u8, u8 *, u8)

/* register write and read delays */
#define BMP180_MDELAY_DATA_TYPE u32

/***************************************************************/
/**\name    I2C ADDRESS DEFINITION OF BMP180       */
/***************************************************************/
/*BMP180 I2C Address*/
#define BMP180_I2C_ADDR                           (0xEF)

/***************************************************************/
/**\name    ERROR CODE DEFINITIONS    */
/***************************************************************/
#define E_BMP_NULL_PTR                            ((s8) - 127)
#define E_BMP_COMM_RES                            ((s8) - 1)
#define E_BMP_OUT_OF_RANGE                        ((s8) - 2)

/***************************************************************/
/**\name    CONSTANTS       */
/***************************************************************/
#define BMP180_RETURN_FUNCTION_TYPE               s8
#define   BMP180_INIT_VALUE                       ((u8)0)
#define   BMP180_INITIALIZE_OVERSAMP_SETTING_U8X  ((u8)0)
#define   BMP180_INITIALIZE_SW_OVERSAMP_U8X       ((u8)0)
#define   BMP180_INITIALIZE_NUMBER_OF_SAMPLES_U8X ((u8)1)
#define   BMP180_GEN_READ_WRITE_DATA_LENGTH       ((u8)1)
#define   BMP180_TEMPERATURE_DATA_LENGTH          ((u8)2)
#define   BMP180_PRESSURE_DATA_LENGTH             ((u8)3)
#define   BMP180_SW_OVERSAMP_U8X                  ((u8)1)
#define   BMP180_OVERSAMP_SETTING_U8X             ((u8)3)
#define   BMP180_2MS_DELAY_U8X                    (2)
#define   BMP180_3MS_DELAY_U8X                    (3)
#define   BMP180_AVERAGE_U8X                      (3)
#define   BMP180_INVALID_DATA                     (0)
#define   BMP180_CHECK_DIVISOR                    (0)
#define   BMP180_DATA_MEASURE                     (3)
#define   BMP180_CALCULATE_TRUE_PRESSURE          (8)
#define   BMP180_CALCULATE_TRUE_TEMPERATURE       (8)
#define BMP180_SHIFT_BIT_POSITION_BY_01_BIT       (1)
#define BMP180_SHIFT_BIT_POSITION_BY_02_BITS      (2)
#define BMP180_SHIFT_BIT_POSITION_BY_04_BITS      (4)
#define BMP180_SHIFT_BIT_POSITION_BY_06_BITS      (6)
#define BMP180_SHIFT_BIT_POSITION_BY_08_BITS      (8)
#define BMP180_SHIFT_BIT_POSITION_BY_11_BITS      (11)
#define BMP180_SHIFT_BIT_POSITION_BY_12_BITS      (12)
#define BMP180_SHIFT_BIT_POSITION_BY_13_BITS      (13)
#define BMP180_SHIFT_BIT_POSITION_BY_15_BITS      (15)
#define BMP180_SHIFT_BIT_POSITION_BY_16_BITS      (16)

/***************************************************************/
/**\name    REGISTER ADDRESS DEFINITION       */
/***************************************************************/
/*register definitions */

#define BMP180_PROM_START__ADDR       (0xAA)
#define BMP180_PROM_DATA__LEN         (22)

#define BMP180_CHIP_ID_REG            (0xD0)
#define BMP180_VERSION_REG            (0xD1)

#define BMP180_CTRL_MEAS_REG          (0xF4)
#define BMP180_ADC_OUT_MSB_REG        (0xF6)
#define BMP180_ADC_OUT_LSB_REG        (0xF7)

#define BMP180_SOFT_RESET_REG         (0xE0)

/* temperature measurement */
#define BMP180_T_MEASURE              (0x2E)

/* pressure measurement*/
#define BMP180_P_MEASURE              (0x34)

/* TO be spec'd by GL or SB*/
#define BMP180_TEMP_CONVERSION_TIME   (5)

#define BMP180_PARAM_MG               (3038)
#define BMP180_PARAM_MH               (-7357)
#define BMP180_PARAM_MI               (3791)

/****************************************************/
/**\name    ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define BMP180_TEMPERATURE_DATA_BYTES (2)
#define BMP180_PRESSURE_DATA_BYTES    (3)
#define BMP180_TEMPERATURE_LSB_DATA   (1)
#define BMP180_TEMPERATURE_MSB_DATA   (0)
#define BMP180_PRESSURE_MSB_DATA      (0)
#define BMP180_PRESSURE_LSB_DATA      (1)
#define BMP180_PRESSURE_XLSB_DATA     (2)

#define BMP180_CALIB_DATA_SIZE        (22)
#define BMP180_CALIB_PARAM_AC1_MSB    (0)
#define BMP180_CALIB_PARAM_AC1_LSB    (1)
#define BMP180_CALIB_PARAM_AC2_MSB    (2)
#define BMP180_CALIB_PARAM_AC2_LSB    (3)
#define BMP180_CALIB_PARAM_AC3_MSB    (4)
#define BMP180_CALIB_PARAM_AC3_LSB    (5)
#define BMP180_CALIB_PARAM_AC4_MSB    (6)
#define BMP180_CALIB_PARAM_AC4_LSB    (7)
#define BMP180_CALIB_PARAM_AC5_MSB    (8)
#define BMP180_CALIB_PARAM_AC5_LSB    (9)
#define BMP180_CALIB_PARAM_AC6_MSB    (10)
#define BMP180_CALIB_PARAM_AC6_LSB    (11)
#define BMP180_CALIB_PARAM_B1_MSB     (12)
#define BMP180_CALIB_PARAM_B1_LSB     (13)
#define BMP180_CALIB_PARAM_B2_MSB     (14)
#define BMP180_CALIB_PARAM_B2_LSB     (15)
#define BMP180_CALIB_PARAM_MB_MSB     (16)
#define BMP180_CALIB_PARAM_MB_LSB     (17)
#define BMP180_CALIB_PARAM_MC_MSB     (18)
#define BMP180_CALIB_PARAM_MC_LSB     (19)
#define BMP180_CALIB_PARAM_MD_MSB     (20)
#define BMP180_CALIB_PARAM_MD_LSB     (21)

/**************************************************************/
/**\name    STRUCTURE DEFINITIONS                         */
/**************************************************************/

/*!
 * @brief This structure holds all device specific calibration parameters
 */
struct bmp180_calib_param_t
{
    s16 ac1; /**<calibration ac1 data*/
    s16 ac2; /**<calibration ac2 data*/
    s16 ac3; /**<calibration ac3 data*/
    u16 ac4; /**<calibration ac4 data*/
    u16 ac5; /**<calibration ac5 data*/
    u16 ac6; /**<calibration ac6 data*/
    s16 b1; /**<calibration b1 data*/
    s16 b2; /**<calibration b2 data*/
    s16 mb; /**<calibration mb data*/
    s16 mc; /**<calibration mc data*/
    s16 md; /**<calibration md data*/
};

/*!
 * @brief This structure holds BMP180 initialization parameters
 */
struct bmp180_t
{
    struct bmp180_calib_param_t calib_param; /**<calibration data*/
    u8 mode; /**<power mode*/
    u8 chip_id; /**<chip id*/
    u8 ml_version; /**<ml version*/
    u8 al_version; /**<al version*/
    u8 dev_addr; /**<device address*/
    u8 sensortype; /**< sensor type*/
    s32 param_b5; /**<pram*/
    s32 number_of_samples; /**<sample calculation*/
    s16 oversamp_setting; /**<oversampling setting*/
    s16 sw_oversamp; /**<software oversampling*/
    BMP180_WR_FUNC_PTR; /**< bus write function pointer*/
    BMP180_RD_FUNC_PTR; /**< bus read function pointer*/
    void (*delay_msec)(BMP180_MDELAY_DATA_TYPE); /**< delay function pointer*/
};

/**************************************************************/
/**\name    BIT MASK, LENGTH AND POSITION FOR REGISTERS     */
/**************************************************************/
/**************************************************************/

/**\name    BIT MASK, LENGTH AND POSITION FOR
 * CHIP ID REGISTERS     */

/**************************************************************/
#define BMP180_CHIP_ID__POS (0)
#define BMP180_CHIP_ID__MSK (0xFF)
#define BMP180_CHIP_ID__LEN (8)
#define BMP180_CHIP_ID__REG (BMP180_CHIP_ID_REG)

/**************************************************************/

/**\name    BIT MASK, LENGTH AND POSITION FOR
 * ML VERSION  */

/**************************************************************/
#define BMP180_ML_VERSION__POS (0)
#define BMP180_ML_VERSION__LEN (4)
#define BMP180_ML_VERSION__MSK (0x0F)
#define BMP180_ML_VERSION__REG (BMP180_VERSION_REG)

/**************************************************************/

/**\name    BIT MASK, LENGTH AND POSITION FOR
 * AL VERSION  */

/**************************************************************/
#define BMP180_AL_VERSION__POS (4)
#define BMP180_AL_VERSION__LEN (4)
#define BMP180_AL_VERSION__MSK (0xF0)
#define BMP180_AL_VERSION__REG (BMP180_VERSION_REG)

/**************************************************************/
/**\name    GET AND SET BITSLICE FUNCTIONS*/
/**************************************************************/
/* DATA REGISTERS */
/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */
#define BMP180_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##__MSK) >> (bitname##__POS))

#define BMP180_SET_BITSLICE(regvar, bitname, val) \
    ((regvar & ~bitname##__MSK) | ((val << bitname##__POS) & bitname##__MSK))

/**************************************************************/
/**\name    FUNCTION DECLARATIONS */
/**************************************************************/
/**************************************************************/
/**\name    FUNCTION FOR INTIALIZATION */
/**************************************************************/

/*!
 *  @brief This function is used for initialize
 *  the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP180
 *  chip id is read in the register 0xD0 bit from 0 to 7
 *
 *   @param bmp180 structure pointer.
 *
 *  @note While changing the parameter of the bmp180_t
 *  @note consider the following point:
 *  Changing the reference value of the parameter
 *  will changes the local copy or local reference
 *  make sure your changes will not
 *  affect the reference value of the parameter
 *  (Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval -1 -> Error
 *
 *
 */
BMP180_RETURN_FUNCTION_TYPE bmp180_init(struct bmp180_t *bmp180);

/**************************************************************/
/**\name    FUNCTION FOR TEMPERATURE AND PRESSURE READ */
/**************************************************************/

/*!
 *  @brief this API is used to calculate the true
 *  temperature using the uncompensated temperature(ut)
 *  @note For reading the ut data refer : bmp180_read_ut()
 *
 *  @param v_uncomp_temperature_u32:
 *  the value of uncompensated temperature
 *
 *  @return Return the temperature in steps of 0.1 deg Celsius
 *
 *
 */
s16 bmp180_get_temperature(u32 v_uncomp_temperature_u32);

/*!
 *  @brief this API is used to calculate the true
 *  pressure using the uncompensated pressure(up)
 *  @note For reading the up data refer : bmp180_read_up()
 *
 *  @param v_uncomp_pressure_u32: the value of uncompensated pressure
 *
 *  @return Return the value of pressure in steps of 1.0 Pa
 *
 */
s32 bmp180_get_pressure(u32 v_uncomp_pressure_u32);

/**************************************************************/
/**\name    FUNCTION FOR UNCOMPENSATED PRESSURE AND TEMPERATURE */
/**************************************************************/

/*!
 *  @brief this API is used to read the
 *  uncompensated temperature(ut) from the register
 *  @note 0xF6(MSB) bit from 0 to 7
 *  @note 0xF7(LSB) bit from 0 to 7
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval -1 -> Error
 *
 *
 */
u16 bmp180_get_uncomp_temperature(void);

/*!
 *  @brief this API is used to read the
 *  uncompensated pressure(up) from the register
 *  @note 0xF6(MSB) bit from 0 to 7
 *  @note 0xF7(LSB) bit from 0 to 7
 *  @note 0xF8(LSB) bit from 3 to 7
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval -1 -> Error
 *
 */
u32  bmp180_get_uncomp_pressure(void);

/**************************************************************/
/**\name    FUNCTION FOR CALIBRATION */
/**************************************************************/

/*!
 *  @brief this function used for read the calibration
 *  parameter from the register
 *
 *  Parameter  |  MSB    |  LSB    |  bit
 * ------------|---------|---------|-----------
 *      AC1    |  0xAA   | 0xAB    | 0 to 7
 *      AC2    |  0xAC   | 0xAD    | 0 to 7
 *      AC3    |  0xAE   | 0xAF    | 0 to 7
 *      AC4    |  0xB0   | 0xB1    | 0 to 7
 *      AC5    |  0xB2   | 0xB3    | 0 to 7
 *      AC6    |  0xB4   | 0xB5    | 0 to 7
 *      B1     |  0xB6   | 0xB7    | 0 to 7
 *      B2     |  0xB8   | 0xB9    | 0 to 7
 *      MB     |  0xBA   | 0xBB    | 0 to 7
 *      MC     |  0xBC   | 0xBD    | 0 to 7
 *      MD     | 0xBE    | 0xBF    | 0 to 7
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval -1 -> Error
 *
 *
 */
BMP180_RETURN_FUNCTION_TYPE bmp180_get_calib_param(void);

/* __BMP180_H__*/
#endif
