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
* @file bmp180.c
* @date 10/01/2020
* @version  2.2.5
*
*/

#include "bmp180.h"
static struct bmp180_t *p_bmp180;

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
BMP180_RETURN_FUNCTION_TYPE bmp180_init(struct bmp180_t *bmp180)
{
    /* used to return the bus communication results*/
    BMP180_RETURN_FUNCTION_TYPE v_com_rslt_s8 = E_BMP_COMM_RES;
    u8 v_data_u8 = BMP180_INIT_VALUE;

    /* assign BMP180 ptr */
    p_bmp180 = bmp180;

    /* read Chip Id */
    v_com_rslt_s8 = p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,
                                                   BMP180_CHIP_ID__REG,
                                                   &v_data_u8,
                                                   BMP180_GEN_READ_WRITE_DATA_LENGTH);
    p_bmp180->chip_id = BMP180_GET_BITSLICE(v_data_u8, BMP180_CHIP_ID);
    p_bmp180->number_of_samples = BMP180_INITIALIZE_NUMBER_OF_SAMPLES_U8X;
    p_bmp180->oversamp_setting = BMP180_INITIALIZE_OVERSAMP_SETTING_U8X;
    p_bmp180->sw_oversamp = BMP180_INITIALIZE_SW_OVERSAMP_U8X;
    v_com_rslt_s8 += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,
                                                    BMP180_VERSION_REG,
                                                    &v_data_u8,
                                                    BMP180_GEN_READ_WRITE_DATA_LENGTH);

    /* read Version reg */
    p_bmp180->ml_version = BMP180_GET_BITSLICE(v_data_u8, BMP180_ML_VERSION); /* get ML version */
    p_bmp180->al_version = BMP180_GET_BITSLICE(v_data_u8, BMP180_AL_VERSION); /* get AL version */
    v_com_rslt_s8 += bmp180_get_calib_param();

    return v_com_rslt_s8;
}

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
BMP180_RETURN_FUNCTION_TYPE bmp180_get_calib_param(void)
{
    /* used to return the bus communication results*/
    BMP180_RETURN_FUNCTION_TYPE v_com_rslt_s8 = E_BMP_COMM_RES;

    /* Array holding the calibration informations*/
    u8 a_data_u8r[BMP180_CALIB_DATA_SIZE] = {
        BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE,
        BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE,
        BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE,
        BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE,
        BMP180_INIT_VALUE, BMP180_INIT_VALUE
    };

    /* read calibration data*/
    v_com_rslt_s8 = p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,
                                                   BMP180_PROM_START__ADDR,
                                                   a_data_u8r,
                                                   BMP180_PROM_DATA__LEN);

    /*parameters AC1-AC6*/
    p_bmp180->calib_param.ac1 =
        (s16)((((s32)((s8)a_data_u8r[BMP180_CALIB_PARAM_AC1_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_AC1_LSB]);
    p_bmp180->calib_param.ac2 =
        (s16)((((s32)((s8)a_data_u8r[BMP180_CALIB_PARAM_AC2_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_AC2_LSB]);
    p_bmp180->calib_param.ac3 =
        (s16)((((s32)((s8)a_data_u8r[BMP180_CALIB_PARAM_AC3_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_AC3_LSB]);
    p_bmp180->calib_param.ac4 =
        (u16)((((u32)((u8)a_data_u8r[BMP180_CALIB_PARAM_AC4_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_AC4_LSB]);
    p_bmp180->calib_param.ac5 =
        (u16)((((u32)((u8)a_data_u8r[BMP180_CALIB_PARAM_AC5_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_AC5_LSB]);
    p_bmp180->calib_param.ac6 =
        (u16)((((u32)((u8)a_data_u8r[BMP180_CALIB_PARAM_AC6_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_AC6_LSB]);

    /*parameters B1,B2*/
    p_bmp180->calib_param.b1 =
        (s16)((((s32)((s8)a_data_u8r[BMP180_CALIB_PARAM_B1_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_B1_LSB]);
    p_bmp180->calib_param.b2 =
        (s16)((((s32)((s8)a_data_u8r[BMP180_CALIB_PARAM_B2_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_B2_LSB]);

    /*parameters MB,MC,MD*/
    p_bmp180->calib_param.mb =
        (s16)((((s32)((s8)a_data_u8r[BMP180_CALIB_PARAM_MB_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_MB_LSB]);
    p_bmp180->calib_param.mc =
        (s16)((((s32)((s8)a_data_u8r[BMP180_CALIB_PARAM_MC_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_MC_LSB]);
    p_bmp180->calib_param.md =
        (s16)((((s32)((s8)a_data_u8r[BMP180_CALIB_PARAM_MD_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              a_data_u8r[BMP180_CALIB_PARAM_MD_LSB]);

    return v_com_rslt_s8;
}

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
s16 bmp180_get_temperature(u32 v_uncomp_temperature_u32)
{
    s16 v_temperature_s16 = BMP180_INIT_VALUE;
    s32 v_x1_s32, v_x2_s32 = BMP180_INIT_VALUE;

    /* calculate temperature*/
    v_x1_s32 = (((s32) v_uncomp_temperature_u32 - (s32) p_bmp180->calib_param.ac6) * (s32) p_bmp180->calib_param.ac5) >>
               BMP180_SHIFT_BIT_POSITION_BY_15_BITS;
    if (v_x1_s32 == BMP180_CHECK_DIVISOR && p_bmp180->calib_param.md == BMP180_CHECK_DIVISOR)
    {
        return BMP180_INVALID_DATA;
    }

    /* executed only the divisor is not zero*/
    v_x2_s32 = ((s32) p_bmp180->calib_param.mc << BMP180_SHIFT_BIT_POSITION_BY_11_BITS) /
               (v_x1_s32 + p_bmp180->calib_param.md);
    p_bmp180->param_b5 = v_x1_s32 + v_x2_s32;
    v_temperature_s16 =
        ((p_bmp180->param_b5 + BMP180_CALCULATE_TRUE_TEMPERATURE) >> BMP180_SHIFT_BIT_POSITION_BY_04_BITS);

    return v_temperature_s16;
}

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
s32 bmp180_get_pressure(u32 v_uncomp_pressure_u32)
{
    s32 v_pressure_s32, v_x1_s32, v_x2_s32, v_x3_s32, v_b3_s32, v_b6_s32 = BMP180_INIT_VALUE;
    u32 v_b4_u32, v_b7_u32 = BMP180_INIT_VALUE;

    v_b6_s32 = p_bmp180->param_b5 - 4000;

    /*****calculate B3************/
    v_x1_s32 = (v_b6_s32 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_12_BITS;
    v_x1_s32 *= p_bmp180->calib_param.b2;
    v_x1_s32 >>= BMP180_SHIFT_BIT_POSITION_BY_11_BITS;
    v_x2_s32 = (p_bmp180->calib_param.ac2 * v_b6_s32);
    v_x2_s32 >>= BMP180_SHIFT_BIT_POSITION_BY_11_BITS;
    v_x3_s32 = v_x1_s32 + v_x2_s32;
    v_b3_s32 = (((((s32)p_bmp180->calib_param.ac1) * 4 + v_x3_s32) << p_bmp180->oversamp_setting) + 2) >>
               BMP180_SHIFT_BIT_POSITION_BY_02_BITS;

    /*****calculate B4************/
    v_x1_s32 = (p_bmp180->calib_param.ac3 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_13_BITS;
    v_x2_s32 = (p_bmp180->calib_param.b1 * ((v_b6_s32 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_12_BITS)) >>
               BMP180_SHIFT_BIT_POSITION_BY_16_BITS;
    v_x3_s32 = ((v_x1_s32 + v_x2_s32) + 2) >> BMP180_SHIFT_BIT_POSITION_BY_02_BITS;
    v_b4_u32 = (p_bmp180->calib_param.ac4 * (u32)(v_x3_s32 + 32768)) >> BMP180_SHIFT_BIT_POSITION_BY_15_BITS;
    v_b7_u32 = ((u32)(v_uncomp_pressure_u32 - v_b3_s32) * (50000 >> p_bmp180->oversamp_setting));
    if (v_b7_u32 < 0x80000000)
    {
        if (v_b4_u32 != BMP180_CHECK_DIVISOR)
        {
            v_pressure_s32 = (v_b7_u32 << BMP180_SHIFT_BIT_POSITION_BY_01_BIT) / v_b4_u32;
        }
        else
        {
            return BMP180_INVALID_DATA;
        }
    }
    else
    {
        if (v_b4_u32 != BMP180_CHECK_DIVISOR)
        {
            v_pressure_s32 = (v_b7_u32 / v_b4_u32) << BMP180_SHIFT_BIT_POSITION_BY_01_BIT;
        }
        else
        {
            return BMP180_INVALID_DATA;
        }
    }
    v_x1_s32 = v_pressure_s32 >> BMP180_SHIFT_BIT_POSITION_BY_08_BITS;
    v_x1_s32 *= v_x1_s32;
    v_x1_s32 = (v_x1_s32 * BMP180_PARAM_MG) >> BMP180_SHIFT_BIT_POSITION_BY_16_BITS;
    v_x2_s32 = (v_pressure_s32 * BMP180_PARAM_MH) >> BMP180_SHIFT_BIT_POSITION_BY_16_BITS;

    /*pressure in Pa*/
    v_pressure_s32 += (v_x1_s32 + v_x2_s32 + BMP180_PARAM_MI) >> BMP180_SHIFT_BIT_POSITION_BY_04_BITS;

    return v_pressure_s32;
}

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
u16 bmp180_get_uncomp_temperature(void)
{
    u16 v_ut_u16 = BMP180_INIT_VALUE;

    /* Array holding the temperature LSB and MSB data*/
    u8 v_data_u8[BMP180_TEMPERATURE_DATA_BYTES] = { BMP180_INIT_VALUE, BMP180_INIT_VALUE };
    u8 v_ctrl_reg_data_u8 = BMP180_INIT_VALUE;

    /* used to return the bus communication results*/
    BMP180_RETURN_FUNCTION_TYPE v_com_rslt_s8 = E_BMP_COMM_RES;
    u8 v_wait_time_u8 = BMP180_INIT_VALUE;

    v_ctrl_reg_data_u8 = BMP180_T_MEASURE;
    v_wait_time_u8 = BMP180_TEMP_CONVERSION_TIME;
    v_com_rslt_s8 = p_bmp180->BMP180_BUS_WRITE_FUNC(p_bmp180->dev_addr,
                                                    BMP180_CTRL_MEAS_REG,
                                                    &v_ctrl_reg_data_u8,
                                                    BMP180_GEN_READ_WRITE_DATA_LENGTH);
    p_bmp180->delay_msec(v_wait_time_u8);
    v_com_rslt_s8 += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,
                                                    BMP180_ADC_OUT_MSB_REG,
                                                    v_data_u8,
                                                    BMP180_TEMPERATURE_DATA_LENGTH);
    if (v_com_rslt_s8 == 0)
    {
        v_ut_u16 =
            (u16)((((s32)((s8)v_data_u8[BMP180_TEMPERATURE_MSB_DATA])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
                  (v_data_u8[BMP180_TEMPERATURE_LSB_DATA]));
    }

    return v_ut_u16;
}

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
u32 bmp180_get_uncomp_pressure(void)
{
    /*j included for loop*/
    u8 v_j_u8 = BMP180_INIT_VALUE;
    u32 v_up_u32 = BMP180_INIT_VALUE;

    /*get the calculated pressure data*/
    u32 v_sum_u32 = BMP180_INIT_VALUE;
    u8 v_data_u8[BMP180_PRESSURE_DATA_BYTES] = { BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE };
    u8 v_ctrl_reg_data_u8 = BMP180_INIT_VALUE;

    /* used to return the bus communication results*/
    BMP180_RETURN_FUNCTION_TYPE v_com_rslt_s8 = E_BMP_COMM_RES;

    if (p_bmp180->sw_oversamp == BMP180_SW_OVERSAMP_U8X && p_bmp180->oversamp_setting == BMP180_OVERSAMP_SETTING_U8X)
    {
        for (v_j_u8 = BMP180_INIT_VALUE; v_j_u8 < BMP180_DATA_MEASURE; v_j_u8++)
        {
            /* 3 times getting pressure data*/
            v_ctrl_reg_data_u8 = BMP180_P_MEASURE +
                                 (p_bmp180->oversamp_setting << BMP180_SHIFT_BIT_POSITION_BY_06_BITS);
            v_com_rslt_s8 = p_bmp180->BMP180_BUS_WRITE_FUNC(p_bmp180->dev_addr,
                                                            BMP180_CTRL_MEAS_REG,
                                                            &v_ctrl_reg_data_u8,
                                                            BMP180_GEN_READ_WRITE_DATA_LENGTH);
            p_bmp180->delay_msec(BMP180_2MS_DELAY_U8X + (BMP180_3MS_DELAY_U8X << (p_bmp180->oversamp_setting)));
            v_com_rslt_s8 += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,
                                                            BMP180_ADC_OUT_MSB_REG,
                                                            v_data_u8,
                                                            BMP180_PRESSURE_DATA_LENGTH);
            if (v_com_rslt_s8 == 0)
            {
                v_sum_u32 =
                    (u32)((((u32)v_data_u8[BMP180_PRESSURE_MSB_DATA] << BMP180_SHIFT_BIT_POSITION_BY_16_BITS) |
                           ((u32) v_data_u8[BMP180_PRESSURE_LSB_DATA] << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
                           (u32) v_data_u8[BMP180_PRESSURE_XLSB_DATA]) >>
                          (BMP180_CALCULATE_TRUE_PRESSURE - p_bmp180->oversamp_setting));
                p_bmp180->number_of_samples = BMP180_INITIALIZE_NUMBER_OF_SAMPLES_U8X;
                v_up_u32 = v_up_u32 + v_sum_u32;
            }

            /*add up with dummy var*/
        }
        v_up_u32 = v_up_u32 / BMP180_AVERAGE_U8X; /*averaging*/
    }
    else if (p_bmp180->sw_oversamp == BMP180_INITIALIZE_SW_OVERSAMP_U8X)
    {
        v_ctrl_reg_data_u8 = BMP180_P_MEASURE + (p_bmp180->oversamp_setting << BMP180_SHIFT_BIT_POSITION_BY_06_BITS);
        v_com_rslt_s8 = p_bmp180->BMP180_BUS_WRITE_FUNC(p_bmp180->dev_addr,
                                                        BMP180_CTRL_MEAS_REG,
                                                        &v_ctrl_reg_data_u8,
                                                        BMP180_GEN_READ_WRITE_DATA_LENGTH);
        p_bmp180->delay_msec(BMP180_2MS_DELAY_U8X + (BMP180_3MS_DELAY_U8X << (p_bmp180->oversamp_setting)));
        v_com_rslt_s8 += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,
                                                        BMP180_ADC_OUT_MSB_REG,
                                                        v_data_u8,
                                                        BMP180_PRESSURE_DATA_LENGTH);
        if (v_com_rslt_s8 == 0)
        {
            v_up_u32 =
                (u32)((((u32)v_data_u8[BMP180_PRESSURE_MSB_DATA] << BMP180_SHIFT_BIT_POSITION_BY_16_BITS) |
                       ((u32) v_data_u8[BMP180_PRESSURE_LSB_DATA] << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
                       (u32) v_data_u8[BMP180_PRESSURE_XLSB_DATA]) >>
                      (BMP180_CALCULATE_TRUE_PRESSURE - p_bmp180->oversamp_setting));
            p_bmp180->number_of_samples = BMP180_INITIALIZE_NUMBER_OF_SAMPLES_U8X;
        }
    }

    return v_up_u32;
}
