/*****************************************************************************
* | File      	:   EPD_Config.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master 
*                and enhance portability
*----------------
* |	This version:   V2.0
* | Date        :   2018-10-30
* | Info        :
* 1.add:
*   UBYTE\UWORD\UDOUBLE
* 2.Change:
*   EPD_RST -> EPD_RST_PIN
*   EPD_DC -> EPD_DC_PIN
*   EPD_CS -> EPD_CS_PIN
*   EPD_BUSY -> EPD_BUSY_PIN
* 3.Remote:
*   EPD_RST_1\EPD_RST_0
*   EPD_DC_1\EPD_DC_0
*   EPD_CS_1\EPD_CS_0
*   EPD_BUSY_1\EPD_BUSY_0
* 3.add:
*   #define DEV_Digital_Write(_pin, _value) bcm2835_gpio_write(_pin, _value)
*   #define DEV_Digital_Read(_pin) bcm2835_gpio_lev(_pin)
*   #define DEV_SPI_WriteByte(__value) bcm2835_spi_transfer(__value)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#ifndef _EPD_CONFIG_H_
#define _EPD_CONFIG_H_

#include "main.h"
#include "air32f10x.h"
#include "debug.h"
#include <stdint.h>
#include <stdio.h>

/**
 * Uncomment the part number to enable
*/

// #define EPD_1IN02
// #define EPD_1IN54_V2
#define EPD_1IN54
// #define EPD_1IN54B_V2
// #define EPD_1IN54B
// #define EPD_1IN54C
// #define EPD_1IN64G
// #define EPD_2IN7_V2
// #define EPD_2IN7
// #define EPD_2IN7B_V2
// #define EPD_2IN7B
// #define EPD_2IN9_V2
// #define EPD_2IN9
// #define EPD_2IN9B_V3
// #define EPD_2IN9BC
// #define EPD_2IN9D
// #define EPD_2IN13_V2
// #define EPD_2IN13_V3
// #define EPD_2IN13
// #define EPD_2IN13B_V3
// #define EPD_2IN13B_V4
// #define EPD_2IN13BC
// #define EPD_2IN13D
// #define EPD_2IN36G
// #define EPD_2IN66
// #define EPD_2IN66B
// #define EPD_3IN0G
// #define EPD_3IN7
// #define EPD_3IN52
// #define EPD_4IN01F
// #define EPD_4IN2
// #define EPD_4IN2B_V2
// #define EPD_4IN2BC
// #define EPD_4IN37G
// #define EPD_5IN65F
// #define EPD_5IN83_V2
// #define EPD_5IN83
// #define EPD_5IN83B_V2
// #define EPD_5IN83BC
// #define EPD_7IN3F
// #define EPD_7IN3G
// #define EPD_7IN5_HD
// #define EPD_7IN5_V2
// #define EPD_7IN5
// #define EPD_7IN5B_HD
// #define EPD_7IN5B_V2
// #define EPD_7IN5BC

#define DEBUG 1

/**
 * e-Paper GPIO
 */
#define EPD_RST_PIN     GPIOA, GPIO_Pin_6
#define EPD_DC_PIN      GPIOA, GPIO_Pin_4
#define EPD_CS_PIN      GPIOA, GPIO_Pin_3
#define EPD_BUSY_PIN    GPIOA, GPIO_Pin_2

/**
 * GPIO read and write
 */
#define EPD_Digital_Write(_pin, _value) GPIO_WriteBit(_pin, _value == 0? Bit_RESET:Bit_SET)
#define EPD_Digital_Read(_pin)          GPIO_ReadInputDataBit(_pin)
#define EPD_SPI_WriteByte(_value)       SPI_TxRx(_value)
#define EPD_Delay_ms(__xms)             Delay_Ms(__xms)

#endif
