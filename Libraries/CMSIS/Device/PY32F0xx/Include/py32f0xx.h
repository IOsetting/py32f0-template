/**
  ******************************************************************************
  * @file    py32f0xx.h
  * @brief   CMSIS PY32F0xx Device Peripheral Access Layer Header File.
  * @version v1.0.0
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup py32f0xx
  * @{
  */
#ifndef __PY32F0XX_H
#define __PY32F0XX_H

/**
  * @brief PY32F0 Family
  */
#if !defined (PY32F0)
#define PY32F0
#endif /* PY32F0 */

/**  Uncomment the line below according to the target PY32 device used in your
  *  application.
  */

#if !defined (PY32F002Ax5) && !defined (PY32F002x6) && \
    !defined (PY32F003x4) && !defined (PY32F003x6) && !defined (PY32F003x8) && \
    !defined (PY32F030x3) && !defined (PY32F030x4) && !defined (PY32F030x6) && !defined (PY32F030x7) && !defined (PY32F030x8) && !defined (PY32F030xx) && \
    !defined (PY32F031x3) && !defined (PY32F031x4) && !defined (PY32F031x6) && !defined (PY32F031x7) && !defined (PY32F031x8) && \
    !defined (PY32F071x6) && !defined (PY32F071x8) && !defined (PY32F071x9) && !defined (PY32F071xB) && \
    !defined (PY32F072x6) && !defined (PY32F072x8) && !defined (PY32F072x9) && !defined (PY32F072xB)
/* #define PY32F002Ax5 */  /*!< PY32F002Ax5 Devices (PY32F002Ax5 microcontrollers where the Flash memory is 20  Kbytes) */
/* #define PY32F002x6  */  /*!< PY32F002x6  Devices (PY32F002x6  microcontrollers where the Flash memory is 24  Kbytes) */
/* #define PY32F003x4  */  /*!< PY32F003x4  Devices (PY32F003x4  microcontrollers where the Flash memory is 16  Kbytes) */
/* #define PY32F003x6  */  /*!< PY32F003x6  Devices (PY32F003x6  microcontrollers where the Flash memory is 32  Kbytes) */
/* #define PY32F003x8  */  /*!< PY32F003x8  Devices (PY32F003x8  microcontrollers where the Flash memory is 64  Kbytes) */
/* #define PY32F030x3  */  /*!< PY32F030x3  Devices (PY32F030x3  microcontrollers where the Flash memory is 8   Kbytes) */
/* #define PY32F030x4  */  /*!< PY32F030x4  Devices (PY32F030x4  microcontrollers where the Flash memory is 16  Kbytes) */
/* #define PY32F030x6  */  /*!< PY32F030x6  Devices (PY32F030x6  microcontrollers where the Flash memory is 32  Kbytes) */
/* #define PY32F030x7  */  /*!< PY32F030x7  Devices (PY32F030x7  microcontrollers where the Flash memory is 48  Kbytes) */
/* #define PY32F030x8  */  /*!< PY32F030x8  Devices (PY32F030x8  microcontrollers where the Flash memory is 64  Kbytes) */
/* #define PY32F031x3  */  /*!< PY32F031x3  Devices (PY32F031x3  microcontrollers where the Flash memory is 8   Kbytes) */
/* #define PY32F031x4  */  /*!< PY32F031x4  Devices (PY32F031x4  microcontrollers where the Flash memory is 16  Kbytes) */
/* #define PY32F031x6  */  /*!< PY32F031x6  Devices (PY32F031x6  microcontrollers where the Flash memory is 32  Kbytes) */
/* #define PY32F031x7  */  /*!< PY32F031x7  Devices (PY32F031x7  microcontrollers where the Flash memory is 48  Kbytes) */
/* #define PY32F031x8  */  /*!< PY32F031x8  Devices (PY32F031x8  microcontrollers where the Flash memory is 64  Kbytes) */
/* #define PY32F071x6  */  /*!< PY32F071x6  Devices (PY32F071x6  microcontrollers where the Flash memory is 32  Kbytes) */
/* #define PY32F071x8  */  /*!< PY32F071x8  Devices (PY32F071x8  microcontrollers where the Flash memory is 64  Kbytes) */
/* #define PY32F071x9  */  /*!< PY32F071x9  Devices (PY32F071x9  microcontrollers where the Flash memory is 96  Kbytes) */
/* #define PY32F071xB  */  /*!< PY32F071xB  Devices (PY32F071xB  microcontrollers where the Flash memory is 128 Kbytes) */
/* #define PY32F072x6  */  /*!< PY32F072x6  Devices (PY32F072x6  microcontrollers where the Flash memory is 32  Kbytes) */
/* #define PY32F072x8  */  /*!< PY32F072x8  Devices (PY32F072x8  microcontrollers where the Flash memory is 64  Kbytes) */
/* #define PY32F072x9  */  /*!< PY32F072x9  Devices (PY32F072x9  microcontrollers where the Flash memory is 96  Kbytes) */
/* #define PY32F072xB  */  /*!< PY32F072xB  Devices (PY32F072xB  microcontrollers where the Flash memory is 128 Kbytes) */
#endif
/**  Tip: To avoid modifying this file each time you need to switch between these
  *       devices, you can define the device in your toolchain compiler preprocessor.
  */

#if (defined(PY32F002Ax5))
#define PY32F002APRE
#elif (defined(PY32F002Bx5))
#define PY32F002BPRE
#elif (defined(PY32F030x3) || defined(PY32F030x4) || defined(PY32F030x6) || defined(PY32F030x7) || defined(PY32F030x8))
#define PY32F030PRE
#elif (defined(PY32F031x3) || defined(PY32F031x4) || defined(PY32F031x6) || defined(PY32F031x7) || defined(PY32F031x8))
#define PY32F031PRE
#elif (defined(PY32F003x4) || defined(PY32F003x6) || defined(PY32F003x8))
#define PY32F003PRE
#elif (defined(PY32F071x6) || defined(PY32F071x8) || defined(PY32F071x9) || defined(PY32F071xB))
#define PY32F071PRE
#elif (defined(PY32F072x6) || defined(PY32F072x8) || defined(PY32F072x9) || defined(PY32F072xB))
#define PY32F072PRE
#endif

/**
  * @brief CMSIS Device version number V1.0.0
  */
#define __PY32F0_DEVICE_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __PY32F0_DEVICE_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __PY32F0_DEVICE_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __PY32F0_DEVICE_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __PY32F0_DEVICE_VERSION        ((__PY32F0_DEVICE_VERSION_MAIN << 24)\
                                        |(__PY32F0_DEVICE_VERSION_SUB1 << 16)\
                                        |(__PY32F0_DEVICE_VERSION_SUB2 << 8 )\
                                        |(__PY32F0_DEVICE_VERSION_RC))

/**
  * @brief Device_Included
  */
#if defined(PY32F002Ax5)
#include "py32f002ax5.h"
#elif defined(PY32F00Bx5)
#include "py32f002bx5.h"
#elif defined(PY32F003x4)
#include "py32f003x4.h"
#elif defined(PY32F003x6)
#include "py32f003x6.h"
#elif defined(PY32F003x8)
#include "py32f003x8.h"
#elif defined(PY32F030x3)
#include "py32f030x3.h"
#elif defined(PY32F030x4)
#include "py32f030x4.h"
#elif defined(PY32F030x6)
#include "py32f030x6.h"
#elif defined(PY32F030x7)
#include "py32f030x7.h"
#elif defined(PY32F030x8)
#include "py32f030x8.h"
#elif defined(PY32F031x3)
#include "py32f031x3.h"
#elif defined(PY32F031x4)
#include "py32f031x4.h"
#elif defined(PY32F031x6)
#include "py32f031x6.h"
#elif defined(PY32F031x7)
#include "py32f031x7.h"
#elif defined(PY32F031x8)
#include "py32f031x8.h"
#elif defined(PY32F071x6)
#include "py32f071x6.h"
#elif defined(PY32F071x8)
#include "py32f071x8.h"
#elif defined(PY32F071x9)
#include "py32f071x9.h"
#elif defined(PY32F071xB)
#include "py32f071xB.h"
#elif defined(PY32F072x6)
#include "py32f072x6.h"
#elif defined(PY32F072x8)
#include "py32f072x8.h"
#elif defined(PY32F072x9)
#include "py32f072x9.h"
#elif defined(PY32F072xB)
#include "py32f072xB.h"
#else
#error "Please select first the target PY32F0xx device used in your application (in py32f0xx.h file)"
#endif /* Device_Included */

/**
  * @brief Exported_types
  */
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  ERROR = 0,
  SUCCESS = !ERROR
} ErrorStatus;

/**
  * @brief Exported_macros
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define HW32_REG(ADDRESS)     ( * ((volatile unsigned           int * )(ADDRESS)))

#define HW16_REG(ADDRESS)     ( * ((volatile unsigned short     int * )(ADDRESS)))

#define HW8_REG(ADDRESS)      ( * ((volatile unsigned          char * )(ADDRESS)))

/**
  * @brief Comment the line below if you will not use the peripherals drivers.
  *        In this case, these drivers will not be included and the application code will
  *        be based on direct access to peripherals registers
  */
#if !defined  (USE_HAL_DRIVER)
/*#define USE_HAL_DRIVER */
#endif /* USE_HAL_DRIVER */

#if defined (USE_HAL_DRIVER)
#if (defined(PY32F071PRE) || defined(PY32F072PRE))
#include "py32f07x_hal.h"
#else
#include "py32f0xx_hal.h"
#endif /* Device_Included */
#endif /* USE_HAL_DRIVER */

#endif /* __PY32F0xx_H */

#ifdef __cplusplus
}
#endif /* __cplusplus */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

