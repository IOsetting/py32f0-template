#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "py32f002b_ll_rcc.h"
#include "py32f002b_ll_bus.h"
#include "py32f002b_ll_system.h"
#include "py32f002b_ll_cortex.h"
#include "py32f002b_ll_utils.h"
#include "py32f002b_ll_pwr.h"
#include "py32f002b_ll_gpio.h"
#include "py32f002b_ll_tim.h"
#include "stdio.h"

#define DEBUG 1

#if DEBUG == 0
    #define DEBUG_PRINTF(...)                   (void)(0)
#else
    #define DEBUG_PRINTF(...)                   printf(__VA_ARGS__);
#endif


void APP_ErrorHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
