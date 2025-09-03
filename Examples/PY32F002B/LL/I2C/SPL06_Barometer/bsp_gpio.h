#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void BSP_Rcc48MConfig(void);
void BSP_GpioConfig(void);
void BSP_I2cReconfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H */
