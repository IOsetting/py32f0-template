#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "py32f0xx_hal_def.h"
#include "py32f0xx_hal_conf.h"

#ifndef ENCODER_TIM
#define ENCODER_TIM                            htim3
#endif

#ifndef ENCODER_GPIO_PORT
#define ENCODER_GPIO_PORT                      GPIOA
#endif
#ifndef ENCODER_GPIO_CH1
#define ENCODER_GPIO_CH1                       GPIO_PIN_6
#endif

#ifndef ENCODER_GPIO_CH2
#define ENCODER_GPIO_CH2                       GPIO_PIN_7
#endif

extern TIM_HandleTypeDef htim3;

typedef enum {
  Incremented = 1,
  Decremented = -1,
  Neutral = 0,
} Encoder_Status;


void Encoder_Config(void);
void Encoder_Init(void);
uint16_t Encoder_Read();
Encoder_Status Encoder_Get_Status();

#endif
