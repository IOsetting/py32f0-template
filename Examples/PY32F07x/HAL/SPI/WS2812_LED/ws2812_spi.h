#ifndef __WS2812_SIP_H
#define __WS2812_SIP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define WS2812_NUM_LEDS 64
#define WS2812_SPI_HANDLE Spi1Handle

#define WS2812_RESET_PULSE 60
#define WS2812_BUFFER_SIZE (WS2812_NUM_LEDS * 24 + WS2812_RESET_PULSE)

extern SPI_HandleTypeDef WS2812_SPI_HANDLE;
extern uint8_t ws2812_buffer[];

void ws2812_init(void);
void ws2812_send_spi(void);
void ws2812_pixel(uint16_t led_no, uint8_t r, uint8_t g, uint8_t b);
void ws2812_pixel_all(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_SIP_H */
