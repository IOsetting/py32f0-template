/**
 * Source code from: 
 *   https://www.newinnovations.nl/post/controlling-ws2812-and-ws2812b-using-only-stm32-spi/
*/
#include <string.h>
#include "ws2812_spi.h"

#define WS2812_FILL_BUFFER(COLOR) \
    for( uint8_t mask = 0x80; mask; mask >>= 1 ) { \
        if( COLOR & mask ) { *ptr++ = 0xfc; } \
        else { *ptr++ = 0x80; }}

uint8_t ws2812_buffer[WS2812_BUFFER_SIZE];

void ws2812_init(void) {
    memset(ws2812_buffer, 0, WS2812_BUFFER_SIZE);
    ws2812_send_spi();
}

void ws2812_send_spi(void)
{
    uint16_t i;
    for (i = 0; i < WS2812_BUFFER_SIZE; i++)
    {
        WS2812_SPI_TxRxByte(*(ws2812_buffer + i));
    }
}

void ws2812_pixel(uint16_t led_no, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t * ptr = &ws2812_buffer[24 * led_no];
    WS2812_FILL_BUFFER(g);
    WS2812_FILL_BUFFER(r);
    WS2812_FILL_BUFFER(b);
}

void ws2812_pixel_all(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t * ptr = ws2812_buffer;
    for( uint16_t i = 0; i < WS2812_NUM_LEDS; ++i) {
        WS2812_FILL_BUFFER(g);
        WS2812_FILL_BUFFER(r);
        WS2812_FILL_BUFFER(b);
    }
}