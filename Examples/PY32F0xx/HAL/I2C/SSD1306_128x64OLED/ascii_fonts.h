#ifndef __ASCII_FONTS_H__
#define __ASCII_FONTS_H__

#include <stdint.h>

typedef struct {
    uint8_t width;
    uint8_t height;
    uint8_t order;
    uint8_t bytes;
    const uint8_t *data;
} FontDef_t;

extern FontDef_t Font_3x5;
extern FontDef_t Font_5x7;
extern FontDef_t Font_6x8;
extern FontDef_t Font_6x12;
extern FontDef_t Font_6x10;
extern FontDef_t Font_8x16;
extern FontDef_t Font_11x18;
extern FontDef_t Font_12x24;
extern FontDef_t Font_16x26;
extern FontDef_t Font_16x32;

#endif // __ASCII_FONTS_H__
