// Copyright 2021 IOsetting <iosetting(at)outlook.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ssd1306.h"

/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))

/* SSD1306 data buffer */
static uint8_t SSD1306_Buffer_all[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* Private SSD1306 structure */
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;

/* Private variable */
static SSD1306_t SSD1306;

uint8_t SSD1306_Init(void) 
{
    LL_mDelay(500);
    /* Init LCD */
    SSD1306_WriteCommand(0xAE); //display off
    /**
     * Set the lower start column address of pointer by command 00h~0Fh.
     * Set the upper start column address of pointer by command 10h~1Fh.
     */
    SSD1306_WriteCommand(0x00); //---set low column address
    SSD1306_WriteCommand(0x10); //---set high column address

    /** set contrast control register, 2 bytes, 0x00 - 0xFF */
    SSD1306_WriteCommand(0x81);
    SSD1306_WriteCommand(0x7F);
    /** 0xA4,Output follows RAM content
     *  0xA5,Output ignores RAM content */
    SSD1306_WriteCommand(0xA4);
    /** 0xA6, Normal display (RESET)
     *  0xA7, Inverse display */
    SSD1306_WriteCommand(0xA6);
    /* 0x20,Set Memory Addressing Mode, 2 bytes, 
     *   0x00,Horizontal Addressing Mode (slide horizontally and goto next page)
     *   0x01,Vertical Addressing Mode (slide vertically and goto next column)
     *   0x02,Page Addressing Mode (RESET) (slide horizontally and remain in the same page)
     *   0x03,Invalid
    */
    SSD1306_WriteCommand(0x20); 
    SSD1306_WriteCommand(0x00);
    /**
     * Set the page start address of the target display location by command B0h to B7h
     * For Page Addressing Mode only
     */
    SSD1306_WriteCommand(0xB0);
    /** 
     * Set Page Address, 3 bytes
     * For Horizontal and Vertical Addressing Mode only
     */
    SSD1306_WriteCommand(0x22);
    SSD1306_WriteCommand(0x00); // From Page 0
    SSD1306_WriteCommand(0x07); // To Page 7

    /** 
     * COM Output Scan Direction
     * 0xC0: normal mode (RESET) Scan from COM0 to COM[N –1]
     * 0xC8: remapped mode. Scan from COM[N-1] to COM0 */
    SSD1306_WriteCommand(0xC8); //Set COM Output Scan Direction
    /**
     * Set display RAM display start line register from 0-63 */
    SSD1306_WriteCommand(0x40);
    /**
     * Segment Re-map
     * 0xA0: column address 0 is mapped to SEG0 (RESET),
     * 0xA1: column address 127 is mapped to SEG0 */
    SSD1306_WriteCommand(0xA1);
    /**
     * Set MUX ratio to N+1 MUX
     * N=A[5:0]: from 16MUX to 64MUX, RESET=111111b (i.e. 63d, 64MUX)
     * A[5:0] from 0 to 14 are invalid entry.*/
    SSD1306_WriteCommand(0xA8);
    SSD1306_WriteCommand(0x3F);
    /** 
     * Set Display Offset, Set vertical shift by COM from 0d~63d
     * The value is reset to 00h after RESET */
    SSD1306_WriteCommand(0xD3);
    SSD1306_WriteCommand(0x00); // offset in vertical
    /**
     * Set COM Pins Hardware Configuration
     * A[4]=0b, Sequential COM pin configuration
     * A[4]=1b(RESET), Alternative COM pin configuration
     * A[5]=0b(RESET), Disable COM Left/Right remap
     * A[5]=1b, Enable COM Left/Right remap */
    SSD1306_WriteCommand(0xDA);
    SSD1306_WriteCommand(0x12); // A[4]=0, A[5]=1
    /**
     * Set Display Divide Ratio/Oscillator Frequency
     * */
    SSD1306_WriteCommand(0xD5);
    SSD1306_WriteCommand(0xF0); // divide ratio
    /**
     * Set Pre-charge Period */
    SSD1306_WriteCommand(0xD9);
    SSD1306_WriteCommand(0x22);
    /**
     * Set V COMH Deselect Level
     * 0x00: 0.65 * Vcc
     * 0x10: 0.77 * Vcc (RESET)
     * 0x11: 0.83 * Vcc
     * */
    SSD1306_WriteCommand(0xDB);
    SSD1306_WriteCommand(0x10);

    /** charge pump setting
     * 0x10: Disable charge pump(RESET)
     * 0x14: Enable charge pump during display on
     */
    SSD1306_WriteCommand(0x8D);
    SSD1306_WriteCommand(0x14);

    /** 0xAE, Display OFF (sleep mode), 
     *  0xAF, Display ON in normal mode */
    SSD1306_WriteCommand(0xAF);

    /* Clear screen */
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    /* Update screen */
    SSD1306_UpdateScreen();

    /* Set default values */
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;

    /* Initialized OK */
    SSD1306.Initialized = 1;

    /* Return OK */
    return 1;
}

void SSD1306_UpdateScreen(void) 
{
    APP_I2C_Transmit(SSD1306_I2C_ADDR, 0x40, SSD1306_Buffer_all, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
}

void SSD1306_ToggleInvert(void) 
{
    uint16_t i;
    
    /* Toggle invert */
    SSD1306.Inverted = !SSD1306.Inverted;
    
    /* Do memory toggle */
    for (i = 0; i < sizeof(SSD1306_Buffer_all); i++)
    {
        SSD1306_Buffer_all[i] = ~SSD1306_Buffer_all[i];
    }
}

void SSD1306_Fill(uint8_t color)
{
    if (SSD1306.Inverted)
    {
        color = (uint8_t)!color;
    }
    /* Set memory */
    memset(SSD1306_Buffer_all, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
}

void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        /* Error */
        return;
    }

    /* Check if pixels are inverted */
    if (SSD1306.Inverted)
    {
        color = (uint8_t)!color;
    }

    /* Set color */
    if (color == SSD1306_COLOR_WHITE)
    {
        SSD1306_Buffer_all[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        SSD1306_Buffer_all[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void SSD1306_GotoXY(uint16_t x, uint16_t y)
{
    /* Set write pointers */
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

char SSD1306_Putc(char ch, FontDef_t* font, uint8_t color)
{
    uint32_t i, b, j, k;

    for (i = 0; i < font->height; i++)
    {
        for (j = 0; j < font->bytes; j++)
        {
            b = font->data[((ch - 32) * font->height + i) * font->bytes + j];
            if (font->order == 0)
            {
                for (k = 0; k < 8 && k < font->width - j * 8; k++)
                {
                    if ((b << k) & 0x80)
                    {
                        SSD1306_DrawPixel(SSD1306.CurrentX + (j * 8) + k, (SSD1306.CurrentY + i), (uint8_t) color);
                    }
                    else
                    {
                        SSD1306_DrawPixel(SSD1306.CurrentX + (j * 8) + k, (SSD1306.CurrentY + i), (uint8_t) !color);
                    }
                }
            }
            else
            {
                for (k = 0; k < 8 && k < font->width - j * 8; k++)
                {
                    if (b & (0x0001 << k))
                    {
                        SSD1306_DrawPixel(SSD1306.CurrentX + (j * 8) + k, (SSD1306.CurrentY + i), (uint8_t) color);
                    }
                    else
                    {
                        SSD1306_DrawPixel(SSD1306.CurrentX + (j * 8) + k, (SSD1306.CurrentY + i), (uint8_t) !color);
                    }
                }
            }
        }
    }

    /* Increase pointer */
    SSD1306.CurrentX += font->width;

    /* Return character written */
    return ch;
}

char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color)
{
    /* Write characters */
    while (*str)
    {
        /* Write character by character */
        if (SSD1306_Putc(*str, Font, color) != *str)
        {
            /* Return error */
            return *str;
        }
        
        /* Increase string pointer */
        str++;
    }
    
    /* Everything OK, zero should be returned */
    return *str;
}

void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t c)
{
    int16_t dx, dy, sx, sy, err, e2, i, tmp; 
    
    /* Check for overflow */
    if (x0 >= SSD1306_WIDTH)
    {
        x0 = SSD1306_WIDTH - 1;
    }
    if (x1 >= SSD1306_WIDTH)
    {
        x1 = SSD1306_WIDTH - 1;
    }
    if (y0 >= SSD1306_HEIGHT)
    {
        y0 = SSD1306_HEIGHT - 1;
    }
    if (y1 >= SSD1306_HEIGHT)
    {
        y1 = SSD1306_HEIGHT - 1;
    }
    
    dx = (x0 < x1) ? (x1 - x0) : (x0 - x1); 
    dy = (y0 < y1) ? (y1 - y0) : (y0 - y1); 
    sx = (x0 < x1) ? 1 : -1; 
    sy = (y0 < y1) ? 1 : -1; 
    err = ((dx > dy) ? dx : -dy) / 2; 

    if (dx == 0)
    {
        if (y1 < y0)
        {
            tmp = y1;
            y1 = y0;
            y0 = tmp;
        }
        
        if (x1 < x0)
        {
            tmp = x1;
            x1 = x0;
            x0 = tmp;
        }
        
        /* Vertical line */
        for (i = y0; i <= y1; i++)
        {
            SSD1306_DrawPixel(x0, i, c);
        }
        
        /* Return from function */
        return;
    }
    
    if (dy == 0)
    {
        if (y1 < y0)
        {
            tmp = y1;
            y1 = y0;
            y0 = tmp;
        }
        
        if (x1 < x0)
        {
            tmp = x1;
            x1 = x0;
            x0 = tmp;
        }
        
        /* Horizontal line */
        for (i = x0; i <= x1; i++)
        {
            SSD1306_DrawPixel(i, y0, c);
        }
        
        /* Return from function */
        return;
    }

    while (1)
    {
        SSD1306_DrawPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        e2 = err;
        if (e2 > -dx)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y0 += sy;
        }
    }
}

void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, c);
    SSD1306_DrawPixel(x0, y0 - r, c);
    SSD1306_DrawPixel(x0 + r, y0, c);
    SSD1306_DrawPixel(x0 - r, y0, c);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawPixel(x0 + x, y0 + y, c);
        SSD1306_DrawPixel(x0 - x, y0 + y, c);
        SSD1306_DrawPixel(x0 + x, y0 - y, c);
        SSD1306_DrawPixel(x0 - x, y0 - y, c);

        SSD1306_DrawPixel(x0 + y, y0 + x, c);
        SSD1306_DrawPixel(x0 - y, y0 + x, c);
        SSD1306_DrawPixel(x0 + y, y0 - x, c);
        SSD1306_DrawPixel(x0 - y, y0 - x, c);
    }
}

void SSD1306_Image(uint8_t *img, uint8_t frame, uint8_t x, uint8_t y)
{
    uint32_t i, b, j;

    b = 0;
    if(frame >= img[2])
        return;
    uint32_t start = (frame * (img[3] + (img[4] << 8)));
    
    /* Go through font */
    for (i = 0; i < img[1]; i++) {
        for (j = 0; j < img[0]; j++) {
            SSD1306_DrawPixel(x + j, (y + i), (uint8_t) (img[b/8 + 5 + start] >> (b%8)) & 1);
            b++;
        }
    }
}

void SSD1306_ON(void)
{
    SSD1306_WriteCommand(0x8D);
    SSD1306_WriteCommand(0x14);
    SSD1306_WriteCommand(0xAF);
}
void SSD1306_OFF(void)
{
    SSD1306_WriteCommand(0x8D);
    SSD1306_WriteCommand(0x10);
    SSD1306_WriteCommand(0xAE);
}

void SSD1306_WriteCommand(uint8_t command)
{
    APP_I2C_Transmit(SSD1306_I2C_ADDR, 0x00, &command, 1);
}

void SSD1306_WriteData(uint8_t data)
{
    APP_I2C_Transmit(SSD1306_I2C_ADDR, 0x40, &data, 1);
}
