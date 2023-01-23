/******************************************************************************
** 
 * \file        st7567.c
 * \author      IOsetting | iosetting@outlook.com
 * \date        
 * \brief       Library of ST7567 LCD
 * \note        
 * \version     v0.1
 * \ingroup     demo
 *
 *              A8    -> RES, RESET
 *              A9    -> DC, A0
 *              A10   -> LED-A, Backlight
 *              B2    -> CSB, Chip Select
 *              B3    -> SCK, SCL, CLK, Clock
 *              B5    -> MOSI, SDA
 *              GND   -> GND
 *              3.3V  -> VCC
 * 
 * 
******************************************************************************/

#include <string.h>
#include "st7567.h"

/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))

#if ST7567_X_ORIENT == ST7567_SEG_DIRECTION_REVERSE
    #define ST7567_X_OFFSET  ST7567_SEG_EXPAND
#else
    #define ST7567_X_OFFSET  0
#endif

/**
 * In datasheet, it says "the column address is increased (+1) after each display 
 * data access (read/write). This allows MPU accessing DDRAM content continuously. 
 * This feature stops at the end of each page (Column Address “83h”) because the 
 * Column Address and Page Address circuits are independent. For example, both Page 
 * Address and Column Address should be assigned for changing the DDRAM pointer 
 * from (Page-0, Column-83h) to (Page-1, Column-0)."
 * In actual test the Page Address will grow automatically.
*/
/* ST7567 data buffer */
static uint8_t ST7567_Buffer_all[(ST7567_WIDTH + ST7567_SEG_EXPAND) * ST7567_HEIGHT / 8];

/* Private ST7567 structure */
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} ST7567_t;

/* Private variable */
static ST7567_t ST7567;


static void ST7567_TransmitByte(uint8_t dat)
{
    ST7567_CS_LOW;
    SPI_TxRxByte(dat);
    ST7567_CS_HIGH;
}

static void ST7567_Transmit(const uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
    while (Size-- > 0)
    {
        ST7567_TransmitByte(*(pData++));
    }
}

void ST7567_WriteCommand(uint8_t command)
{
    ST7567_DC_LOW;
    ST7567_TransmitByte(command);
    ST7567_DC_HIGH;
}

void ST7567_WriteData(uint8_t data)
{
    ST7567_TransmitByte(data);
}

void ST7567_Init(void)
{
    ST7567_Reset();
    ST7567_BackLight_On();

    ST7567_WriteCommand(ST7567_RESET);
    ST7567_WriteCommand(ST7567_POWER_CONTROL
        |ST7567_POWER_CONTROL_VB
        |ST7567_POWER_CONTROL_VR
        |ST7567_POWER_CONTROL_VF); 
    ST7567_WriteCommand(ST7567_SET_EV);
    ST7567_WriteCommand(ST7567_SET_EV_MASK & 0x20);
    ST7567_WriteCommand(ST7567_BIAS_1_9);
    ST7567_WriteCommand(ST7567_X_ORIENT);
    ST7567_WriteCommand(ST7567_Y_ORIENT);
    ST7567_WriteCommand(ST7567_REGULATION_RATIO | ST7567_REGULATION_RATIO_5_0);
    ST7567_WriteCommand(ST7567_INVERSE_DISPLAY_OFF);
    ST7567_WriteCommand(ST7567_DISPLAY_ON);
    ST7567_WriteCommand(ST7567_ALL_PIXEL_NORMAL);

    ST7567_WriteCommand(ST7567_SET_START_LINE | (0x00 & ST7567_SET_START_LINE_MASK));
    ST7567_WriteCommand(ST7567_SET_PAGE_ADDRESS | (0x00 & ST7567_SET_PAGE_ADDRESS_MASK));
    ST7567_WriteCommand(ST7567_SET_COLUMN_ADDRESS_MSB);
    ST7567_WriteCommand(ST7567_SET_COLUMN_ADDRESS_LSB);
}

void ST7567_Reset(void)
{
    ST7567_RESET_LOW;
    LL_mDelay(5);
    ST7567_RESET_HIGH;
}

void ST7567_BackLight_On(void)
{
    ST7567_BL_HIGH;
}

void ST7567_BackLight_Off(void)
{
    ST7567_BL_LOW;
}

void ST7567_SetContrast(uint8_t val)
{
    ST7567_WriteCommand(ST7567_SET_EV);
    ST7567_WriteCommand(ST7567_SET_EV_MASK & val);
}

void ST7567_UpdateScreen(void) 
{
    uint8_t i = 0, *pt = ST7567_Buffer_all;
    for (i = 0; i < ST7567_PAGES; i++)
    {
        ST7567_WriteCommand(ST7567_SET_PAGE_ADDRESS|(i & ST7567_SET_PAGE_ADDRESS_MASK));
        ST7567_WriteCommand(ST7567_SET_COLUMN_ADDRESS_MSB|(ST7567_X_OFFSET >> 4));
        ST7567_WriteCommand(ST7567_SET_COLUMN_ADDRESS_LSB|(ST7567_X_OFFSET & 0x0F));
        ST7567_Transmit(pt + (ST7567_WIDTH * i), ST7567_WIDTH, ST7567_TIMEOUT);
    }
}

void ST7567_ToggleInvert(void) 
{
    /* Toggle invert */
    ST7567.Inverted = !ST7567.Inverted;
    if (ST7567.Inverted)
    {
        ST7567_WriteCommand(ST7567_INVERSE_DISPLAY_ON);
    }
    else
    {
        ST7567_WriteCommand(ST7567_INVERSE_DISPLAY_OFF);
    }
}

void ST7567_Fill(uint8_t color)
{
    /* Set memory */
    memset(ST7567_Buffer_all, (color == ST7567_COLOR_BACK) ? 0x00 : 0xFF, sizeof(ST7567_Buffer_all));
}

void ST7567_DrawPixel(uint16_t x, uint16_t y, uint8_t color)
{
    if (x >= ST7567_WIDTH || y >= ST7567_HEIGHT)
    {
        /* Error */
        return;
    }

    if (color == ST7567_COLOR_FRONT)
    {
        ST7567_Buffer_all[x + (y / 8) * ST7567_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        ST7567_Buffer_all[x + (y / 8) * ST7567_WIDTH] &= ~(1 << (y % 8));
    }
}

void ST7567_GotoXY(uint16_t x, uint16_t y)
{
    /* Set write pointers */
    ST7567.CurrentX = x;
    ST7567.CurrentY = y;
}

char ST7567_Putc(char ch, FontDef_t* font, uint8_t color)
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
                        ST7567_DrawPixel(ST7567.CurrentX + (j * 8) + k, (ST7567.CurrentY + i), (uint8_t) color);
                    }
                    else
                    {
                        ST7567_DrawPixel(ST7567.CurrentX + (j * 8) + k, (ST7567.CurrentY + i), (uint8_t) !color);
                    }
                }
            }
            else
            {
                for (k = 0; k < 8 && k < font->width - j * 8; k++)
                {
                    if (b & (0x0001 << k))
                    {
                        ST7567_DrawPixel(ST7567.CurrentX + (j * 8) + k, (ST7567.CurrentY + i), (uint8_t) color);
                    }
                    else
                    {
                        ST7567_DrawPixel(ST7567.CurrentX + (j * 8) + k, (ST7567.CurrentY + i), (uint8_t) !color);
                    }
                }
            }
        }
    }

    /* Increase pointer */
    ST7567.CurrentX += font->width;

    /* Return character written */
    return ch;
}

char ST7567_Puts(char* str, FontDef_t* Font, uint8_t color)
{
    /* Write characters */
    while (*str)
    {
        /* Write character by character */
        if (ST7567_Putc(*str, Font, color) != *str)
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
 

void ST7567_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t c)
{
    int16_t dx, dy, sx, sy, err, e2, i, tmp; 
    
    /* Check for overflow */
    if (x0 >= ST7567_WIDTH)
    {
        x0 = ST7567_WIDTH - 1;
    }
    if (x1 >= ST7567_WIDTH)
    {
        x1 = ST7567_WIDTH - 1;
    }
    if (y0 >= ST7567_HEIGHT)
    {
        y0 = ST7567_HEIGHT - 1;
    }
    if (y1 >= ST7567_HEIGHT)
    {
        y1 = ST7567_HEIGHT - 1;
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
            ST7567_DrawPixel(x0, i, c);
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
            ST7567_DrawPixel(i, y0, c);
        }
        
        /* Return from function */
        return;
    }

    while (1)
    {
        ST7567_DrawPixel(x0, y0, c);
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

void ST7567_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c)
{
    /* Check input parameters */
    if (x >= ST7567_WIDTH || y >= ST7567_HEIGHT)
    {
        /* Return error */
        return;
    }

    /* Check width and height */
    if ((x + w) >= ST7567_WIDTH)
    {
        w = ST7567_WIDTH - x;
    }
    if ((y + h) >= ST7567_HEIGHT)
    {
        h = ST7567_HEIGHT - y;
    }

    /* Draw 4 lines */
    ST7567_DrawLine(x, y, x + w, y, c);         /* Top line */
    ST7567_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
    ST7567_DrawLine(x, y, x, y + h, c);         /* Left line */
    ST7567_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void ST7567_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c)
{
    uint8_t i;
    
    /* Check input parameters */
    if (x >= ST7567_WIDTH || y >= ST7567_HEIGHT)
    {
        /* Return error */
        return;
    }

    /* Check width and height */
    if ((x + w) >= ST7567_WIDTH)
    {
        w = ST7567_WIDTH - x;
    }
    if ((y + h) >= ST7567_HEIGHT)
    {
        h = ST7567_HEIGHT - y;
    }

    /* Draw lines */
    for (i = 0; i <= h; i++)
    {
        /* Draw lines */
        ST7567_DrawLine(x, y + i, x + w, y + i, c);
    }
}

void ST7567_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color)
{
    /* Draw lines */
    ST7567_DrawLine(x1, y1, x2, y2, color);
    ST7567_DrawLine(x2, y2, x3, y3, color);
    ST7567_DrawLine(x3, y3, x1, y1, color);
}


void ST7567_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color)
{
    int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
    yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
    curpixel = 0;
    
    deltax = ABS(x2 - x1);
    deltay = ABS(y2 - y1);
    x = x1;
    y = y1;

    if (x2 >= x1)
    {
        xinc1 = 1;
        xinc2 = 1;
    }
    else
    {
        xinc1 = -1;
        xinc2 = -1;
    }

    if (y2 >= y1)
    {
        yinc1 = 1;
        yinc2 = 1;
    }
    else
    {
        yinc1 = -1;
        yinc2 = -1;
    }

    if (deltax >= deltay)
    {
        xinc1 = 0;
        yinc2 = 0;
        den = deltax;
        num = deltax / 2;
        numadd = deltay;
        numpixels = deltax;
    }
    else
    {
        xinc2 = 0;
        yinc1 = 0;
        den = deltay;
        num = deltay / 2;
        numadd = deltax;
        numpixels = deltay;
    }

    for (curpixel = 0; curpixel <= numpixels; curpixel++)
    {
        ST7567_DrawLine(x, y, x3, y3, color);

        num += numadd;
        if (num >= den)
        {
            num -= den;
            x += xinc1;
            y += yinc1;
        }
        x += xinc2;
        y += yinc2;
    }
}

void ST7567_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ST7567_DrawPixel(x0, y0 + r, c);
    ST7567_DrawPixel(x0, y0 - r, c);
    ST7567_DrawPixel(x0 + r, y0, c);
    ST7567_DrawPixel(x0 - r, y0, c);

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

        ST7567_DrawPixel(x0 + x, y0 + y, c);
        ST7567_DrawPixel(x0 - x, y0 + y, c);
        ST7567_DrawPixel(x0 + x, y0 - y, c);
        ST7567_DrawPixel(x0 - x, y0 - y, c);

        ST7567_DrawPixel(x0 + y, y0 + x, c);
        ST7567_DrawPixel(x0 - y, y0 + x, c);
        ST7567_DrawPixel(x0 + y, y0 - x, c);
        ST7567_DrawPixel(x0 - y, y0 - x, c);
    }
}

void ST7567_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ST7567_DrawPixel(x0, y0 + r, c);
    ST7567_DrawPixel(x0, y0 - r, c);
    ST7567_DrawPixel(x0 + r, y0, c);
    ST7567_DrawPixel(x0 - r, y0, c);
    ST7567_DrawLine(x0 - r, y0, x0 + r, y0, c);

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

        ST7567_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        ST7567_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        ST7567_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        ST7567_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}

void ST7567_Image(uint8_t *img, uint8_t frame, uint8_t x, uint8_t y)
{
    uint32_t i, b, j;

    b = 0;
    if(frame >= img[2])
        return;
    uint32_t start = (frame * (img[3] + (img[4] << 8)));
    
    /* Go through font */
    for (i = 0; i < img[1]; i++) {
        for (j = 0; j < img[0]; j++) {
            ST7567_DrawPixel(x + j, (y + i), (uint8_t) (img[b/8 + 5 + start] >> (b%8)) & 1);
            b++;
        }
    }
}

void ST7567_TestDisplayRAM(void)
{
    uint16_t x, y, pos = 0;
    for (y = 0; y < 2; y++)
    {
        for (x = 0; x < ST7567_WIDTH; x++)
        {
            ST7567_Buffer_all[pos++] = x;
        }
        ST7567_Buffer_all[pos++] = 0x11;
        ST7567_Buffer_all[pos++] = 0x11;
        ST7567_Buffer_all[pos++] = 0x11;
        ST7567_Buffer_all[pos++] = 0x11;
        
    }
    ST7567_Transmit(ST7567_Buffer_all, sizeof(ST7567_Buffer_all), ST7567_TIMEOUT);
}
