// This example was tested with 'PY32F030F28P6TU TSSOP20' chip

#include "py32f0xx_hal_dma.h"
#include "py32f0xx_hal_i2c.h"
#include "py32f0xx_bsp_printf.h"

#define I2C_ADDRESS        0xA0     /* host address */

I2C_HandleTypeDef I2cHandle;

void APP_ErrorHandler(void);
static void APP_I2C_Config(void);

#define SLAVE_ADDRESS_LCD 0x4E // PCF8574

// LCD handling functions are borrowed from https://controllerstech.com/i2c-lcd-in-stm32/
void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd&0xf0);
  data_l = ((cmd<<4)&0xf0);
  data_t[0] = data_u|0x0C; // en=1, rs=0
  data_t[1] = data_u|0x08; // en=0, rs=0
  data_t[2] = data_l|0x0C; // en=1, rs=0
  data_t[3] = data_l|0x08; // en=0, rs=0
  HAL_I2C_Master_Transmit (&I2cHandle, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data&0xf0);
  data_l = ((data<<4)&0xf0);
  data_t[0] = data_u|0x0D; // en=1, rs=0
  data_t[1] = data_u|0x09; // en=0, rs=0
  data_t[2] = data_l|0x0D; // en=1, rs=0
  data_t[3] = data_l|0x09; // en=0, rs=0
  HAL_I2C_Master_Transmit (&I2cHandle, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
  lcd_send_cmd (0x80);
  for (int i=0; i<70; i++) {
    lcd_send_data (' ');
  }
}

void lcd_put_cur(int row, int col)
{
    switch (row) {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}

void lcd_init (void)
{
  // 4 bit initialisation
  HAL_Delay(50); // wait for > 40ms
  lcd_send_cmd (0x30);
  HAL_Delay(5); // wait for > 4.1ms
  lcd_send_cmd (0x30);
  HAL_Delay(1); // wait for > 100us
  lcd_send_cmd (0x30);
  HAL_Delay(10);
  lcd_send_cmd (0x20); // 4bit mode
  HAL_Delay(10);

  // display initialisation
  lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
  HAL_Delay(1);
  lcd_send_cmd (0x08); // Display on/off control --> D=0, C=0, B=0 ---> display off
  HAL_Delay(1);
  lcd_send_cmd (0x01); // clear display
  HAL_Delay(1);
  HAL_Delay(1);
  lcd_send_cmd (0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  HAL_Delay(1);
  lcd_send_cmd (0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
  while (*str) lcd_send_data (*str++);
}

int main(void)
{
  HAL_Init();

  BSP_USART_Config();
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  APP_I2C_Config();

  lcd_init ();

  lcd_send_string ("HELLO WORLD!");
  HAL_Delay(1000);
  lcd_put_cur(1, 0);
  lcd_send_string("HOWDY!");
  HAL_Delay(2000);

  while(1);
}

static void APP_I2C_Config(void)
{
  I2cHandle.Instance             = I2C;
  I2cHandle.Init.ClockSpeed      = 100000;        // 100KHz ~ 400KHz
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

void APP_I2C_Transmit(uint8_t devAddress, uint8_t memAddress, uint8_t *pData, uint16_t len)
{
  HAL_I2C_Mem_Write(&I2cHandle, devAddress, memAddress, I2C_MEMADD_SIZE_8BIT, pData, len, 5000);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Export assert error source and line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1);
}
#endif /* USE_FULL_ASSERT */
