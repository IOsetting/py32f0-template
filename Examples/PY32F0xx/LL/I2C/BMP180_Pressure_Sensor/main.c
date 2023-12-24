/***
 * Demo: BMP180 - Digital Pressure Sensor
 * 
 *              PF1   -> SCL
 *              PF0   -> SDA
 *              GND   -> GND
 *              3.3V  -> VCC
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "bmp180.h"
#include "i2c.h"

#define MASTER_ADDRESS        0xA0

struct bmp180_t bmp180;
uint16_t v_uncomp_temp_u16a, v_uncomp_temp_u16b;
uint32_t v_uncomp_press_u32a, v_uncomp_press_u32b;

static void APP_I2CConfig(void);


int main(void)
{
  int8_t rslt;

  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: BMP180 - Digital Pressure Sensor\r\nClock: %ld\r\n", SystemCoreClock);

  APP_I2CConfig();

  // for (uint8_t i = 0; i < 0xFF; i++)
  // {
  //   printf("%02x ", i);
  //   if (APP_I2C_TestAddress(i) == SUCCESS)
  //   {
  //     printf("SUCCESS");
  //   }
  //   printf("\r\n");
  // }

  bmp180.bus_write = APP_I2C_Transmit;
  bmp180.bus_read = APP_I2C_Receive;
  bmp180.dev_addr = BMP180_I2C_ADDR;
  bmp180.delay_msec = LL_mDelay;

  rslt = bmp180_init(&bmp180);
  printf("bmp180_init status: %2d\r\n", rslt);

  rslt = bmp180_get_calib_param();
  printf("bmp180_get_calib_param status: %2d\r\n", rslt);

  while (1)
  {
    /*  This API is used to read the
    *  uncompensated temperature(ut) value*/
    v_uncomp_temp_u16a = bmp180_get_uncomp_temperature();

    /*  This API is used to read the
    *  uncompensated pressure(ut) value*/
    v_uncomp_press_u32a = bmp180_get_uncomp_pressure();

    /****************************************************************************
     *  This API is used to read the
     *  true temperature(t) value input
     *  parameter as uncompensated temperature(ut)
     ***************************************************************************/
    v_uncomp_temp_u16b = bmp180_get_temperature(v_uncomp_temp_u16a);

    /****************************************************************************
     *  This API is used to read the
     *  true pressure(p) value
     *  input parameter as uncompensated pressure(up)
     ***************************************************************************/
    v_uncomp_press_u32b = bmp180_get_pressure(v_uncomp_press_u32a);

    printf("T16: %d, %d, P32: %ld, %ld\r\n",
           v_uncomp_temp_u16a,
           v_uncomp_temp_u16b,
           v_uncomp_press_u32a,
           v_uncomp_press_u32b);
    LL_mDelay(1000);
  }
}

static void APP_I2CConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  // PF1 SCL
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  // PF0 SDA
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

  LL_I2C_InitTypeDef I2C_InitStruct;
  /*
   * Clock speed:
   * - standard = 100khz
   * - fast     = 400khz
  */
  I2C_InitStruct.ClockSpeed      = LL_I2C_MAX_SPEED_FAST;
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1     = MASTER_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
