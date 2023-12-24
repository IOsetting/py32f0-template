/***
 * Demo: BMP280 - Digital Pressure Sensor
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
#include "bmp280.h"
#include "i2c.h"

#define MASTER_ADDRESS        0xA0

struct bmp280_dev bmp;
struct bmp280_config conf;
struct bmp280_uncomp_data ucomp_data;
uint32_t pres32, pres64;
double pres;

static void APP_I2CConfig(void);


int main(void)
{
  int8_t rslt;

  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: BMP280 - Digital Pressure Sensor\r\nClock: %ld\r\n", SystemCoreClock);

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

  /* Map the delay function pointer with the function responsible for implementing the delay */
  bmp.delay_ms = LL_mDelay;
  /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
  bmp.dev_id = BMP280_I2C_ADDR_PRIM;
  /* Select the interface mode as I2C */
  bmp.intf = BMP280_I2C_INTF;
  /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
  bmp.read = APP_I2C_Receive;
  bmp.write = APP_I2C_Transmit;

  rslt = bmp280_init(&bmp);
  printf("bmp280_init status: %2d\r\n", rslt);

  /* Always read the current settings before writing, especially when
   * all the configuration is not modified
   */
  rslt = bmp280_get_config(&conf, &bmp);
  printf("bmp280_get_config status: %2d\r\n", rslt);

  /* configuring the temperature oversampling, filter coefficient and output data rate */
  /* Overwrite the desired settings */
  conf.filter = BMP280_FILTER_COEFF_2;

  /* Pressure oversampling set at 4x */
  conf.os_pres = BMP280_OS_4X;

  /* Setting the output data rate as 1HZ(1000ms) */
  conf.odr = BMP280_ODR_1000_MS;
  rslt = bmp280_set_config(&conf, &bmp);
  printf("bmp280_set_config status: %2d\r\n", rslt);

  /* Always set the power mode after setting the configuration */
  rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
  printf("bmp280_set_power_mode status: %2d\r\n", rslt);
  while (1)
  {
    /* Reading the raw data from sensor */
    rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

    /* Getting the compensated pressure using 32 bit precision */
    rslt = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);

    /* Getting the compensated pressure using 64 bit precision */
    rslt = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);

    /* Getting the compensated pressure as floating point value */
    // rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
    printf("UP: %ld, P32: %ld, P64: %ld, P64N: %ld\r\n",
           ucomp_data.uncomp_press,
           pres32,
           pres64,
           pres64 / 256);
    bmp.delay_ms(1000); /* Sleep time between measurements = BMP280_ODR_1000_MS */
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
