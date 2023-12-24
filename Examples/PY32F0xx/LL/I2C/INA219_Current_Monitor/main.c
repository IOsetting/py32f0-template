/***
 * Demo: INA219, Bidirectional Current/Power Monitor
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
#include "ina219.h"
#include "i2c.h"

#define MASTER_ADDRESS        0x33

uint8_t ina219_shuntVolt_LSB_uV = 10;  // Shunt Voltage LSB value = 10uV
uint16_t ina219_calValue = 0;

uint16_t xbuf[4];

static uint8_t APP_I2CConfig(void);

int main(void)
{
  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: INA219 Bidirectional Current/Power Monitor\r\nClock: %ld\r\n", SystemCoreClock);

  APP_I2CConfig();

  // Reset
  INA219_Write(INA219_REG_CONF, INA219_RESET_YES << 15);
  xbuf[0] = INA219_Read(INA219_REG_CONF);

  // Configurate
  xbuf[1] = (INA219_RESET_NO << 15)
            |(INA219_BUS_VOLTAGE_RANGE_16V << 13)
            |(INA219_PGA_40_MV << 11)
            |(INA219_ADC_MODE_12_BIT_8_SAMPLES << 7)
            |(INA219_ADC_MODE_12_BIT_8_SAMPLES << 3)
            |(INA219_MODE_SHUNT_BUS_VOLTAGE_CONTINUOUS);
  INA219_Write(INA219_REG_CONF, xbuf[1]);
  xbuf[2] = INA219_Read(INA219_REG_CONF);
  printf("CONF: (reset)%04X, (write):%04X, (read-back):%04X\r\n", xbuf[0], xbuf[1], xbuf[2]);

  /*
   Max shunt voltage on shunt resistor is 40mV, R = 0.1Ω, so max expected current is 0.04V/0.1Ω = 0.4A
   Current_LSB = Max expected current / 2^15 = 0.4 / 2^15 = 0.000012207
   According to the cal fomula: 
     cal = 0.04096 / (Current_LSB * 0.1Ω) = 0.04096 / (0.000012207 * 0.1) = 33554.518 = 0x8312
   This value can be adjusted according the result of an external ammeter
  */
  ina219_calValue = 0x8312;
  INA219_Write(INA219_REG_CALIBRATION, ina219_calValue);

  while (1)
  {
    /* Shunt voltage is the voltage drop across the shunt resistor */
    xbuf[0] = INA219_Read(INA219_REG_SHUNT_VOLTAGE);
    /* Bus voltage is the voltage from VIN- to ground */
    xbuf[1] = INA219_Read(INA219_REG_BUS_VOLTAGE);
    /* Power = (Bus Voltage * Current) / 5000  */
    xbuf[2] = INA219_Read(INA219_REG_POWER);
    /* Current = (Shunt Voltage *  Calibration Register) / 4096  */
    xbuf[3] = INA219_Read(INA219_REG_CURRENT);
    printf("SHNT:%6d(%6dmV), BUS:%04X(%6dmV), POWR:%04X, CURT:%04X(%6duA)\r\n",
        (int16_t)xbuf[0], 
        (int16_t)xbuf[0] / 100, // Shunt voltage, 1 LSB step size = 10 μV
        xbuf[1],
        (xbuf[1] >> 3) * 4,     // Bus Voltage, bit[3,15], 1 LSB step size = 4 mV
        xbuf[2],
        xbuf[3],
        (int16_t)xbuf[3]);
    LL_mDelay(1000);
  }
}

static uint8_t APP_I2CConfig(void)
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

  return 0;
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
