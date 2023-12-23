/***
 * Demo: L3G4200D, 3-axis gyroscope
 * 
 *              PF1   -> SCL
 *              PF0   -> SDA
 *              GND   -> GND
 *              3.3V  -> VCC
 *              3.3V  -> CS (pull up to enable I2C)
 */
#include <string.h>
#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_bsp_clock.h"
#include "l3g4200d.h"


#define MASTER_ADDRESS      0xA0

extern int16_t delta[3], threshold[3];
int16_t vect[3];

static void APP_I2CConfig(void);

int main(void)
{
  uint8_t temp;

  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: L3G4200D, 3-axis gyroscope\r\nClock: %ld\r\n", SystemCoreClock);

  APP_I2CConfig();

  L3G4200D_Begin(L3G4200D_SCALE_500DPS, L3G4200D_DATARATE_100HZ_12_5);

  printf("Calibrating...\r\n");
  L3G4200D_Calibrate(100);
  printf("Delta:\tX:%6d  Y:%6d  Z:%6d\r\n", delta[0], delta[1], delta[2]);
  printf("Threshold:\tX:%6d  Y:%6d  Z:%6d\r\n", threshold[0], threshold[1], threshold[2]);

  printf("Start measuring...\r\n");
  while (1)
  {
    temp = L3G4200D_ReadTemperature();
    L3G4200D_ReadRaw(vect);
    printf("Temp:%2d   X:%6d  Y:%6d  Z:%6d ", temp, vect[0], vect[1], vect[2]);
    L3G4200D_ReadNormalize(vect);
    printf("\t\tX:%6d  Y:%6d  Z:%6d\r\n", vect[0], vect[1], vect[2]);
    LL_mDelay(50);
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
