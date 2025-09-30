/***
 * Demo: QMC6309, 3-axis e-compass
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
#include "qmc6309.h"


#define MASTER_ADDRESS      0xA0


static void APP_I2CConfig(void);

int main(void)
{
  int16_t magnet[3];
  int32_t heading;

  BSP_RCC_HSI_PLL48MConfig();

  BSP_USART_Config(115200);
  printf("I2C Demo: QMC6309 3-Axis Magnetic Sensor\r\nClock: %ld\r\n", SystemCoreClock);

  APP_I2CConfig();

  QMC6309_Init();
  LL_mDelay(200);
  
  while (1)
  {
    //printf("id: %02X ", QMC6309_Read(QMC6309_REG_CHIP_ID));
    QMC6309_ReadAll(magnet);
    heading = QMC6309_Heading(magnet[0], magnet[1], magnet[2]);
    printf("X:%7d  Y:%7d  Z:%7d  H:%7ld\r\n", magnet[0], magnet[1], magnet[2], heading);
    LL_mDelay(100);
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
