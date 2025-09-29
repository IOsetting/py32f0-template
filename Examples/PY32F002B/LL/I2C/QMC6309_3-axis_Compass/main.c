#include "main.h"
#include "qmc6309.h"


int main(void)
{
  int16_t qmc63_buff[3];

  BSP_Rcc48MConfig();
  BSP_GpioConfig();
  BSP_UartConfig(115200);
  BSP_I2cReconfig();
  LL_mDelay(10);

  if (QMC6309_Detect(QMC6309_ADDR) == SUCCESS)
  {
    BSP_UartTxString("qmc6309 detected\r\n");
    QMC6309_Init();
    while (1)
    {
      QMC6309_ReadAll(qmc63_buff);
      BSP_UartTxInt(qmc63_buff[0], 9);
      BSP_UartTxInt(qmc63_buff[1], 8);
      BSP_UartTxInt(qmc63_buff[2], 8);
      BSP_UartTxString("\r\n");
      LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_2);
      LL_mDelay(100);
    }
  }
  else
  {
    BSP_UartTxString("not detected\r\n");
  }
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
