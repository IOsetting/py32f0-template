#include "main.h"
#include "spl06.h"


int main(void)
{
  uint8_t buf[14];
  int32_t data[2];
  float t, p;

  BSP_Rcc48MConfig();
  BSP_GpioConfig();
  BSP_UartConfig(115200);
  BSP_I2cReconfig();

  if (SPL06_Detect(SPL06_ADDR))
  {
    BSP_UartTxString("0x77 detected\r\n");
    SPL06_Init(SPL06_ADDR, SPL06_Mode_ContinuousBoth);
  }
  else if (SPL06_Detect(SPL06_ADDR_ALT))
  {
    BSP_UartTxString("0x76 detected\r\n");
    SPL06_Init(SPL06_ADDR_ALT, SPL06_Mode_ContinuousBoth);
  }
  else
  {
    BSP_UartTxString("not detected\r\n");
  }
  SPL06_SetPressureRate(SPL06_Rate_X8, SPL06_Rate_X8);
  SPL06_SetTemperatureRate(SPL06_Rate_X8, SPL06_Rate_X8);

  buf[0] = SPL06_Read(SPL06_TMP_CFG);
  BSP_UartTxHex(&buf[0], 1);
  BSP_UartTxChar(' ');
  buf[0] = SPL06_Read(SPL06_PSR_CFG);
  BSP_UartTxHex(&buf[0], 1);
  BSP_UartTxString("\r\n");

  while (1)
  {
    SPL06_GetBoth(data);
    t = SPL06_GetTemperature(data[1]);
    p = SPL06_GetPressure(data[1], data[0]);
    BSP_UartTxHex((uint8_t *)&data[0], 4);
    BSP_UartTxChar(' ');
    BSP_UartTxHex((uint8_t *)&data[1], 4);
    BSP_UartTxChar(' ');
    BSP_UartTxHex((uint8_t *)&t, 4);
    BSP_UartTxChar(' ');
    BSP_UartTxHex((uint8_t *)&p, 4);
    BSP_UartTxString("\r\n");
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_2);
    LL_mDelay(500);
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
