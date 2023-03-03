// This example was tested with 'PY32F003W16S6TU SOP16' chip

#include "py32f0xx_hal_dma.h"
#include "py32f0xx_bsp_printf.h"

// https://github.com/scottc11/stm32-encoder
#include "encoder.h"

void APP_ErrorHandler(void);

Encoder_Status encoderStatus;

int main(void)
{
  HAL_Init();

  BSP_USART_Config();
  printf("SystemClk:%ld\r\n", SystemCoreClock);

  Encoder_Config(); // configure the encoder's timer
  Encoder_Init(); // start the encoder's timer

  while(1) {
    encoderStatus = Encoder_Get_Status();

    switch(encoderStatus) {
      case Incremented:
        printf("Incremented\n");
        break;
      case Decremented:
        printf("Decremented\n");
        break;
      case Neutral:
        break;
    }
  }
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
