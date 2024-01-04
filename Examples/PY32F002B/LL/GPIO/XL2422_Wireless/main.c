#include "main.h"
#include "py32f002b_bsp_clock.h"
#include "py32f002b_bsp_printf.h"
#include "xl2400p.h"
#include <string.h>

// 0:TX, 1:RX
#define XL2400_MODE 0

uint8_t data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
uint8_t Rec_Test[32];

static void APP_XL2400_Init(void);


int main(void)
{
  BSP_RCC_HSI_24MConfig();
  BSP_USART_Config(115200);
  printf("xl2400 demo\r\n");

  APP_XL2400_Init();
  XL2400_Init();
  LL_mDelay(200);

#if XL2400_MODE == 0
  XL2400_SetChannel(19);
  XL2400_SetTxMode();
  printf("XL2400 TX Initialized\r\n");
  uint8_t i = 0;
  while (1)
  {
    *data = i++;
    uint16_t size = strlen((char *)data);
    XL2400_Tx(data, size);
    LL_mDelay(500);
  }
#else
  // RX
  XL2400_SetRxMode();
	XL2400_SetChannel(18);
  while (1)
  {
    uint8_t status = XL2400_Rx(Rec_Test);
    if (status & XL2400_FLAG_RX_DR)
    {
      uint8_t Len = strlen((char *)Rec_Test);
      for (uint8_t i = 0; i < Len; i++)
      {
        printf("%02X ", Rec_Test[i]);
      }
      printf("\r\n");
    }
  }
#endif
}

static void APP_XL2400_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  XL2400_InitTypedef_t xl2400_initStruct;

  /* Enable GPIOB clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /*
   * XL2400P interface pins
   */
  /* GPIO output - B0:CSN, B1:SCK, B2:MOSI */
  GPIO_InitStruct.Pin       = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode      = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull      = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* GPIO input - B3:MISO, B4:INT */
  GPIO_InitStruct.Pin       = LL_GPIO_PIN_3 | LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode      = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull      = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  xl2400_initStruct.NssPort       = GPIOB;
  xl2400_initStruct.NssPin        = LL_GPIO_PIN_0;
  xl2400_initStruct.ClockPort     = GPIOB;
  xl2400_initStruct.ClockPin      = LL_GPIO_PIN_1;
  xl2400_initStruct.MosiPort      = GPIOB;
  xl2400_initStruct.MosiPin       = LL_GPIO_PIN_2;
  xl2400_initStruct.MisoPort      = GPIOB;
  xl2400_initStruct.MisoPin       = LL_GPIO_PIN_3;
  xl2400_initStruct.InterruptPort = GPIOB;
  xl2400_initStruct.InterruptPin  = LL_GPIO_PIN_4;
  xl2400_initStruct.SetPinHigh    = LL_GPIO_SetOutputPin;
  xl2400_initStruct.SetPinLow     = LL_GPIO_ResetOutputPin;
  xl2400_initStruct.GetPinState   = LL_GPIO_IsInputPinSet;
  XL2400_Config(xl2400_initStruct);
}

void APP_ErrorHandler(void)
{
  while (1);
}
