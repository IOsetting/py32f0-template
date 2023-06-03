/***
 * Demo: RX Interrupts
 */
#include "main.h"
#include "py32f0xx_bsp_clock.h"

#define BUF_SIZE    128

extern const char HEX_TABLE[16];
uint8_t pBuf[BUF_SIZE], ch;
__IO uint8_t printFlag = 0, pos = 0;

void APP_USART_Config(uint32_t baudRate);
void APP_USART_IRQCallback(USART_TypeDef *USARTx);


/**
 * Overwrite __io_getchar and __io_putchar to avoid conflicts. 
 * Both functions are already defined in py32f0xx_bsp_printf.c but use UART1 instead.
*/
int __io_getchar(void) 
{
    return (0);
}

int __io_putchar(int ch)
{
    return (0);
}

/**
 * Send one char to UART port
*/
void APP_UART_TxChar(char ch)
{
    LL_USART_TransmitData8(USART2, ch);
    while (!LL_USART_IsActiveFlag_TC(USART2));
    LL_USART_ClearFlag_TC(USART2);
}

/**
 * Send one string (a serial of chars end with '\0') to UART port
*/
void APP_UART_TxString(char *str)
{
    while (*str) APP_UART_TxChar(*str++);
}

int main(void)
{
  // Set system clock to 48MHz
  BSP_RCC_HSI_PLL48MConfig();
  APP_USART_Config(115200);

  APP_UART_TxString("PY32F0 UART Interrupt RX Demo\r\n");

  while (1)
  {
    if (printFlag == 1)
    {
      LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
      APP_UART_TxString((char *)pBuf);
      APP_UART_TxString("\r\n");
      printFlag = 0;
      pos = 0;
    }
  }
}

void USART2_IRQHandler(void)
{
  APP_USART_IRQCallback(USART2);
}

/**
  * @brief USART2 GPIO & Interrupt Config
  */
void APP_USART_Config(uint32_t baudRate)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  /* USART2 Init */
  LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, baudRate);
  LL_USART_SetDataWidth(USART2, LL_USART_DATAWIDTH_8B);
  LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_1);
  LL_USART_SetParity(USART2, LL_USART_PARITY_NONE);
  LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);
  // Both direction
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
  LL_USART_Enable(USART2);
  LL_USART_ClearFlag_TC(USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  // PA2     ------> USART2_TX
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_4);
  // PA3     ------> USART2_RX
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_4);

  // Enable parity error interrupt
  LL_USART_EnableIT_PE(USART2);
  // Enable framing error, overrun error and noise error interrupt
  LL_USART_EnableIT_ERROR(USART2);
  // Enable RX not empty interrupt
  LL_USART_EnableIT_RXNE(USART2);
  // Interrupt priority
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);

}

void APP_USART_IRQCallback(USART_TypeDef *USARTx)
{
  // Parity Error
  if (LL_USART_IsActiveFlag_PE(USARTx))
  {
    LL_USART_ClearFlag_PE(USARTx);
  }
  // Framing Error
  if (LL_USART_IsActiveFlag_FE(USARTx))
  {
    LL_USART_ClearFlag_FE(USARTx);
  }
  // OverRun Error
  if (LL_USART_IsActiveFlag_ORE(USARTx))
  {
    LL_USART_ClearFlag_ORE(USARTx);
  }
  // Noise error
  if (LL_USART_IsActiveFlag_NE(USARTx))
  {
    LL_USART_ClearFlag_NE(USARTx);
  }
  // RX Not Empty, RX Not Empty Interrupt is enabled
  if (LL_USART_IsActiveFlag_RXNE(USARTx) && LL_USART_IsEnabledIT_RXNE(USARTx))
  {
    // Read one byte
    ch = LL_USART_ReceiveData8(USARTx);
    
    if (ch == '\r' || ch == '\n')
    {
      // Set print flag when meet \r or \n, ignore empty string
      if (pos > 0)
      {
        pBuf[pos++] = '\0';
        printFlag = 1;
      }
    }
    else
    {
      // Append new char
      pBuf[pos++] = ch;
      if (pos == BUF_SIZE)
      {
        pos = 0;
      }
    }
    // Read USART_DR will clear RXNE flag, so LL_USART_ClearFlag_RXNE() is not necessary
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
