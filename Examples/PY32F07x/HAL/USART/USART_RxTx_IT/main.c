/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
uint8_t aTxBuffer[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
uint8_t aRxBuffer[12] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

int main(void)
{
  HAL_Init();

  UartHandle.Instance          = USART2;
  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  HAL_UART_Init(&UartHandle);
  
  if (HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)aTxBuffer, 12) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  while (1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  printf("Uart Error, ErrorCode = %ld\r\n", huart->ErrorCode);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if (HAL_UART_Receive_IT(UartHandle, (uint8_t *)aRxBuffer, 12) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Send the received bytes */
  if (HAL_UART_Transmit_IT(UartHandle, (uint8_t *)aRxBuffer, 12) != HAL_OK)
  {
    APP_ErrorHandler();
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

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
