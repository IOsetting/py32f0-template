/**
 * PY32F07x          USB2TTL
 * PA02(TX)     -->  RX
 * PA03(RX)     -->  TX
 * GND          -->  GND
 * 
 *                     CAN
 * PB8          -->  CAN_RX
 * PB9          -->  CAN_TX
 * GND          -->  GND
 * 
*/
#include "main.h"
#include "py32f07x_bsp_printf.h"
#include "py32f07x_bsp_clock.h"

/**
 * Exchange TX_ID and RX_ID when testing communication between two devices
*/
#define TX_ID       0x12345670
#define RX_ID       0x1234567F

uint8_t TxData[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};
uint8_t RxData[8] = {0};


int main(void)
{
  uint32_t i;
  CAN_HandleTypeDef CanHandle = {0};
  CAN_FilterTypeDef CanFilter = {0};
  CAN_TxHeaderTypeDef CanTxHeader;
  CAN_RxHeaderTypeDef CanRxHeader;

  HAL_Init();
  BSP_HSE_ClockConfig();

  BSP_USART_Config();
  printf("PY32F07x CAN Example\r\nClock: %ld\r\n", SystemCoreClock);
  
  CanHandle.Instance                      = CAN1;
  CanHandle.Init.FrameFormat              = CAN_FRAME_CLASSIC;
  CanHandle.Init.Mode                     = CAN_MODE_NORMAL;
  CanHandle.Init.Prescaler                = 6U; 
  CanHandle.Init.NominalSyncJumpWidth     = 4U;
  CanHandle.Init.NominalTimeSeg1          = 12U;
  CanHandle.Init.NominalTimeSeg2          = 4U;
  printf("CAN initializing\r\n");
  if (HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  printf("CAN initialized\r\n");
  
  /* CAN filter config */
  CanFilter.IdType         = CAN_EXTENDED_ID;
  CanFilter.FilterChannel  = CAN_FILTER_CHANNEL_0;
  CanFilter.Rank           = CAN_FILTER_RANK_CHANNEL_NUMBER;
  CanFilter.FilterID       = RX_ID;
  CanFilter.FilterFormat   = 0xFFFFFFFF;
  CanFilter.MaskID         = 0x0; /* Use all 29 bits for ID comparison */
  CanFilter.MaskFormat     = 0xFFFFFFFF;
  if (HAL_CAN_ConfigFilter(&CanHandle, &CanFilter) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  printf("CAN configurated\r\n");

  if (HAL_CAN_Start(&CanHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  printf("CAN started\r\n");
  
  CanTxHeader.Identifier   = TX_ID;
  CanTxHeader.IdType       = CAN_EXTENDED_ID;
  CanTxHeader.TxFrameType  = CAN_DATA_FRAME;
  CanTxHeader.FrameFormat  = CAN_FRAME_CLASSIC;
  CanTxHeader.Handle       = 0x0;
  CanTxHeader.DataLength   = CAN_DLC_BYTES_8;
  if (HAL_CAN_AddMessageToTxFifo(&CanHandle, &CanTxHeader, TxData, CAN_TX_FIFO_PTB) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  printf("CAN sending\r\n");
  if (HAL_CAN_ActivateTxRequest(&CanHandle, CAN_TXFIFO_PTB_SEND) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Wait for receiving */
  while (__HAL_CAN_GET_RX_FIFO_FILL_LEVEL(&CanHandle) == CAN_RX_FIFO_EMPTY);

  while (1)
  {
    if (HAL_CAN_GetRxMessage(&CanHandle, &CanRxHeader, RxData) == HAL_OK)
    {
      printf("Data received (length: %ld): ", CanRxHeader.DataLength);
      for (i = 0; i < CanRxHeader.DataLength; i++)
      {
        printf("%02X ", RxData[i]);
      }
      printf("\r\n");
    }
    else
    {
      HAL_Delay(100);
    }
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
