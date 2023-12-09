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

CAN_HandleTypeDef CanHandle = {0};
CAN_FilterTypeDef CanFilter = {0};
CAN_RxHeaderTypeDef CanRxHeader;
CAN_TxHeaderTypeDef CanTxHeader;
uint8_t TxData[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};
uint8_t RxData[8] = {0};
uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};


int main(void)
{
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
  CanFilter.MaskID         = 0xE0000000; /* Use all 29 bits for ID comparison */
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

  /* Enable interrupt */
  __HAL_CAN_ENABLE_IT(&CanHandle, (CAN_IT_RX_COMPLETE | CAN_IT_TX_PTB_COMPLETE));
  
  while (1);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t i;

  HAL_CAN_GetRxMessage(hcan, &CanRxHeader, RxData);
  printf("Data received (length: %ld): ", CanRxHeader.DataLength);
  for (i = 0; i < CanRxHeader.DataLength; i++)
  {
    printf("%02X ", RxData[i]);
  }
  printf("\r\n");
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
