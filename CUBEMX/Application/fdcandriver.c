/**
 * @file fdcandriver.c
 * @author Simen Bjerkestrand (Simen.Bjerkestrand@propulsentnu.no)
 * @brief simple FDCAN driver for classic CAN with Fifo0
 * @version 1.0
 * @date 2023-11-04
 * 
 * @copyright Propulse NTNU (c) 2023
 * 
 */

/*****************************************************************************/
/** Included libraries                                                      **/
/*****************************************************************************/
#include "fdcandriver.h"

/*****************************************************************************/
/** Module variables                                                        **/
/*****************************************************************************/
typedef struct FDCANCallback_internal_s {
   FDCANCallback fp;
   uint32_t identifier;
} FDCANCallback_internal_t;

static FDCANCallback_internal_t FDCAN_listCallbacks[FDCAN_MAX_CALLBACKS];
static uint32_t FDCAN_listCallbacksCount;

/*****************************************************************************/
/** Public API                                                              **/
/*****************************************************************************/

STATUS FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
  /* Start the FDCAN bus */
  if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
  {
    return FDCAN_STATUS_ERROR;
  }

  /* Activate the FIFO interrupt for respective fdcan controller */
  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    return FDCAN_STATUS_ERROR;
  }

  return FDCAN_STATUS_OK;
}

STATUS FDCAN_addCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, FDCANCallback *processData)
{
  /* Check if we have space to add a new Callback */
  if (FDCAN_listCallbacksCount >= FDCAN_MAX_CALLBACKS) {
      return FDCAN_STATUS_MAX_LIMIT_REACHED;
  }

  /* Check if identifier already exists */
  for (uint32_t i = 0; i < FDCAN_listCallbacksCount; i++) {
      if (FDCAN_listCallbacks[i].identifier == identifier) {
          return FDCAN_STATUS_ERROR;
      }
  }

  /* Add the callback to the list and update the count */
  FDCAN_listCallbacks[FDCAN_listCallbacksCount] = (FDCANCallback_internal_t) {
      .fp = processData,
      .identifier = identifier
  };
  FDCAN_listCallbacksCount++;

  return FDCAN_STATUS_OK;
}

STATUS FDCAN_sendData(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t TxData[64])
{
  FDCAN_TxHeaderTypeDef TxHeader;

  /* Set the identifier to send */
  TxHeader.Identifier = identifier;

  /* Set the data length to send */
  TxHeader.DataLength = FDCAN_DLC_BYTES_64;

  /* Set the identifier type to send */
  TxHeader.IdType = FDCAN_STANDARD_ID;

  /* Set the frame type to send */
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;

  /* Set the error state indicator to send */
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

  /* Set the bit rate switch to send */
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;

  /* Set the FD format to send */
  TxHeader.FDFormat = FDCAN_FD_CAN;

  /* Set the Tx event FIFO control to send */
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

  /* Set the message marker to send */
  TxHeader.MessageMarker = 0;

  /* Set the message to send */
  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
  {
    return FDCAN_STATUS_ERROR;
  }

  return FDCAN_STATUS_OK;
}

/*****************************************************************************/
/** Local function definitions                                              **/
/*****************************************************************************/

/* This function overwrites _weak function definition in STM32 HAL FDCAN driver when linking project*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[64];

  /* Fetch the CAN message from the FIFO buffer*/
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }

  /* check if we have a callback for the identifier */
  for (uint32_t i = 0; i < FDCAN_listCallbacksCount; i++) {
      if (FDCAN_listCallbacks[i].identifier == RxHeader.Identifier) {
          FDCAN_listCallbacks[i].fp(RxData);
      }
  }
}
