/**
 * @file fdcandriver.h
 * @author Simen Bjerkestrand (Simen.Bjerkestrand@propulsentnu.no)
 * @brief simple FDCAN driver for classic CAN with Fifo0
 * @version 1.0
 * @date 2023-11-04
 * 
 * @copyright Propulse NTNU (c) 2023
 * 
 */

#ifndef __FDCANDRIVER_H__
#define __FDCANDRIVER_H__

/*****************************************************************************/
/** ANSI C libraries                                                        **/
/*****************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/*****************************************************************************/
/** Module includes                                                         **/
/*****************************************************************************/
#include "fdcan.h"

/*****************************************************************************/
/** Module defines                                                          **/
/*****************************************************************************/
#define STATUS uint32_t
#define FDCAN_MAX_CALLBACKS               10

/*****************************************************************************/
/** Module status codes                                                     **/
/*****************************************************************************/
#define FDCAN_STATUS_OK                   0x00
#define FDCAN_STATUS_ERROR                0x01
#define FDCAN_STATUS_INVALID_ARG          0x02
#define FDCAN_STATUS_MAX_LIMIT_REACHED    0x03

/*****************************************************************************/
/** Module data types                                                       **/
/*****************************************************************************/
typedef void (*FDCANCallback) (uint8_t RxData[8]);

/*****************************************************************************/
/** Public API                                                              **/
/*****************************************************************************/

/**
 * @brief Start the FDCAN controller
 *
 * @param hdfcan the FDCAN controller to start
 * @return The status of the operation.
 */
STATUS FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Add a callback function to the FDCAN controller
 *
 * @param hfdcan the FDCAN controller to add the callback to
 * @param identifier the identifier to add the callback to
 * @param processData the callback function to add
 * @return The status of the operation.
 */
STATUS FDCAN_addCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, FDCANCallback *processData);

/**
 * @brief Send data on the FDCAN bus
 *
 * @param hfdcan the FDCAN controller to send data on
 * @param identifier the identifier to send data on
 * @param TxData the data to send
 * @return
 */
STATUS FDCAN_sendData(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t TxData[8]);

#endif // __FDCANDRIVER_H__
