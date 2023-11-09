/*
 * print_server.h
 *
 *  Created on: Sep 21, 2021
 *      Author: Daniel
 */

#ifndef PRINT_SERVER_H_
#define PRINT_SERVER_H_
#include "cmsis_os2.h"

extern osMessageQueueId_t print_message_queue_id;
extern osMemoryPoolId_t pool_id;
#define print_size 64


typedef struct{
	uint8_t Idx;
	uint8_t Buf[print_size];
}pool_mem;

_Bool PrintServerInit(void);

#endif /* PRINT_SERVER_H_ */
