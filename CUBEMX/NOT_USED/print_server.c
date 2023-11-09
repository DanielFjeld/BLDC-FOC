/*
 * print_server.c
 *
 *  Created on: Sep 21, 2021
 *      Author: Daniel
 */
#include "main.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
 #include "cmsis_os2.h"

#define size_of_pool_and_queue 32
#define print_size 64

osMessageQueueId_t print_message_queue_id;
osMemoryPoolId_t pool_id;

osMessageQueueAttr_t message_queue_attr;
osThreadAttr_t print_server_thread_attr;
osThreadId_t thread_id;

UART_HandleTypeDef huart;

typedef struct{
	uint8_t Idx;
	uint8_t Buf[print_size];
}pool_mem;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//create thread flag to signal finish transferring
	osThreadFlagsSet(thread_id, 0x01);
}

void PrintServer(void){
	pool_mem *mem_ptr;
	while(1){
		//get pointer to pool from print_serve
		osMessageQueueGet(print_message_queue_id, &mem_ptr, 0U, osWaitForever);

		if (mem_ptr != NULL) { // mem Block was available
			//print the string that is stored at the pointer location
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)mem_ptr->Buf, mem_ptr->Idx);
			//wait until UART is done transmitting
			osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

			//free the memory pool id
			osStatus_t status = osMemoryPoolFree(pool_id, mem_ptr); // free mem block
			switch (status)  {
				case osOK:
				  break;
				case osErrorParameter:
				  break;
				case osErrorNoMemory:
				  break;
				default:
				  break;
			}
		}
	}
}

_Bool PrintServerInit(void){
	//create memory pool
	pool_id = osMemoryPoolNew (size_of_pool_and_queue, sizeof(pool_mem), NULL);

	//create message queue
	message_queue_attr.name = "message queue printServer";
	print_message_queue_id = osMessageQueueNew(size_of_pool_and_queue, 4, &message_queue_attr);

	//create printServer thread
	print_server_thread_attr.priority = 9;
	print_server_thread_attr.name = "print thread";
	thread_id = osThreadNew((void *)PrintServer, NULL, &print_server_thread_attr);

	return 0;
}
