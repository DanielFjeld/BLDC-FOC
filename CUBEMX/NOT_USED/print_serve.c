/*
 * print_serve.c
 *
 *  Created on: Aug 24, 2021
 *      Author: Daniel
 */
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "print_serve.h"
#include "print_server.h"

void PrintServerPrintf(const char *fmt, ...)
{
	va_list args;
	pool_mem *mem_ptr;

	//allocate a memory pool
	mem_ptr = (pool_mem *)osMemoryPoolAlloc(pool_id, 0U);
	if (mem_ptr != NULL) { //if there was a pool available
		va_start(args, fmt);
		vsnprintf((char *)mem_ptr->Buf, print_size, fmt, args);

		//get length of strin
		mem_ptr->Idx = strlen((char *)mem_ptr->Buf);
		va_end(args);

		//put a message with pointer to pool
		osStatus_t status =osMessageQueuePut(print_message_queue_id, &mem_ptr, 0U, 0U);
		if(status != osOK){
			osMemoryPoolFree(pool_id, mem_ptr);
		}
	}
}
