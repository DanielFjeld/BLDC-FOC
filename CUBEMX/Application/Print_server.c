/*
 * Print_server.c
 *
 *  Created on: Nov 11, 2023
 *      Author: Daniel
 */
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "print_server.h"
/* Define print buffer size */
#define PRINT_BUFFER_SIZE 256
/* For convenience, define a pointer to UART handle */
UART_HandleTypeDef *uart_handle_p = &huart1;

volatile uint8_t buff[PRINT_BUFFER_SIZE];
void PrintServerPrintf(const char *fmt, ...)
{
/* local variable to store the final text string to be sent to UART */

/* va_list is a type to hold information about variable arguments */
va_list args;
/* va_start must be called before accessing variable argument list */
va_start(args, fmt);
/* The vsnprintf() function formats and stores a series of characters and
* values in the buffer target-string. The vsnprintf() function works just
* like the snprintf() function, except that arg_ptr points to a list of
* arguments whose number can vary from call to call in the program. These
* arguments should be initialized by the va_start function for each call.
* In contrast, the snprintf() function can have a list of arguments, but
* the number of arguments in that list is fixed when you compile the
* program.
*
* The vsnprintf() function converts each entry in the argument list
* according to the corresponding format specifier in format. The format has
* the same form and function as the format string for the printf()
* function. */
vsnprintf((char *)buff, PRINT_BUFFER_SIZE, fmt, args);
/* va_end should be executed before the function returns whenever
* va_start has been previously used in that function */
va_end(args);
HAL_UART_Transmit_DMA(uart_handle_p, (uint8_t *)buff, strlen((char *)buff));
}
/* End of print_server.c */

