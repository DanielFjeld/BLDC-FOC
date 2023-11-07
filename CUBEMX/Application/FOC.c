/*
 * FOC.c
 *
 *  Created on: Oct 30, 2023
 *      Author: Daniel
 */
#include "main.h"
#include "FOC.h"
#include "tim.h"
#include "adc.h"

#include "cmsis_os2.h"

#include "current_ADC.h"
#include "print_server.h"

void test_thread(void);
osThreadId_t test_thread_id;
osThreadAttr_t test_attr;
void FOC_init(void){
	test_attr.name = "Test thread";
	test_attr.priority = 10;
	test_thread_id = osThreadNew((void *)test_thread, NULL, &test_attr);

	//Print_Server (USART communication)
	PrintServerInit(); //Priority 5

	//Current measure thread
	current_init(); //Priority 0

	//Encoder 1 and 2 thread
	//encoder_init(); //Priority 1

	//CTRL thread (PWM generation, fault handling)
	CTRL_init(); //Priority 2

	//MISC thread (measure voltage and temperature with ADC and set status LED)
	//MISC_init(); //Priority 3

	//CAN thread
	//CAN_init(); //Priority 4


}

void test_thread(void){
	while(1){
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		osDelay(100);
		HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 1);
		osDelay(100);
//		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
//		osDelay(100);
//		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
		osDelay(100);
		HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 0);
		osDelay(100);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		osDelay(500);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	//set flag to indicate end of conversion
	if(hadc == &hadc1)ADC1_ConvCpltCallback(hadc);
}
