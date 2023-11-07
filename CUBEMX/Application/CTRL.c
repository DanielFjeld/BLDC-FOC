/*
 * CTRL.c
 *
 *  Created on: Nov 2, 2023
 *      Author: Daniel
 */
#include "main.h"
#include "math.h"
#include <stdlib.h>
#include "CTRL.h"
#include "tim.h"
#include "dma.h"
#include "cmsis_os2.h"

#include "print_serve.h"


//thread setup
void CTRL_thread(void);
osThreadId_t CTRL_thread_id;
osThreadAttr_t CTRL_attr;

//PWM setup
void inverter(uint16_t angle, uint16_t duty);
#define duty_max 1499
#define pi 3.1415926535

//angle PID


//initialization
void CTRL_init(void){
	//create thread
//	CTRL_attr.name = "Control thread";
//	CTRL_attr.priority = 8;
//	CTRL_thread_id = osThreadNew((void *)CTRL_thread, NULL, &CTRL_attr);
}

//thread
void CTRL_init_PWM(void){
	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler(); //error
	if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler(); //error

	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler(); //error
	if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler(); //error

	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler(); //error
	if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler(); //error

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	HAL_TIM_Base_Start_IT(&htim3);

	uint16_t setpoint = 0;

//	while(1){
//
//		osThreadFlagsWait (update_flag, osFlagsWaitAny, osWaitForever);
//		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
////		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
//
//		setpoint += 1;
//		if(setpoint > 360)setpoint -= 360;
////		inverter(setpoint, 1000); //1499 is max voltage
////		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
//		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
//		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
//		//inverter(setpoint, 1000); //1499 is max voltage
//		HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
////		osDelay(1000);
////		inverter(90, 1000); //1499 is max voltage
////		osDelay(1000);
//	}
}

//sin(θ◦) ≈ 4θ(180 − θ) 40500 − θ(180 − θ);
float _sin(float deg){
	return (4*deg*(180-deg)/(40500 - deg*(180-deg)));
}

void inverter(uint16_t angle, uint16_t voltage){

	angle = angle%360;
	uint32_t compare_M1 = 0;
	uint32_t compare_M2 = 0;
	uint32_t compare_M3 = 0;

	float deg = (float)(angle%60);
	//uint32_t T1 = (uint32_t )(duty_max*duty*((240-4*deg)*(120-deg)/(40500 - (60-deg)*(120-deg))) ); //*pi/180
	uint16_t T1 = (uint16_t)(voltage*(float)( 4*(60-deg)*(180-(60-deg))/(40500 - (60-deg)*(180-(60-deg))))); //*pi/180
	uint16_t T2 = (uint16_t)(voltage*(float)(4*deg*(180-deg)/(40500 - deg*(180-deg))) );
	uint16_t T0 = (duty_max-T1-T2)/2;

	if(angle >= 0 && angle < 60){
		compare_M1 = T0;
		compare_M2 = T0+T2;
		compare_M3 = T0+T1+T2;
		}
	else if(angle >= 60 && angle < 120){
		compare_M1 = T0;
		compare_M2 = T0+T1+T2;
		compare_M3 = T0+T1;
		}
	else if(angle >= 120 && angle < 180){
		compare_M1 = T0+T2;
		compare_M2 = T0+T1+T2;
		compare_M3 = T0;
		}
	else if(angle >= 180 && angle < 240){
		compare_M1 = T0+T1+T2;
		compare_M2 = T0+T1;
		compare_M3 = T0;
		}
	else if(angle >= 240 && angle < 300){
		compare_M1 = T0+T1+T2;
		compare_M2 = T0;
		compare_M3 = T0+T2;
		}
	else if(angle >= 300 && angle < 360){
		compare_M1 = T0+T1;
		compare_M2 = T0;
		compare_M3 = T0+T1+T2;
		}

	//PrintServerPrintf("OK %d %d %d %d\r\n", (uint32_t)(compare_M1), (uint32_t)(compare_M2), (uint32_t)(compare_M3), (int32_t)angle);
	TIM1->CCR1 = compare_M1;
	TIM1->CCR2 = compare_M2;
	TIM1->CCR3 = compare_M3;
}
void shutoff(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

}
void tim1_PWM_PulseFinishedCallback(void){
	//osThreadFlagsSet(CTRL_thread_id, update_flag);
}
