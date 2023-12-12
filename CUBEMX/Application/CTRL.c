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


#include "CURRENT_adc.h"

//PWM setup
#define duty_max 1499
#define pi 3.1415926535

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
}

//sin(θ◦) ≈ 4θ(180 − θ) 40500 − θ(180 − θ);
//float _sin(float deg){
//	return (4*deg*(180-deg)/(40500 - deg*(180-deg)));
//}

void inverter(int16_t angle, uint16_t voltage, uint8_t direction){
	angle = (angle+360*2)%360;
//	angle = (360 - angle);
//	angle = angle%360;
	uint32_t compare_M1 = 0;
	uint32_t compare_M2 = 0;
	uint32_t compare_M3 = 0;

	float deg = (float)(angle%60);
	//uint32_t T1 = (uint32_t )(duty_max*duty*((240-4*deg)*(120-deg)/(40500 - (60-deg)*(120-deg))) ); //*pi/180
	uint16_t T1 = (uint16_t)(voltage*(float)( 4*(60-deg)*(180-(60-deg))/(40500 - (60-deg)*(180-(60-deg))))); //*pi/180
	uint16_t T2 = (uint16_t)(voltage*(float)(4*deg*(180-deg)/(40500 - deg*(180-deg))));
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
	else if(angle >= 300 && angle <= 360){
		compare_M1 = T0+T1;
		compare_M2 = T0;
		compare_M3 = T0+T1+T2;
		}
//	PrintServerPrintf("OK %d %d %d %d\r\n", (uint32_t)(compare_M1), (uint32_t)(compare_M2), (uint32_t)(compare_M3), (int32_t)angle);
	TIM1->CCR1 = compare_M1;
	if(direction){
		TIM1->CCR2 = compare_M3;
		TIM1->CCR3 = compare_M2;
	}
	else{
		TIM1->CCR2 = compare_M2;
		TIM1->CCR3 = compare_M3;
	}


//	dac_value(angle*8+200);

}
void shutoff(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
}
void shutdown(void){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); //error
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); //error

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); //error
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); //error

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); //error
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); //error

	HAL_TIM_Base_Stop(&htim1);
}
