/*
 * current_ADC.c
 *
 *  Created on: Oct 30, 2023
 *      Author: Daniel
 */
#include "main.h"
#include <stdlib.h>
#include "current_ADC.h"
#include "adc.h"
#include "dma.h"
#include "cmsis_os2.h"

#include "CTRL.h"
#include "print_serve.h"
#include "PID.h"

#include "dac.h"
#include "music.h"

//thread setup
void current_thread(void);
osThreadId_t current_thread_id;
osThreadAttr_t current_attr;

//ADC setup
#define number_of_oversample 16 //times two
#define number_of_channels 4
#define adc_half_cplt_flag 1
#define adc_cplt_flag 2

volatile uint32_t adc_result_DMA[number_of_channels*2];

//Vrefint calibration
uint16_t *vrefint = (uint16_t*)0x1FFF75AA;

//Current mA
#define lpf 100
volatile uint32_t Voltage_offset[3] = {0};
volatile uint32_t Voltage_offset_temp[3] = {0};
volatile int32_t current[3] = {0};
volatile int16_t VDDA = 0;
volatile uint8_t half_flag = 0;
volatile uint8_t full_flag = 0;

uint32_t lpf_index = 0;
//int32_t current_half[3] = {0};
//int32_t current_full[3] = {0};


//DMA callback flags



//Current PID
PID_instance Current_PID = {0};
float Angle;
double Kp = 0.05;
double Ki = 100;
double Kd = 0.0;
#define sampletime 26
#define max_voltage 1499
#define min_voltage 0

//Safety
#define Current_limit 2000 //mA
#define Temperature_limit 40 //degrees C

//initialization
void current_init(void){
	//create thread
	current_attr.name = "Current thread";
	//current_attr.priority = 9;
	current_thread_id = osThreadNew((void *)current_thread, NULL, &current_attr);
}


uint16_t frequenzy(uint16_t freq){
	if (freq == 0)return !0;
	return 12500/freq;
}

//thread
void current_thread(void){
	CTRL_init_PWM();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_result_DMA, number_of_channels*2);

	uint32_t DC_current = 0;
	SetSampleTime(&Current_PID, sampletime);
	SetTunings(&Current_PID, Kp, Ki, Kd, 1);
	SetOutputLimits(&Current_PID, min_voltage, max_voltage);
	SetControllerDirection(&Current_PID, DIRECT);
	SetMode(&Current_PID,  MANUAL);
	Initialize(&Current_PID);


	//--------------------------------TEST--------------------------------------
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	uint32_t time_to_die = 0;
	uint32_t melody_time = 0;
	uint8_t melody_index = 0;
	float increment = 0.01;

	uint8_t calibrating = 100;
	while(calibrating){



		if(!half_flag && !full_flag)continue; //return to While(1);
		half_flag = 0;
		full_flag = 0;

		Voltage_offset_temp[0] += current[0];
		Voltage_offset_temp[1] += current[1];
		Voltage_offset_temp[2] += current[2];

		calibrating--;

		if(!calibrating){
			Voltage_offset[0] = -Voltage_offset_temp[0]/50/lpf;
			Voltage_offset[1] = -Voltage_offset_temp[1]/50/lpf;
			Voltage_offset[2] = -Voltage_offset_temp[2]/50/lpf;
		}
	}

	SetMode(&Current_PID,  AUTOMATIC);
	while(1){
		//start DMA ADC converting


		//HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);

		//wait until callback
		//uint32_t thread_flag = 0;
		//thread_flag = osThreadFlagsWait (adc_cplt_flag | adc_half_cplt_flag , osFlagsWaitAny, osWaitForever);
		if(!half_flag && !full_flag)continue; //return to While(1);
		half_flag = 0;
		full_flag = 0;
		HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 1);
		HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 0);
		//results in buffer now

		DC_current = (uint32_t)((abs((int)current[0])+abs((int)current[1])+abs((int)current[2]))/2);
		int32_t DC_current_offset = (current[0]+current[1]+current[2])/3;




			//lpf_index = 0;
			//PrintServerPrintf("Set%4d %4d %4d\r\n", (int32_t)Current_PID.Setpoint, (int32_t)Current_PID.Input, (int32_t)Current_PID.Output);
			//PrintServerPrintf("M1:%5dmA M2:%5dmA M3:%5dmA DC:%4d Vref:%4dmV\r\n", current[0], current[1], current[2], DC_current, VDDA+2);
		//}

		//test time
		time_to_die++;


		//----------------------test rotation--------------------------
//		Current_PID.Setpoint = 500;
//		if(!(time_to_die%1))Angle += increment;
//		if(Angle >= 360 && time_to_die < 25000*15){
//			if(increment < 3.5)increment+=0.01;
//			Angle -= 360;
//		}
//		else if(Angle >= 360 && time_to_die < 25000*30){
//			if(increment > 0)increment-=0.01;
//			if(increment < 0.1){
//				shutoff();
//				while(1);
//			}
//			Angle -= 360;
//		}

		//--------------------------Music-------------------------------
		Current_PID.Setpoint = 300;
		uint16_t freq = 0;
		if((time_to_die-melody_time) >=  (melody[melody_index][1]*25)){
			if ((time_to_die-melody_time)+25 >=  (melody[melody_index][1]*25)){
				melody_index++;
				melody_time= time_to_die;
			}
		}
		else freq = melody[melody_index][0];

		if(melody_index == NUM_NOTES){
			shutoff();
			while(1); //done playing
		}
		if(freq == 0)Angle = 0;
		else if(!(time_to_die%frequenzy(freq)) && Angle == 0)Angle = 90;
		else if(!(time_to_die%frequenzy(freq)) && Angle == 90)Angle = 0;


		//-------------------Change Current setpiont------------------------------
//		if(time_to_die > 25000*1)Current_PID.Setpoint = 400;
//		if(time_to_die > 25000*2)Current_PID.Setpoint = 0;
//		if(time_to_die > 25000*3)Current_PID.Setpoint = 400;
//		if(time_to_die > 25000*4)Current_PID.Setpoint = 0;


		if(DC_current == Current_limit){
			shutoff();
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
			while(1);
		}


		//----------------------------Current PID loop------------------------------
		Current_PID.Input = (double)DC_current;
		Compute(&Current_PID);

		if(Voltage_offset[0] == 0)shutoff();
		else inverter((uint16_t)Angle, Current_PID.Output);
		//osThreadFlagsSet(CTRL_thread_id, update_flag);


		//DAC for measuring fast signals
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)Current_PID.Input*4096/VDDA);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		//osDelay(10);

	}
}

//test

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
	VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[3]/number_of_oversample);
	current[0] = -(int32_t)(adc_result_DMA[2]/number_of_oversample*(VDDA*153/100/4095)-Voltage_offset[0])*50;
	current[1] = -(int32_t)(adc_result_DMA[1]/number_of_oversample*(VDDA*153/100/4095)-Voltage_offset[1])*50;
	current[2] = -(int32_t)(adc_result_DMA[0]/number_of_oversample*(VDDA*153/100/4095)-Voltage_offset[2])*50;
	half_flag = 1;
}
void ADC1_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
	VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[7]/number_of_oversample);
	current[0] = -(int32_t)(adc_result_DMA[6]/number_of_oversample*(VDDA*153/100/4095)-Voltage_offset[0])*50;
	current[1] = -(int32_t)(adc_result_DMA[5]/number_of_oversample*(VDDA*153/100/4095)-Voltage_offset[1])*50;
	current[2] = -(int32_t)(adc_result_DMA[4]/number_of_oversample*(VDDA*153/100/4095)-Voltage_offset[2])*50;
	full_flag = 1;
}
