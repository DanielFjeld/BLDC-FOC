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

//ADC setup
#define number_of_oversample 16 //times two
#define number_of_channels 4
#define number_of_calibration_points 10000

//DMA data
volatile uint32_t adc_result_DMA[number_of_channels*2];

//Vrefint calibration
uint16_t *vrefint = (uint16_t*)0x1FFF75AA;

//Current mA
volatile uint8_t Flag = 0;
volatile uint32_t Voltage_offset[3] = {0};
volatile int16_t VDDA = 0;

Current_Callback IRQ_callback;
Current data;

#define Vref 3000
#define ADC_RES 4095

int32_t ADC_CAL_init(ADC_HandleTypeDef *hadc){
	/*
	 * if a negative number is returned, an error has occurred
	 * else it will return the VREFINT in mV
	 *
	 * error codes
	 * 	-1 	null pointer error
	 *  -2 	ADC calibration failed
	 *  -3 	ADC start failed
	 *  -4 	ADC timeout
	 *  -5 	ADC failed
	 *  -6  ADC stop failed
	 */

	HAL_StatusTypeDef HAL_status;
	ADC_HandleTypeDef *adc_handle_CAL;

	//check if pointer is NULL
	if(hadc == NULL)return -1; //null pointer error

	//copy pointer
	adc_handle_CAL = hadc;

	//do a calibration
	HAL_status = HAL_ADCEx_Calibration_Start(adc_handle_CAL, ADC_SINGLE_ENDED);
	if (HAL_status != HAL_OK)return -2; //ADC calibration failed

	//start ADC
	uint16_t calibrating = number_of_calibration_points;
	volatile uint32_t Voltage_offset_temp[3] = {0};
	ADC_ChannelConfTypeDef sConfig = {0};
	while(calibrating){
		sConfig.Channel = ADC_CHANNEL_0;
		HAL_status = HAL_ADC_ConfigChannel(adc_handle_CAL, &sConfig); // required for code to sample both channels, why?
		if (HAL_status != HAL_OK)return -3; //ADC start failed
		HAL_status = HAL_ADC_Start(adc_handle_CAL);
		if (HAL_status != HAL_OK)return -3; //ADC start failed
		//whait until the ADC has gotten the value
		HAL_status = HAL_ADC_PollForConversion(adc_handle_CAL, 1000);
		if (HAL_status == HAL_TIMEOUT)return -4; //ADC timeout
		if (HAL_status != HAL_OK)return -5; //ADC failed
		uint32_t M3_raw = HAL_ADC_GetValue(adc_handle_CAL);
		HAL_status = HAL_ADC_Stop(adc_handle_CAL);
		if (HAL_status != HAL_OK)return -6; //ADC start failed

		sConfig.Channel = ADC_CHANNEL_1;
		HAL_status = HAL_ADC_ConfigChannel(adc_handle_CAL, &sConfig); // required for code to sample both channels, why?
		if (HAL_status != HAL_OK)return -3; //ADC start failed
		HAL_status = HAL_ADC_Start(adc_handle_CAL);
		if (HAL_status != HAL_OK)return -3; //ADC start failed
		//whait until the ADC has gotten the value
		HAL_status = HAL_ADC_PollForConversion(adc_handle_CAL, 1000);
		if (HAL_status == HAL_TIMEOUT)return -4; //ADC timeout
		if (HAL_status != HAL_OK)return -5; //ADC failed
		uint32_t M2_raw = HAL_ADC_GetValue(adc_handle_CAL);
		HAL_status = HAL_ADC_Stop(adc_handle_CAL);
		if (HAL_status != HAL_OK)return -6; //ADC start failed

		sConfig.Channel = ADC_CHANNEL_2;
		HAL_status = HAL_ADC_ConfigChannel(adc_handle_CAL, &sConfig); // required for code to sample both channels, why?
		if (HAL_status != HAL_OK)return -3; //ADC start failed
		HAL_status = HAL_ADC_Start(adc_handle_CAL);
		if (HAL_status != HAL_OK)return -3; //ADC start failed
		//whait until the ADC has gotten the value
		HAL_status = HAL_ADC_PollForConversion(adc_handle_CAL, 1000);
		if (HAL_status == HAL_TIMEOUT)return -4; //ADC timeout
		if (HAL_status != HAL_OK)return -5; //ADC failed
		uint32_t M1_raw = HAL_ADC_GetValue(adc_handle_CAL);
		HAL_status = HAL_ADC_Stop(adc_handle_CAL);
		if (HAL_status != HAL_OK)return -6; //ADC start failed

		sConfig.Channel = ADC_CHANNEL_3;
		HAL_status = HAL_ADC_ConfigChannel(adc_handle_CAL, &sConfig); // required for code to sample both channels, why?
		if (HAL_status != HAL_OK)return -3; //ADC start failed
		HAL_status = HAL_ADC_Start(adc_handle_CAL);
		if (HAL_status != HAL_OK)return -3; //ADC start failed
		//whait until the ADC has gotten the value
		HAL_status = HAL_ADC_PollForConversion(adc_handle_CAL, 1000);
		if (HAL_status == HAL_TIMEOUT)return -4; //ADC timeout
		if (HAL_status != HAL_OK)return -5; //ADC failed
		uint32_t vdda_raw = HAL_ADC_GetValue(adc_handle_CAL);
		HAL_status = HAL_ADC_Stop(adc_handle_CAL);
		if (HAL_status != HAL_OK)return -6; //ADC start failed
		//get current samples;
		VDDA = (int16_t)3000*(*vrefint)/(vdda_raw/number_of_oversample);
		Voltage_offset_temp[0] += (int32_t)((M1_raw/number_of_oversample*VDDA)/4095)*153/100; //*153/100
		Voltage_offset_temp[1] += (int32_t)((M2_raw/number_of_oversample*VDDA)/4095)*153/100;
		Voltage_offset_temp[2] += (int32_t)((M3_raw/number_of_oversample*VDDA)/4095)*153/100;

		calibrating--;

		if(!calibrating){
			Voltage_offset[0] = Voltage_offset_temp[0]/number_of_calibration_points;
			Voltage_offset[1] = Voltage_offset_temp[1]/number_of_calibration_points;
			Voltage_offset[2] = Voltage_offset_temp[2]/number_of_calibration_points;
		}
	}
	return VDDA; //success
}

void current_init(Current_Callback __IRQ_callback){
	ADC_CAL_init(&hadc1);

	IRQ_callback = __IRQ_callback;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_result_DMA, number_of_channels*2);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[3]/number_of_oversample);
	data.Current_M1 = -(int32_t)((((adc_result_DMA[2]/number_of_oversample*VDDA)/4095)*153/100)-Voltage_offset[0])*50;
	data.Current_M2 = -(int32_t)((((adc_result_DMA[1]/number_of_oversample*VDDA)/4095)*153/100)-Voltage_offset[1])*50;
	data.Current_M3 = -(int32_t)((((adc_result_DMA[0]/number_of_oversample*VDDA)/4095)*153/100)-Voltage_offset[2])*50;
	data.Current_DC = (uint32_t)((abs((int)data.Current_M1)+abs((int)data.Current_M2)+abs((int)data.Current_M3))/2);
	IRQ_callback(&data);
}
void ADC1_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[7]/number_of_oversample);
	data.Current_M1 = -(int32_t)((((adc_result_DMA[6]/number_of_oversample*VDDA)/4095)*153/100)-Voltage_offset[0])*50;
	data.Current_M2 = -(int32_t)((((adc_result_DMA[5]/number_of_oversample*VDDA)/4095)*153/100)-Voltage_offset[1])*50;
	data.Current_M3 = -(int32_t)((((adc_result_DMA[4]/number_of_oversample*VDDA)/4095)*153/100)-Voltage_offset[2])*50;
	data.Current_DC = (uint32_t)((abs((int)data.Current_M1)+abs((int)data.Current_M2)+abs((int)data.Current_M3))/2);
	IRQ_callback(&data);
}

//	SetMode(&Current_PID2,  AUTOMATIC);

//--------------------------------TEST--------------------------------------
//	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//	uint32_t time_to_die = 0;
//	uint32_t melody_time = 0;
//	uint8_t melody_index = 0;
//	float increment = 0.01;
//	int wholenote = (60000 * 2) / tempo;
//	int divider = melody[melody_index + 1];
//	int noteDuration = 0;

	//-----------------CAN----------
//	FDCAN_addCallback(&hfdcan1, 0x11, test);
//	FDCAN_Start(&hfdcan1);
//
//	FDCAN_sendData(&hfdcan1, 0x33, TxData);
//
//	int NUM_NOTES = sizeof(melody) / sizeof(melody[0]) / 2;

//	while(1){

		//HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);

		//wait until callback
		//uint32_t thread_flag = 0;
		//thread_flag = osThreadFlagsWait (adc_cplt_flag | adc_half_cplt_flag , osFlagsWaitAny, osWaitForever);
//		if(!half_flag && !full_flag)continue; //return to While(1);
//		half_flag = 0;
//		full_flag = 0;
//		HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 1);
//		HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 0);
		//results in buffer now



		//test time
//		time_to_die++;


		//----------------------test rotation UP and DOWN--------------------------
//		Current_PID.Setpoint = 2500;
//		Angle += increment;
//		if(Angle >= 360){
//			Angle -= 360;
//		}
//
//		if(time_to_die%500 == 0 && time_to_die < 25000*5){
//			if(increment < 1.5)increment+=0.01;
//		}
//
//		else if(time_to_die%500 == 0){
//			increment-=0.01;
//			if(increment < 0){
//				shutoff();
//				shutdown();
//				while(1);
//			}
//		}

		//----------------------test rotation from CAN--------------------------

//		if(speed < speed_lpf)speed += 0.0001;
//		if(speed > speed_lpf)speed -= 0.0001;
//		Current_PID.Setpoint = 2000;
//		Angle += speed;
//		if(Angle >= 360){
//			Angle -= 360;
//		}

		//----------------------test rotation 2500 deg/sec--------------------------

//		Current_PID.Setpoint = 10000;
//		Angle += 1;
//		if(Angle >= 360){
//			Angle -= 360;
//		}

		//--------------------------Music-------------------------------
//		Current_PID.Setpoint = 5000;
//
//		uint16_t freq = 0;
//		if (divider > 0) {
//		  // regular note, just proceed
//		  noteDuration = (wholenote) / divider;
//		} else if (divider < 0) {
//		  // dotted notes are represented with negative durations!!
//		  noteDuration = (wholenote) / abs(divider);
//		  noteDuration *= 1.5; // increases the duration in half for dotted notes
//		}
//
//		if((time_to_die-melody_time)-noteDuration*25*0.9 >=  noteDuration*25){
//			if ((time_to_die-melody_time) >=  noteDuration*25){
//				melody_index += 2;
//				melody_time= time_to_die;
//			}
//		}
//		else freq = melody[melody_index];
//
//		if(melody_index >= NUM_NOTES){
//			shutoff();
//			shutdown();
//			while(1); //done playing
//		}
//		if(freq == 0)Angle = 0;
//		else if(!(time_to_die%frequenzy(freq)) && Angle == 0)Angle = 30;
//		else if(!(time_to_die%frequenzy(freq)) && Angle == 30)Angle = 0;


		//-------------------Change Current setpiont------------------------------
//		if(time_to_die > 25000*1)Current_PID.Setpoint = 400;
//		if(time_to_die > 25000*2)Current_PID.Setpoint = 0;
//		if(time_to_die > 25000*3)Current_PID.Setpoint = 400;
//		if(time_to_die > 25000*4)Current_PID.Setpoint = 0;


//		if(DC_current == Current_limit){
//			shutoff();
//			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
//			while(1);
//		}

		//----------------------------Current PID loop------------------------------
//		Current_PID2.Input = (double)DC_current;
//		Compute(&Current_PID2);
//
//		if(Voltage_offset[0] == 0)shutoff();
//		else inverter((uint16_t)Angle, Current_PID2.Output);
//		//osThreadFlagsSet(CTRL_thread_id, update_flag);
//
//		TxData[0] = DC_current & 0xff;
//		TxData[1] = (DC_current>>8) & 0xff;
//
//
//		if(time_to_die%50000 == 0)FDCAN_sendData(&hfdcan1, 0x33, TxData);
//		//DAC for measuring fast signals
//		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)Current_PID2.Input*4096/VDDA);
//		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		//osDelay(1000);
//	}
