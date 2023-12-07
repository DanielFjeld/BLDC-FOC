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

#include "dac.h"

//ADC setup
#define ADC_RES 4095 //times two
#define number_of_calibration_points 1000


#define number_of_oversample 16 //times two
#define number_of_VT_oversample 16 //times two
#define number_of_channels 4
#define number_of_VT_channels 4

//DMA data
volatile uint32_t adc_result_DMA[number_of_channels*2]; //current
volatile uint32_t VT_adc_result_DMA[number_of_VT_channels*2]; //voltage temperature

//Vrefint calibration
uint16_t *vrefint = (uint16_t*)0x1FFF75AA;

//Current mA
volatile uint8_t Flag = 0;
volatile uint32_t Voltage_offset[3] = {0};
volatile int16_t VDDA = 0;

Current_Callback Curent_IRQ_callback;
Current data;

VT_Callback VT_IRQ_callback;
Voltage_Temp VT_data;

#define Vref 3000

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

	calibrating = 0;

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
	Voltage_offset[0] = 2400;
	Voltage_offset[1] = 2400;
	Voltage_offset[2] = 2400;


	//---------------DAC DEBUG-------------
	HAL_DAC_Init(&hdac1);
	HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);


	return VDDA; //success
}

void current_init(Current_Callback __IRQ_callback){
	ADC_CAL_init(&hadc1);

	Curent_IRQ_callback = __IRQ_callback;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_result_DMA, number_of_channels*2);
}
void voltage_temperature_init(VT_Callback __IRQ_callback){
	VT_IRQ_callback = __IRQ_callback;

	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)VT_adc_result_DMA, number_of_VT_channels*2);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == &hadc1){
		VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[3]/number_of_oversample);
		data.Current_M1 = -(int32_t)((((adc_result_DMA[2]/number_of_oversample*VDDA)/ADC_RES)*153/100)-Voltage_offset[0])*50;
		data.Current_M2 = -(int32_t)((((adc_result_DMA[1]/number_of_oversample*VDDA)/ADC_RES)*153/100)-Voltage_offset[1])*50;
		data.Current_M3 = -(int32_t)((((adc_result_DMA[0]/number_of_oversample*VDDA)/ADC_RES)*153/100)-Voltage_offset[2])*50;
		data.Current_DC = ((abs((int)data.Current_M1)+abs((int)data.Current_M2)+abs((int)data.Current_M3))/2);
		Curent_IRQ_callback(&data);
	}
	if (hadc == &hadc2){
		VT_data.Temp_NTC1 = (VT_adc_result_DMA[0]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.Temp_NTC2 = (VT_adc_result_DMA[1]/number_of_VT_oversample*VDDA)/ADC_RES;
//		VT_data.V_Bat = (VT_adc_result_DMA[2]/number_of_VT_oversample*VDDA*360/3.3)/ADC_RES;
		VT_data.V_aux = (VT_adc_result_DMA[3]/number_of_VT_oversample*VDDA*57)/ADC_RES/10;
		VT_IRQ_callback(&VT_data);
	}
//	dac_value(data.Current_DC);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc == &hadc1){
		VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[7]/number_of_oversample);
		data.Current_M1 = -(int32_t)((((adc_result_DMA[6]/number_of_oversample*VDDA)/ADC_RES)*153/100)-Voltage_offset[0])*50;
		data.Current_M2 = -(int32_t)((((adc_result_DMA[5]/number_of_oversample*VDDA)/ADC_RES)*153/100)-Voltage_offset[1])*50;
		data.Current_M3 = -(int32_t)((((adc_result_DMA[4]/number_of_oversample*VDDA)/ADC_RES)*153/100)-Voltage_offset[2])*50;
		data.Current_DC = ((abs((int)data.Current_M1)+abs((int)data.Current_M2)+abs((int)data.Current_M3))/2);
		Curent_IRQ_callback(&data);
	}
	if (hadc == &hadc2){
		VT_data.Temp_NTC1 = (VT_adc_result_DMA[4]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.Temp_NTC2 = (VT_adc_result_DMA[5]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.V_Bat = (VT_adc_result_DMA[6]/number_of_VT_oversample*360000)/ADC_RES;
		VT_data.V_aux = (VT_adc_result_DMA[7]/number_of_VT_oversample*VDDA*57)/ADC_RES/10;
		VT_IRQ_callback(&VT_data);
	}
//	 dac_value(data.Current_DC);

}

void dac_value(uint16_t V_dac){
	uint16_t dac_value = ((V_dac*ADC_RES)/VDDA);
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
}
