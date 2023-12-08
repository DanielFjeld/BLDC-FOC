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

uint16_t calibrating = 0;
volatile uint32_t Voltage_offset_temp[3] = {0};
void ADC_CAL(){
	VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[3]/number_of_oversample);
	Voltage_offset_temp[0] += (int32_t)((adc_result_DMA[2]/number_of_oversample*VDDA)/4095)*153/100; //*153/100
	Voltage_offset_temp[1] += (int32_t)((adc_result_DMA[1]/number_of_oversample*VDDA)/4095)*153/100;
	Voltage_offset_temp[2] += (int32_t)((adc_result_DMA[0]/number_of_oversample*VDDA)/4095)*153/100;
	calibrating--;

	if(!calibrating){
		Voltage_offset[0] = Voltage_offset_temp[0]/number_of_calibration_points;
		Voltage_offset[1] = Voltage_offset_temp[1]/number_of_calibration_points;
		Voltage_offset[2] = Voltage_offset_temp[2]/number_of_calibration_points;
	}
}

void current_init(Current_Callback __IRQ_callback){
	//ADC_CAL_init(&hadc1);
	calibrating = number_of_calibration_points;

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
		data.Current_M1 = -(int32_t)(((((int32_t)adc_result_DMA[2]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[0])*50;
		data.Current_M2 = -(int32_t)(((((int32_t)adc_result_DMA[1]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[1])*50;
		data.Current_M3 = -(int32_t)(((((int32_t)adc_result_DMA[0]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[2])*50;
		data.Current_DC = ((abs((int)data.Current_M1)+abs((int)data.Current_M2)+abs((int)data.Current_M3))/2);
		Curent_IRQ_callback(&data);
	}
	if (hadc == &hadc2){
		VT_data.Temp_NTC1 = (VT_adc_result_DMA[0]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.Temp_NTC2 = (VT_adc_result_DMA[1]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.V_Bat = (VT_adc_result_DMA[2]/number_of_VT_oversample*VDDA*34)/ADC_RES;
		VT_data.V_aux = (VT_adc_result_DMA[3]/number_of_VT_oversample*VDDA*57)/ADC_RES/10;
		VT_IRQ_callback(&VT_data);
	}
	if(calibrating)ADC_CAL();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc == &hadc1){
		VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[7]/number_of_oversample);
		data.Current_M1 = -(int32_t)(((((int32_t)adc_result_DMA[6]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[0])*50;
		data.Current_M2 = -(int32_t)(((((int32_t)adc_result_DMA[5]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[1])*50;
		data.Current_M3 = -(int32_t)(((((int32_t)adc_result_DMA[4]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[2])*50;
		data.Current_DC = ((abs((int)data.Current_M1)+abs((int)data.Current_M2)+abs((int)data.Current_M3))/2);
		Curent_IRQ_callback(&data);
	}
	if (hadc == &hadc2){
		VT_data.Temp_NTC1 = (VT_adc_result_DMA[4]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.Temp_NTC2 = (VT_adc_result_DMA[5]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.V_Bat = (VT_adc_result_DMA[6]/number_of_VT_oversample*VDDA*34)/ADC_RES;
		VT_data.V_aux = (VT_adc_result_DMA[7]/number_of_VT_oversample*VDDA*57)/ADC_RES/10;
		VT_IRQ_callback(&VT_data);
	}
}

void dac_value(uint16_t V_dac){
	uint16_t dac_value = ((V_dac*ADC_RES)/VDDA);
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
}
