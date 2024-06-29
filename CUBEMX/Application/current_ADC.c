/*
 * current_ADC.c
 *
 *  Created on: Oct 30, 2023
 *      Author: Daniel
 */
#include "main.h"
#include "math.h"
#include <stdlib.h>
#include "current_ADC.h"
#include "adc.h"
#include "dma.h"

#include "dac.h"

#include "CORDIC_math.h"

#include "IIR.h"

//1000hz butterworth IIR filter
#define a1 1
#define a2 -1.794090467765525609422638808609917759895
#define a3 0.827108702257972283611309194384375587106
#define gain 0.008254558623111725793042303678248572396

IIR_t LPF_CURRENT_1 = {
		.size = 3,
		.Coef_a = {
			a1,
			a2,
			a3
		},
		.Coef_b = {
			1.0f*gain,
			2.0f*gain,
			1.0f*gain
		},
		.last_x = {0},
		.last_y = {0}
};

IIR_t LPF_CURRENT_2 = {
		.size = 3,
		.Coef_a = {
			a1,
			a2,
			a3
		},
		.Coef_b = {
			1.0f*gain,
			2.0f*gain,
			1.0f*gain
		},
		.last_x = {0},
		.last_y = {0}
};

IIR_t LPF_CURRENT_3 = {
		.size = 3,
		.Coef_a = {
			a1,
			a2,
			a3
		},
		.Coef_b = {
			1.0f*gain,
			2.0f*gain,
			1.0f*gain
		},
		.last_x = {0},
		.last_y = {0}
};

//ADC setup
#define ADC_RES 4095 //times two
#define number_of_calibration_points 100

#define number_of_oversample 16 //times two
#define number_of_VT_oversample 16 //times two
#define number_of_channels 4
#define number_of_VT_channels 4

#define pi 3.14159264f

uint32_t aa_test_ADC = 0;

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
volatile int32_t Voltage_offset_temp[3] = {0};

void dac_value(uint16_t V_dac){
	uint16_t dac_value = ((V_dac*ADC_RES)/VDDA);
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
}

void current_init(Current_Callback __IRQ_callback){
	//ADC_CAL_init(&hadc1);
	calibrating = number_of_calibration_points;

	Curent_IRQ_callback = __IRQ_callback;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_result_DMA, number_of_channels*2);


	HAL_DAC_Init(&hdac1);
	HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
}
void voltage_temperature_init(VT_Callback __IRQ_callback){
	VT_IRQ_callback = __IRQ_callback;

	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)VT_adc_result_DMA, number_of_VT_channels*2);
}
void ADC_CAL(){
	VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[3]/number_of_oversample);
	Voltage_offset_temp[2] += -(int32_t)(((((int32_t)adc_result_DMA[2]/number_of_oversample*VDDA)/ADC_RES)*153/100))*50; //*153/100
	Voltage_offset_temp[1] += -(int32_t)(((((int32_t)adc_result_DMA[1]/number_of_oversample*VDDA)/ADC_RES)*153/100))*50;
	Voltage_offset_temp[0] += -(int32_t)(((((int32_t)adc_result_DMA[0]/number_of_oversample*VDDA)/ADC_RES)*153/100))*50;
	calibrating--;

	if(!calibrating){
		Voltage_offset[0] = Voltage_offset_temp[0]/number_of_calibration_points;
		Voltage_offset[1] = Voltage_offset_temp[1]/number_of_calibration_points;
		Voltage_offset[2] = Voltage_offset_temp[2]/number_of_calibration_points;
	}
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == &hadc1){
		if(calibrating)ADC_CAL();
		else {
			VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[3]/number_of_oversample);
			data.Current_M1 = -(int32_t)(((((int32_t)adc_result_DMA[2]/number_of_oversample*VDDA)/ADC_RES)*153/100))*50-(int32_t)Voltage_offset[2];
			data.Current_M2 = -(int32_t)(((((int32_t)adc_result_DMA[1]/number_of_oversample*VDDA)/ADC_RES)*153/100))*50-(int32_t)Voltage_offset[1];
			data.Current_M3 = -(int32_t)(((((int32_t)adc_result_DMA[0]/number_of_oversample*VDDA)/ADC_RES)*153/100))*50-(int32_t)Voltage_offset[0];

//			data.Current_M1 = (int32_t)(IIR(&LPF_CURRENT_1, (float)((float)data.Current_M1/1000.0f))*1000);
//			data.Current_M2 = (int32_t)(IIR(&LPF_CURRENT_2, (float)((float)data.Current_M2/1000.0f))*1000);
//			data.Current_M3 = (int32_t)(IIR(&LPF_CURRENT_3, (float)((float)data.Current_M3/1000.0f))*1000);


			Curent_IRQ_callback(&data);
		}
	}
	if (hadc == &hadc2){
		aa_test_ADC = VT_adc_result_DMA[3];
		VT_data.Temp_NTC2 = (VT_adc_result_DMA[1]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.V_aux = (float)(VT_adc_result_DMA[2]/number_of_VT_oversample*VDDA*5.7f)/ADC_RES/1000.0f;
		VT_data.V_Bat = (float)(VT_adc_result_DMA[3]/number_of_VT_oversample*VDDA*34.0f)/ADC_RES/1000.0f+1.8f;
		VT_IRQ_callback(&VT_data);
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc == &hadc1 && !calibrating){
//		VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[7]/number_of_oversample);
//		data.Current_M1 = -(int32_t)(((((int32_t)adc_result_DMA[6]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[0])*50;
//		data.Current_M2 = -(int32_t)(((((int32_t)adc_result_DMA[5]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[1])*50;
//		data.Current_M3 = -(int32_t)(((((int32_t)adc_result_DMA[4]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[2])*50;

		//data.Current_M2 = data.Current_M1;
		//data.Current_M1 = (int32_t)(IIR(&LPF_CURRENT_1, (float)((float)data.Current_M1/1000.0f))*1000);
		//data.Current_M2 = (int32_t)(IIR(&LPF_CURRENT_2, (float)((float)data.Current_M2/1000.0f))*1000);
		//data.Current_M3 = (int32_t)(IIR(&LPF_CURRENT_3, (float)((float)data.Current_M3/1000.0f))*1000);

//		Curent_IRQ_callback(&data);
	}
}

void dq0(float theta, float a, float b, float c, float *d, float *q){
	float cf = cosf(theta);
	float sf = sinf(theta);
//	RunCordic(theta, &cf, &sf);

    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);
    }
