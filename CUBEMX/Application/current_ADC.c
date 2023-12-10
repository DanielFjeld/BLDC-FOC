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

void dac_value(uint16_t V_dac){
	uint16_t dac_value = ((V_dac*ADC_RES)/VDDA);
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
}

// Function to convert polar to rectangular
typedef struct {
    float real;
    float imag;
} Complex;

Complex polar_to_rectangular(float magnitude, float angle_deg) {
    Complex result;
    float angle_rad = angle_deg * (3.14159264 / 180.0); // Convert angle to radians
    result.real = magnitude * cos(angle_rad);
    result.imag = magnitude * sin(angle_rad);
    return result;
}

// Function to add two complex numbers
Complex add_complex(Complex a, Complex b) {
    Complex result;
    result.real = a.real + b.real;
    result.imag = a.imag + b.imag;
    return result;
}

// Function to calculate vector sum
float calculate_vector_sum(float current_A, float current_B, float current_C) {
    // Convert each current to a phasor (complex number)
    Complex phasor_A = polar_to_rectangular(current_A, 0);       // Phase A - 0 degrees
    Complex phasor_B = polar_to_rectangular(current_B, -120);    // Phase B - 120 degrees
    Complex phasor_C = polar_to_rectangular(current_C, 120);     // Phase C - 240 degrees

    // Sum the phasors
    Complex sum = add_complex(add_complex(phasor_A, phasor_B), phasor_C);

    // Calculate the magnitude of the vector sum
    float magnitude = sqrt(sum.real * sum.real + sum.imag * sum.imag);
    return magnitude;
}

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


	HAL_DAC_Init(&hdac1);
	HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
}
void voltage_temperature_init(VT_Callback __IRQ_callback){
	VT_IRQ_callback = __IRQ_callback;

	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)VT_adc_result_DMA, number_of_VT_channels*2);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == &hadc1){
		if(calibrating)ADC_CAL();
		else {
			VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[3]/number_of_oversample);
			data.Current_M1 = -(int32_t)(((((int32_t)adc_result_DMA[2]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[0])*50;
			data.Current_M2 = -(int32_t)(((((int32_t)adc_result_DMA[1]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[1])*50;
			data.Current_M3 = -(int32_t)(((((int32_t)adc_result_DMA[0]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[2])*50;
			data.Current_DC = sqrt(data.Current_M1*data.Current_M1 + data.Current_M2*data.Current_M2 + data.Current_M3*data.Current_M3);//(int32_t)((abs((int)data.Current_M1)+abs((int)data.Current_M2)+abs((int)data.Current_M3))/2);
			Curent_IRQ_callback(&data);
		}
	}
	if (hadc == &hadc2){
		VT_data.Temp_NTC1 = (VT_adc_result_DMA[0]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.Temp_NTC2 = (VT_adc_result_DMA[1]/number_of_VT_oversample*VDDA)/ADC_RES;
		VT_data.V_Bat = (VT_adc_result_DMA[2]/number_of_VT_oversample*VDDA*34)/ADC_RES;
		VT_data.V_aux = (VT_adc_result_DMA[3]/number_of_VT_oversample*VDDA*57)/ADC_RES/10;
		VT_IRQ_callback(&VT_data);
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc == &hadc1 && !calibrating){
		VDDA = (int16_t)3000*(*vrefint)/(adc_result_DMA[7]/number_of_oversample);
		data.Current_M1 = -(int32_t)(((((int32_t)adc_result_DMA[6]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[0])*50;
		data.Current_M2 = -(int32_t)(((((int32_t)adc_result_DMA[5]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[1])*50;
		data.Current_M3 = -(int32_t)(((((int32_t)adc_result_DMA[4]/number_of_oversample*VDDA)/ADC_RES)*153/100)-(int32_t)Voltage_offset[2])*50;
//		data.Current_DC = ((abs((int)data.Current_M1)+abs((int)data.Current_M2)+abs((int)data.Current_M3))/2);
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

//sin(θ◦) ≈ 4θ(180 − θ) 40500 − θ(180 − θ);
//float _sin(float deg){
//	return (4*deg*(180-deg)/(40500 - deg*(180-deg)));
//}

void dq0(float theta, float a, float b, float c, float *d, float *q){
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///

    float cf = cos(theta);
    float sf = sin(theta);

    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);

    }
