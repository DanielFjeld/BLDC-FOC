/*
 * current_ADC.h
 *
 *  Created on: Oct 30, 2023
 *      Author: Daniel
 */

#ifndef CURRENT_ADC_H_
#define CURRENT_ADC_H_
	typedef struct Current{
			int32_t Current_M1;
			int32_t Current_M2;
			int32_t Current_M3;
			int32_t Current_DC;
	}Current;

	typedef struct Voltage_Temp{
		uint32_t V_Bat;
		uint16_t V_aux;
		int16_t Temp_NTC1;
		int16_t Temp_NTC2;
	}Voltage_Temp;

	typedef void (*Current_Callback) (Current* data);
	typedef void (*VT_Callback) (Voltage_Temp* data);

	void current_init(Current_Callback __IRQ_callback);
	void voltage_temperature_init(VT_Callback __IRQ_callback);
	void dac_value(uint16_t V_dac);

	float calculate_vector_sum(float current_A, float current_B, float current_C);

	void dq0(float theta, float a, float b, float c, float *d, float *q);

#endif /* CURRENT_ADC_H_ */
