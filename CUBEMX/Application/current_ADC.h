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

	typedef void (*Current_Callback) (Current* data);

	void current_init(Current_Callback __IRQ_callback);
	void ADC1_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#endif /* CURRENT_ADC_H_ */
