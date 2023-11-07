/*
 * current_ADC.h
 *
 *  Created on: Oct 30, 2023
 *      Author: Daniel
 */

#ifndef CURRENT_ADC_H_
#define CURRENT_ADC_H_
	void current_init(void);
	void ADC1_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#endif /* CURRENT_ADC_H_ */
