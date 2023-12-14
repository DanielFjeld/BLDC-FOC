/*
 * Calibration.h
 *
 *  Created on: Dec 10, 2023
 *      Author: Daniel
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

//	#include "foc.h"
//	#include "PositionSensor.h"
//	#include "PreferenceWriter.h"
//	#include "user_config.h"
#include "CTRL.h"
#include "Encoders_SPI.h"
#include "current_ADC.h"

	#define V_CAL 0.15f;
	#define LUT_SIZE 360
	#define SIZE 16
	#define NPP 17 //number of pole pairs
	#define CPR 16384//counts per revolution


	extern uint8_t PHASE_ORDER;
	extern float electrical_offset;
	extern float error_filt[SIZE*NPP];
//	extern uint32_t motor_lut[LUT_SIZE];

	void order_phases(Encoders *ps, Current *cs);
	void calibrate(Encoders *ps, Current *cs);
	//void order_phases(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs);
	//void calibrate(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs);

#endif /* CALIBRATION_H_ */
