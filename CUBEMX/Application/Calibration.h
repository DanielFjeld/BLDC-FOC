/*
 * Calibration.h
 *
 *  Created on: Dec 10, 2023
 *      Author: Daniel
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

	#include "foc.h"
	#include "PositionSensor.h"
	#include "PreferenceWriter.h"
	#include "user_config.h"

	#define V_CAL 0.15f;


	void order_phases(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs);
	void calibrate(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs);

#endif /* CALIBRATION_H_ */
