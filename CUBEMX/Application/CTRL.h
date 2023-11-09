/*
 * CTRL.h
 *
 *  Created on: Nov 2, 2023
 *      Author: Daniel
 */
#include "tim.h"

#ifndef CTRL_H_
#define CTRL_H_

	#define update_flag 1

	void CTRL_init(void);
	void CTRL_init_PWM(void);
	void inverter(uint16_t angle, uint16_t voltage);
	void shutoff(void);
	void shutdown(void);
	void tim1_PWM_PulseFinishedCallback(void);
#endif /* CTRL_H_ */
