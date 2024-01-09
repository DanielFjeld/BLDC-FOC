/*
 * Flash.h
 *
 *  Created on: Dec 11, 2023
 *      Author: Daniel
 */

#ifndef FLASH_H_
#define FLASH_H_
	#include <stdlib.h>
	#include "main.h"
	#include "Calibration.h"





	typedef struct Flash{
		char ID[30];

		//PID
		float Angle_kp;
		float Angle_ki;
		float Angle_kd;
		float Velocity_kp;
		float Velocity_ki;
		float Velocity_kd;
		float Current_kp;
		float Current_ki;
		float Current_kd;
		float Current_offset_kp;
		float Current_offset_ki;
		float Current_offset_kd;

		//limits
		float Current_limit;
		float Velocity_limit;

		//encoder position
		float Encoder1_offset;
		float Encoder2_offset;

		//calibration
		uint8_t PHASE_ORDER;
		float electrical_offset;
		float error_filt[SIZE*NPP];
		uint16_t mech_offset;





		//padding for storage
		uint64_t PADDING_ZERO;
	}Flash;

	void Flash_init(uint8_t RAM_CMP);

	void Flash_save();

	Flash * Flash_get_values();

#endif /* FLASH_H_ */
