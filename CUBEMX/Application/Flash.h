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

	typedef struct Flash{
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
		uint32_t Current_limit;
		uint32_t Velocity_limit;
	}Flash;

	void Flash_init();

	void Flash_save(Flash *data);

	Flash * Flash_get_values();

#endif /* FLASH_H_ */
