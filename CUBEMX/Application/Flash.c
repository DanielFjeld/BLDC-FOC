/*
 * Flash.c
 *
 *  Created on: Dec 11, 2023
 *      Author: Daniel
 */
#include "Flash.h"

Flash *ptr;

Flash RAM = {
		.Angle_kp = 2.0f,
		.Angle_ki = 0.0f,
		.Angle_kd = 0.0f,

		.Velocity_kp = 0.4f,
		.Velocity_ki = 0.01f,
		.Velocity_kd = 0.00001f,

		.Current_kp = 0.02f,
		.Current_ki = 20.0f,
		.Current_kd = 0.0f,

		.Current_offset_kp = 0.001f,
		.Current_offset_ki = 30.1f,
		.Current_offset_kd = 0.0f

};

void Flash_init(){
	ptr = &RAM;
}

void Flash_save(Flash *data){

}

Flash *Flash_get_values(){
	return ptr;
}
