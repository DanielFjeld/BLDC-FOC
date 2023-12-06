/*
 * BLDC_FOC.h
 *
 *  Created on: Nov 8, 2023
 *      Author: Daniel
 */

#ifndef BLDC_FOC_H_
#define BLDC_FOC_H_
	#include "math.h"

	//CAN ID = (BLDC ID)(DEVICE ID)(TYPE ID) = 11bits
	//same on all BLDC devices as unique ID
	#define CAN_BLDC_ID 0x6 //4bit value

	//needs to be different on every BLDC module on the BUS
	#define CAN_DEVICE_ID 0x3 //4bit value 0x0 to 0xF

	//TYPE ID on CAN messages received from ECU
	#define CAN_STATUS_ID 0x0

	//TYPE ID on CAN messages transmitted form the device (3 bits)
	#define CAN_FEEDBACK_ID 0x4      	//transmitted every ms
	#define CAN_PID_CURRENT_ID 0x5   	//need to be requested (
	#define CAN_PID_VELOCITY_ID 0x6		//need to be requested
	#define CAN_PID_ANGLE_ID 0x7		//need to be requested
	#define CAN_LIMITS_ID 0x8		//need to be requested

	//------MOTOR PARAM-----------------
	#define polepairs 17
	#define offset 313800
	#define deg_pr_pole 360000/polepairs
	//-----------------------------------------------------

	typedef struct CAN_Feedback{ //64Byte data
		uint32_t Status_mode;
		uint32_t Status_warning;
		uint32_t Status_faults;
		int32_t  Status_setpoint;

		uint32_t Position_Encoder1_pos; 	//0 to 360 000
		uint32_t Position_Encoder2_pos; 	//0 to 360 000
		int32_t  Position_Calculated_pos; //-2,147,483,648 to 2,147,483,647 	//DEG/1000
		int32_t  Position_Velocity; 		//-2,147,483,648 to 2,147,483,647 	//RPM/100

		int32_t Current_M1;
		int32_t Current_M2;
		int32_t Current_M3;
		uint32_t Current_DC;

		int16_t Temp_NTC1;
		int16_t Temp_NTC2;
		int16_t Temp_ENCODER1;
		int16_t Temp_ENCODER2;

		uint32_t Voltage_BAT;
		uint32_t Voltage_AUX;
	}CAN_Feedback;
	typedef struct CAN_Status{
		uint32_t status;
		uint32_t reset_faults;
		int32_t setpoint;
	}CAN_Status;
	typedef struct CAN_PID{
		float Kp;
		float Ki;
		float Kd;
	}CAN_PID;
	typedef struct CAN_LIMITS{ //if variable = NAN == Inactive
		float min;
		float min_warning;
		float min_error;
		float max;
		float max_warning;
		float max_error;
	}CAN_LIMITS;


	//--------------------LIMITS---------------------------------
	typedef enum{
		LIMIT_OK,
		LIMIT_WARNING,
		LIMIT_ERROR
	}LIMITS_t;

	typedef enum{
		INPUT_STOP_WITH_BREAK,
		INPUT_STOP_AND_SHUTDOWN,
		INPUT_START,
		INPUT_RESTART,
		INPUT_CALIBRATE_ENCODER,
		INPUT_RESET_ERRORS

	}BLDC_STATUS_INPUT_t;
	typedef enum{
		BLDC_STOPPED_WITH_BREAK,
		BLDC_STOPPED_AND_SHUTDOWN,
		BLDC_CALIBRATING_ENCODER,
		BLDC_RUNNING
	}BLDC_STATUS_Feedback;

	static char status_sting[4][26] = {
			"  BLDC STOPPED WITH BREAK\0",
			"BLDC STOPPED AND SHUTDOWN\0",
			" BLDC CALIBRATING ENCODER\0",
			"             BLDC RUNNING\0"
	};

	void BLDC_main(void);

#endif /* BLDC_FOC_H_ */
