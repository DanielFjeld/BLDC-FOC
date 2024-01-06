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
	#define CAN_PID_ID 0x5

	//------MOTOR PARAM-----------------
	#define polepairs 17
	#define offset 0//174506
	#define deg_pr_pole 360000/polepairs
	//-----------------------------------------------------

	typedef struct CAN_Feedback{ //64Byte data
		uint32_t Status_mode;
		uint32_t Status_warning;
		uint32_t Status_faults;
		float  Status_setpoint;

		float Position_Encoder1_pos; 	//0 to 360 000
		float Position_Encoder2_pos; 	//0 to 360 000
		float Position_Calculated_pos; //-2,147,483,648 to 2,147,483,647 	//DEG/1000
		float Position_Velocity; 		//-2,147,483,648 to 2,147,483,647 	//RPM/1000

		float Current_Q;
		float Current_D;

		int16_t Temp_ENCODER1;
		int16_t Temp_ENCODER2;

		float Voltage_BAT;
		float Voltage_AUX;

		uint8_t recerve[12];
	}CAN_Feedback;
	typedef struct CAN_Status{
		uint32_t status;
		float setpoint;
	}CAN_Status;



	//--------------------LIMITS---------------------------------
	typedef enum{
		INPUT_STOP_WITH_BREAK,
		INPUT_STOP_AND_SHUTDOWN,
		INPUT_START,
		INPUT_RESTART,
		INPUT_CALIBRATE_ENCODER,
		INPUT_RESET_ERRORS

	}BLDC_STATUS_INPUT_t;
	typedef enum{
		PID_CURRENT_D,
		PID_CURRENT_Q,
		PID_VELOCITY,
		PID_ANGLE
	}PID_t;
	typedef enum{
		BLDC_STOPPED_WITH_BREAK,
		BLDC_STOPPED_AND_SHUTDOWN,
		BLDC_CALIBRATING_ENCODER,
		BLDC_RUNNING,
		BLDC_ERROR
	}BLDC_STATUS_Feedback;

	typedef enum{
		LIMIT_Q_CURRENT,
		LIMIT_D_CURRENT,
		LIMIT_ENCODER_1,
		LIMIT_ENCODER_2,
	}LIMIT_CHECK_t;

	static char status_sting[5][26] = {
			"  BLDC STOPPED WITH BREAK\0",
			"BLDC STOPPED AND SHUTDOWN\0",
			" BLDC CALIBRATING ENCODER\0",
			"             BLDC RUNNING\0",
			"               BLDC ERROR\0"
	};

	void BLDC_main(void);
	void run();

#endif /* BLDC_FOC_H_ */
