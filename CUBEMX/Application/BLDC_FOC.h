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
	#define polepairs 7
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

		//int16_t Temp_ENCODER1;
		//int16_t Temp_ENCODER2;



		float Voltage_magnitude;
		float Voltage_theta;

		float Current_setpoint;
		float Velocity_setpoint;
		float Position_setpoint;

		float reserved;

	}CAN_Feedback;
	typedef struct CAN_Status{
		uint32_t status;
		float setpoint;
		float ramp;
		float reserved;
	}CAN_Status;



	//--------------------LIMITS---------------------------------
	typedef enum{
		//running motor
		INPUT_STOP_WITH_BREAK,
		INPUT_STOP_AND_SHUTDOWN,
		INPUT_START,
		INPUT_RESTART,
		INPUT_CALIBRATE_ENCODER,
		INPUT_RESET_ERRORS,

		//set limits
		SET_LIMIT_Q_CURRENT,
		SET_LIMIT_D_CURRENT,
		SET_LIMIT_ENCODER_1,
		SET_LIMIT_ENCODER_2,
		SET_LIMIT_VOLTAGE,
		SET_LIMIT_CURRENT,
		SET_LIMIT_VELOCITY,
		SET_LIMIT_ANGLE_MAX,
		SET_LIMIT_ANGLE_MIN,

		//set motor
		SET_MOTOR_POLEPAIRS,
		SET_MOTOR_OFFSET,
		SET_MOTOR_DEG_PR_POLE,
		SET_BATTERY_VOLTAGE,


		//set CAN
		SET_CAN_BLDC_ID,
		SET_CAN_FEEDBACK_ID,
		SET_CAN_FAST_ID,

		//set PID on/off
		MODE_PID_CURRENT_D,
		MODE_PID_CURRENT_Q,
		MODE_PID_VELOCITY,
		MODE_PID_ANGLE,

		//set P I D values
		SET_PID_ANGLE_P,
		SET_PID_ANGLE_I,
		SET_PID_ANGLE_D,

		SET_PID_VELOCITY_P,
		SET_PID_VELOCITY_I,
		SET_PID_VELOCITY_D,

		SET_PID_CURRENT_D_P,
		SET_PID_CURRENT_D_I,
		SET_PID_CURRENT_D_D,

		SET_PID_CURRENT_Q_P,
		SET_PID_CURRENT_Q_I,
		SET_PID_CURRENT_Q_D,

		//start calibration
		START_ENCODER_CALIBRATION_ON_START,
		START_MAX_MIN_POSITION_CALIBRATION,

		//reset flash (need to redo calibration before start)
		RESET_FLASH,
		
		//select data on high speed channel 1 and 2
		SELECT_HIGH_SPEED_1_DATA,
		SELECT_HIGH_SPEED_2_DATA,

		//select data to send on DAC
		SELECT_DAC_1_DATA,
		
		//turn on led
		SET_LED,

		SAVE_FLASH,
		SYSTEM_RESET,

		SET_VBAT,
		SET_MAX_VOLTAGE,
		SET_MAX_CURRENT,
		SET_MAX_VELOCITY,
		SET_MIN_POSITION,
		SET_MAX_POSITION,
		SET_MAX_RAMP_RPM,

		FLASH_RESET_TO_RAM,


		ENABLE_CONFIG,
		DISABLE_CONFIG,

		PLAY_MUSIC,

		SET_CAN_ID,



	}BLDC_STATUS_INPUT_t;
	typedef enum{
		PID_CURRENT_D,
		PID_CURRENT_Q,
		PID_VELOCITY,
		PID_ANGLE
	}PID_t;
	typedef enum{
		//running motor
		BLDC_STOPPED_WITH_BREAK,
		BLDC_STOPPED_AND_SHUTDOWN,
		BLDC_CALIBRATING_ENCODER,
		BLDC_RUNNING,
		BLDC_ERROR,
		BLDC_MIN_MAX_POSITION,
		BLDC_CONFIG,
		BLDC_PLAYING_MUSIC,

		//set limits
		BLDC_SET_LIMIT_Q_CURRENT,
		BLDC_SET_LIMIT_D_CURRENT,
		BLDC_SET_LIMIT_ENCODER_1,
		BLDC_SET_LIMIT_ENCODER_2,
		BLDC_SET_LIMIT_VOLTAGE,
		BLDC_SET_LIMIT_CURRENT,
		BLDC_SET_LIMIT_VELOCITY,
		BLDC_SET_LIMIT_ANGLE_MAX,
		BLDC_SET_LIMIT_ANGLE_MIN,

		//set motor
		BLDC_SET_MOTOR_POLEPAIRS,
		BLDC_SET_MOTOR_OFFSET,
		BLDC_SET_MOTOR_DEG_PR_POLE,
		BLDC_SET_BATTERY_VOLTAGE,


		//set CAN
		BLDC_SET_CAN_BLDC_ID,
		BLDC_SET_CAN_FEEDBACK_ID,
		BLDC_SET_CAN_FAST_ID,

		//set PID on/off
		BLDC_SET_PID_CURRENT_D,
		BLDC_SET_PID_CURRENT_Q,
		BLDC_SET_PID_VELOCITY,
		BLDC_SET_PID_ANGLE,

		//set P I D values
		BLDC_SET_PID_ANGLE_P,
		BLDC_SET_PID_ANGLE_I,
		BLDC_SET_PID_ANGLE_D,

		BLDC_SET_PID_VELOCITY_P,
		BLDC_SET_PID_VELOCITY_I,
		BLDC_SET_PID_VELOCITY_D,

		BLDC_SET_PID_CURRENT_D_P,
		BLDC_SET_PID_CURRENT_D_I,
		BLDC_SET_PID_CURRENT_D_D,

		BLDC_SET_PID_CURRENT_Q_P,
		BLDC_SET_PID_CURRENT_Q_I,
		BLDC_SET_PID_CURRENT_Q_D,

		//start calibration
		BLDC_START_ENCODER_CALIBRATION,
		BLDC_START_MAX_MIN_POSITION_CALIBRATION,


		//reset flash (need to redo calibration before start)
		
		//select data on high speed channel 1 and 2
		
		//select data to send on DAC
		
		
		
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
