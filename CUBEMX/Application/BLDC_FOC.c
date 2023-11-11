/*
 * BLDC_FOC.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Daniel
 */
/*TODO:
 * Make driver for encoders (velocity and position)
 * encoder calibration (finds magnetic 0 DEG and create offset for encoder)(can be done manually)
 *
 * OK---->Read voltage and temperature (in Current.c)
 *
 * OK---->use buffer after callback to store all values that get changed in IRQ
 *
 * create IIR or FIR filter with built in STM hardware
 *
 * ---------TEST-------------
 * Test CAN messages
 * Test Current sensors
 * Test position encoder 1
 * Test position encoder 2
 * Test total Position
 * Test Velocity
 * Test voltage
 * Test temperature
 * Test Heat generation @ 50A
 * Test slack and repeatability
 * Test Torque
 *
 *
 * ----------IMPROVEMENTS----------
 * make PID and Limits use int32_t or make float faster
 * try to make loop 50kHz
 * Make CAN use FD
 * Make possible to have different length can message
 * Tune PID from CAN
 * set Limits from CAN
 * and store Limits and PID in flash memory (so it can be edited) also store calibration of encoders (offset)
 * add music to motor again
 *
 *
 * ---------TUNING-------------
 * Tune current PID
 * Tune velocity PID
 * Tune angle PID
 *
 * ---------CAN STATUS--------
 * encoder calibration
 * reset faults
 * RUN
 * STOP
 *
 * ---------CAN test rig------
 * write to status register (buttons, Potentiometer and UART)
 * 		Potentiometer for setpoint
 * 		button to enable run (stop when released)
 * 		button to start calibration (single message) only need to run one time (can store it in memory) encoder 1 and 2
 *
 * print feedback on UART (use plotting program)(Arduino?) 200hz
 */


#include "main.h"
#include "math.h"
#include <stdlib.h>
#include "tim.h"
#include "dma.h"
#include "dac.h"

#include "BLDC_FOC.h"
#include "CTRL.h"
#include "PID.h"
#include "CURRENT_adc.h"
#include "fdcandriver.h"
#include "Encoders_SPI.h"
#include "print_server.h"

#define LOOP_FREQ_KHZ 30

//#define RUNNING_LED_DEBUG
//#define RUNNING_LED_DEBUG2
#define PRINT_DEBUG

#define Current_debug
#define Voltage_debug
//#define Temperature_debug
#define Status_debug
//#define Position_debug

//-------------------MISC-----------------
uint8_t Current_Callback_flag = 0;
uint32_t timing_CAN_feedback = 0;
uint32_t running_LED_timing = 0;
uint32_t error = 0;					//error bits

//----------------------IRQ--------------------
Current IRQ_Current = {0};
Voltage_Temp IRQ_Voltage_Temp = {0};
Encoders IRQ_Encoders = {0};


//----------------------CAN--------------------
CAN_Status IRQ_Status;
CAN_Feedback Feedback;

//PID
//CAN_PID PID_Current;
//CAN_PID PID_Velocity;
//CAN_PID PID_Angle;

//---------------------PID---------------------
PID_instance Current_PID = {0};
PID_instance Velocity_PID = {0};
PID_instance Angle_PID = {0};

//-------------------IRQ handlers---------------------
void Current_IRQ(Current* ptr){
	#ifdef RUNNING_LED_DEBUG2
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
	#endif
	#ifdef RUNNING_LED_DEBUG
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
	#endif

    if(ptr != NULL)memcpy(&IRQ_Current, ptr, sizeof(Current));
    else return;
    Current_Callback_flag = 1;
}
void Voltage_Temp_IRQ(Voltage_Temp* ptr){
	memcpy(&IRQ_Voltage_Temp, ptr, sizeof(Voltage_Temp));
}
void Encoders_IRQ(Encoders* ptr){
	memcpy(&IRQ_Encoders, ptr, sizeof(Encoders));
}

//-------------------CAN RX------------------------
void Can_RX_Status_IRQ(CAN_Status* ptr){
	memcpy(&IRQ_Status, ptr, sizeof(CAN_Status));

}
//void Can_RX_Limits_IRQ(CAN_LIMITS* ptr){
//	memcpy(&Status, ptr, sizeof(CAN_Status));
//}
//void Can_RX_PID_Current_IRQ(CAN_PID* ptr){
//	//memcpy(&Status, ptr, sizeof(CAN_Status));
//}
//void Can_RX_PID_Velocity_IRQ(CAN_PID* ptr){
//	//memcpy(&Status, ptr, sizeof(CAN_Status));
//}
//void Can_RX_PID_Angle_IRQ(CAN_PID* ptr){
//	//memcpy(&Status, ptr, sizeof(CAN_Status));
//}

//-----------------POSITION LIMITS-----------------------
CAN_LIMITS LIMIT_Encoder_1 = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = NAN,
	.min_warning = NAN,
	.max = NAN,
	.min = NAN
};
CAN_LIMITS LIMIT_Encoder_2 = {
	.max_error = 100,
	.min_error = -10,
	.max_warning = 95,
	.min_warning = -5,
	.max = 90,
	.min = 0
};

//--------------ERROR LIMITS---------------------
CAN_LIMITS LIMIT_V_BAT = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = 60000,
	.min_warning = 9000,
	.max = NAN,
	.min = NAN
};
CAN_LIMITS LIMIT_V_AUX = {
	.max_error = 18000,
	.min_error = 9000,
	.max_warning = 16000,
	.min_warning = 9000,
	.max = NAN,
	.min = NAN
};
CAN_LIMITS LIMIT_temp = {
	.max_error = 40,
	.min_error = -25,
	.max_warning = 30,
	.min_warning = -5,
	.max = NAN,
	.min = NAN
};

//------------PID LIMITS------------------
CAN_LIMITS LIMIT_V_motor = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = NAN,
	.min_warning = NAN,
	.max = 10,
	.min = 0
};
CAN_LIMITS LIMIT_Current = {
	.max_error = 60000,
	.min_error = NAN,
	.max_warning = 3000,
	.min_warning = NAN,
	.max = 2,
	.min = 0
};
CAN_LIMITS LIMIT_Velocity = {
	.max_error = 100,
	.min_error = NAN,
	.max_warning = 10,
	.min_warning = NAN,
	.max = 10,
	.min = 0
};

//check value OK
LIMITS_t check_value(CAN_LIMITS* ptr, float value){
	if(value >= ptr->max_error || value <= ptr->min_error) return LIMIT_ERROR;			//error
	if(value >= ptr->max_warning || value <= ptr->min_warning) return LIMIT_WARNING;	//warning
	return LIMIT_OK;																	//OK
}
float Limit(CAN_LIMITS* ptr, float value){
	if(value > ptr->max)return ptr->max;
	if(value < ptr->min)return ptr->min;
	return value;
}

//------------------------MAIN-------------------------
void BLDC_main(void){
	//----------------PID---------
	SetSampleTime(&Current_PID, 40); //40us = 25kHz
	SetTunings(&Current_PID, 0.005f, 10.0f, 0.0f, 1);
	SetOutputLimits(&Current_PID, 0, 10);
	SetControllerDirection(&Current_PID, DIRECT);
	SetMode(&Current_PID,  AUTOMATIC);
	Initialize(&Current_PID);

	SetSampleTime(&Velocity_PID, 100); //100s = 10kHz
	SetTunings(&Velocity_PID, 1.0f, 0.1f, 0.0f, 1);
	SetOutputLimits(&Velocity_PID, 0, 10);
	SetControllerDirection(&Velocity_PID, DIRECT);
	SetMode(&Velocity_PID,  AUTOMATIC);
	Initialize(&Velocity_PID);


	SetSampleTime(&Angle_PID, 100); //100s = 10kHz
	SetTunings(&Angle_PID, 1.0f, 0.0f, 0.0f, 1);
	SetOutputLimits(&Angle_PID, 0, 10);
	SetControllerDirection(&Angle_PID, DIRECT);
	SetMode(&Angle_PID,  AUTOMATIC);
	Initialize(&Angle_PID);

	//setup encoder
	//encoder_init((void*)&Encoders_IRQ);

	//setup current
	current_init((void*)&Current_IRQ);
	voltage_temperature_init((void*)&Voltage_Temp_IRQ);

	//setup temperature and voltage
	//temp_volt_init((void*)&Voltage_Temp_IRQ);

	//setup CAN
	//-----------------CAN----------------------
	FDCAN_addCallback(&hfdcan1, (CAN_STATUS_ID << 8) 		| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_Status_IRQ);

//	FDCAN_addCallback(&hfdcan1, (CAN_LIMITS_ID << 8) 		| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_Limits_IRQ);
//	FDCAN_addCallback(&hfdcan1, (CAN_PID_CURRENT_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_PID_Current_IRQ);
//	FDCAN_addCallback(&hfdcan1, (CAN_PID_VELOCITY_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_PID_Velocity_IRQ);
//	FDCAN_addCallback(&hfdcan1, (CAN_PID_ANGLE_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_PID_Angle_IRQ);

	FDCAN_Start(&hfdcan1);

	//--------------setup PWM------------------
	CTRL_init_PWM();

	HAL_Delay(100); //let thing settle before starting

	Current IRQ_Current_BUFF = {0};
	Voltage_Temp IRQ_Voltage_Temp_BUFF = {0};
	Encoders IRQ_Encoders_BUFF = {0};
	CAN_Status  IRQ_STATUS_BUFF = {0};
	while(1){
		#ifdef RUNNING_LED_DEBUG2
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif
		//check if flag has been set indicating new current measurements
		while(!Current_Callback_flag);



		Current_Callback_flag = 0;

		memcpy(&IRQ_Current_BUFF, &IRQ_Current, sizeof(Current));
		memcpy(&IRQ_Voltage_Temp_BUFF, &IRQ_Voltage_Temp, sizeof(Voltage_Temp));
		memcpy(&IRQ_Encoders_BUFF, &IRQ_Encoders, sizeof(Encoders));
		memcpy(&IRQ_STATUS_BUFF, &IRQ_Status, sizeof(CAN_Status));

		//time keepers
		timing_CAN_feedback++;
		running_LED_timing++;

		//reset warnings
		uint32_t warning = 0;



		//-------------------check warning and error--------------------------- 6us
		#ifdef RUNNING_LED_DEBUG
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif

		LIMITS_t Limit_callback;

		Limit_callback = check_value(&LIMIT_Current, (float)IRQ_Current_BUFF.Current_DC);
		warning |= (Limit_callback&1)      << 0; //warning
		error   |= ((Limit_callback&2)>>1) << 0; //error

		Limit_callback = check_value(&LIMIT_Encoder_1, (float)IRQ_Encoders_BUFF.Encoder1_pos);
		warning |= (Limit_callback&1)      << 1; //warning
		error   |= ((Limit_callback&2)>>1) << 1; //error

		Limit_callback= check_value(&LIMIT_Encoder_2, (float)IRQ_Encoders_BUFF.Encoder2_pos);
		warning |= (Limit_callback&1)      << 2; //warning
		error   |= ((Limit_callback&2)>>1) << 2; //error

		Limit_callback = check_value(&LIMIT_Velocity, (float)IRQ_Encoders_BUFF.Velocity);
		warning |= (Limit_callback&1)      << 3; //warning
		error   |= ((Limit_callback&2)>>1) << 3; //error

		Limit_callback = check_value(&LIMIT_V_AUX, (float)IRQ_Voltage_Temp_BUFF.V_aux);
		warning |= (Limit_callback&1)      << 4; //warning
		error   |= ((Limit_callback&2)>>1) << 4; //error

		Limit_callback = check_value(&LIMIT_V_BAT, (float)IRQ_Voltage_Temp_BUFF.V_Bat);
		warning |= (Limit_callback&1)      << 5; //warning
		error   |= ((Limit_callback&2)>>1) << 5; //error

		Limit_callback = check_value(&LIMIT_temp, (float)IRQ_Voltage_Temp_BUFF.Temp_NTC1);
		warning |= (Limit_callback&1)      << 6; //warning
		error   |= ((Limit_callback&2)>>1) << 6; //error

		Limit_callback = check_value(&LIMIT_temp, (float)IRQ_Voltage_Temp_BUFF.Temp_NTC2);
		warning |= (Limit_callback&1)      << 7; //warning
		error   |= ((Limit_callback&2)>>1) << 7; //error


		//------------------calculate PID----------------------- 6.52us
		#ifdef RUNNING_LED_DEBUG
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif

		Angle_PID.Input = (float)IRQ_Encoders_BUFF.Calculated_pos;
		Velocity_PID.Input = (float)IRQ_Encoders_BUFF.Velocity;
		Current_PID.Input = (float)IRQ_Current_BUFF.Current_DC;

		Angle_PID.Setpoint = Limit(&LIMIT_Encoder_2, IRQ_STATUS_BUFF.setpoint);
		Compute(&Angle_PID);

		Velocity_PID.Setpoint =  Limit(&LIMIT_Velocity, Angle_PID.Output);
		Compute(&Velocity_PID);

		Current_PID.Setpoint = Limit(&LIMIT_Current, Velocity_PID.Output);
		Compute(&Current_PID);

		//-----------------set PWM--------------------- 3.12us
		#ifdef RUNNING_LED_DEBUG
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif

		if(error){
			shutoff();
			shutdown();
		}
		else{
			inverter((IRQ_Encoders_BUFF.Encoder1_pos+90000)/1000, (uint16_t)Limit(&LIMIT_V_motor, Velocity_PID.Output));
		}


		//--------------send can message------------------ 1us
		#ifdef RUNNING_LED_DEBUG
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif

		if(timing_CAN_feedback >= LOOP_FREQ_KHZ*5){ //every 5ms
			timing_CAN_feedback = 0;
			Feedback.Status_warning = warning;
			Feedback.Status_faults = error;
			Feedback.Status_setpoint = IRQ_STATUS_BUFF.setpoint;
			Feedback.Status_mode = IRQ_STATUS_BUFF.status;

			Feedback.Current_DC = IRQ_Current_BUFF.Current_DC;
			Feedback.Current_M1 = IRQ_Current_BUFF.Current_M1;
			Feedback.Current_M2 = IRQ_Current_BUFF.Current_M2;
			Feedback.Current_M3 = IRQ_Current_BUFF.Current_M3;

			Feedback.Voltage_AUX = IRQ_Voltage_Temp_BUFF.V_aux;
			Feedback.Voltage_BAT = IRQ_Voltage_Temp_BUFF.V_Bat;
			Feedback.Temp_NTC1 = IRQ_Voltage_Temp_BUFF.Temp_NTC1;
			Feedback.Temp_NTC2 = IRQ_Voltage_Temp_BUFF.Temp_NTC2;

			Feedback.Position_Encoder1_pos = IRQ_Encoders_BUFF.Encoder1_pos;
			Feedback.Position_Encoder2_pos = IRQ_Encoders_BUFF.Encoder2_pos;
			Feedback.Position_Calculated_pos = IRQ_Encoders_BUFF.Calculated_pos;
			Feedback.Position_Velocity = IRQ_Encoders_BUFF.Velocity;
			FDCAN_sendData(&hfdcan1, (CAN_FEEDBACK_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (uint8_t*)&Feedback);

			//-----------------PRINTF DEBUGGING-------------------
			//will print same info as on CAN-BUS
			#ifdef PRINT_DEBUG
			PrintServerPrintf( "UART DEBUG:"
					#ifdef Current_debug
					"CURRENT[M1:%7d M2:%7d M3:%7d DC:%7d] "
					#endif
					#ifdef Voltage_debug
					"VOLTAGE[Vbat:%5d Vaux:%5d]  "
					#endif
					#ifdef Temperature_debug
					"TEMPERATURE[NTC1:%5d NTC2:%5d]  "
					#endif
					#ifdef Status_debug
					"STATUS[MODE:%2d SP:%2d WARN:0x%02x ERROR:0x%02x]"
					#endif
					#ifdef Position_debug
					"POSITION[EN1:%7d EN2:%7d CALC:%7d VELOCITY:%7d]"
					#endif
					"\r\n"
					#ifdef Current_debug
					, Feedback.Current_M1,  Feedback.Current_M2, Feedback.Current_M3, Feedback.Current_DC
					#endif
					#ifdef Voltage_debug
					, Feedback.Voltage_BAT, Feedback.Voltage_AUX
					#endif
					#ifdef Temperature_debug
					, Feedback.Temp_NTC1, Feedback.Temp_NTC2
					#endif
					#ifdef Status_debug
					, Feedback.Status_mode, Feedback.Status_setpoint, Feedback.Status_warning, Feedback.Status_faults
						#endif
					#ifdef Position_debug
					, Feedback.Position_Encoder1_pos, Feedback.Position_Encoder2_pos, Feedback.Position_Calculated_pos, Feedback.Position_Velocity
					#endif
					); //\r only goes back not to next line!
			#endif
		}

		#ifdef RUNNING_LED_DEBUG
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif

		//----------------set status LEDs---------------------
		if(error)HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
		else HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
		if(warning)HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 1);
		else HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 0);

		#ifndef RUNNING_LED_DEBUG
		#ifndef RUNNING_LED_DEBUG2
		if(running_LED_timing >= LOOP_FREQ_KHZ*100){
			running_LED_timing = 0;
			HAL_GPIO_TogglePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin);
		}
		#endif
		#endif



	}
}

