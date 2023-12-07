/*
 * BLDC_FOC.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Daniel
 */
/*TODO:
 * OK---->Make driver for encoders (velocity and position)
 * encoder calibration (finds magnetic 0 DEG and create offset for encoder)(can be done manually)
 * OK---->Read voltage and temperature (in Current.c)
 * OK---->use buffer after callback to store all values that get changed in IRQ
 * create IIR or FIR filter with built in STM hardware
 * stop if no data or move to another position?
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
 *
 * Test Heat generation @ 50A
 * Test slack and repeatability (while driving motor)
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
 * OK---->reset faults
 * OK---->RUN
 * OK---->STOP
 *
 * ---------CAN test rig------
 * write to status register (buttons, Potentiometer and UART)
 * 		Potentiometer for setpoint
 * 		button to enable run (stop when released)
 * 		button to start calibration (single message) only need to run one time (can store it in memory) encoder 1 and 2
 *
 *print feedback on UART (use plotting program)(Arduino?) 200hz
 *
 *Zero gravity test
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

//#define Current_debug
//#define Voltage_debug
//#define Temperature_debug
#define Status_debug
#define Position_debug

//#define DAC_DEBUG

//#define ZERO_GRAVITY
float weight = 100;
float fast_sin_2(float deg);

//-------------------MISC-----------------
uint8_t Current_Callback_flag = 0;
uint32_t timing_CAN_feedback = 0;
uint32_t running_LED_timing = 0;
uint32_t error = 0;					//error bits


int16_t mech_to_el_deg(int32_t angle_deg, int32_t offset_deg);

//----------------------IRQ--------------------
Current IRQ_Current = {0};
Voltage_Temp IRQ_Voltage_Temp = {0};
Encoders IRQ_Encoders = {0};

//----------------------FIR-------------------
#define FIR_FILTER_LENGTH 10
static float FIR_INPULSE_RESPONSE[FIR_FILTER_LENGTH] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
uint8_t FIR_index = 0;
float FIR_Values[FIR_FILTER_LENGTH] = {0};
float Update_FIR_filter(float input){
	FIR_Values[FIR_index] = input;
	float temp;
	for(int i = 0; i < FIR_FILTER_LENGTH; i++){
		temp += FIR_INPULSE_RESPONSE[i]*FIR_Values[i];
	}
	if(FIR_index < FIR_FILTER_LENGTH-1)FIR_index++;
	else FIR_index = 0;
	return temp;
}

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
	.min_warning = NAN,
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
CAN_LIMITS LIMIT_Current = {
	.max_error = 60000,
	.min_error = NAN,
	.max_warning = 30000,
	.min_warning = NAN,
	.max = 2,
	.min = 0
};
CAN_LIMITS LIMIT_Velocity = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = 3000,
	.min_warning = -3000,
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
	HAL_Delay(1000);
	//----------------PID---------
	SetSampleTime(&Current_PID, 40); //40us = 25kHz
	SetTunings(&Current_PID, 0.005f, 0.001f, 0.0f, 1); //alva
//	SetTunings(&Current_PID, 0.005f, 40.0f, 0.0f, 1); //gimbal
	SetOutputLimits(&Current_PID, 0, 1499);
	SetControllerDirection(&Current_PID, DIRECT);
	SetMode(&Current_PID,  AUTOMATIC);
	Initialize(&Current_PID);

	SetSampleTime(&Velocity_PID, 100); //100s = 10kHz
	SetTunings(&Velocity_PID, 0.00001f, 0.1f, 0.0f, 1);
	SetOutputLimits(&Velocity_PID, 0, 500);
	SetControllerDirection(&Velocity_PID, DIRECT);
	SetMode(&Velocity_PID,  AUTOMATIC);
	Initialize(&Velocity_PID);


	SetSampleTime(&Angle_PID, 100); //100s = 10kHz
	SetTunings(&Angle_PID, 10.0f, 0.0f, 0.0f, 1);
	SetOutputLimits(&Angle_PID, -4000000, 4000000);
	SetControllerDirection(&Angle_PID, DIRECT);
	SetMode(&Angle_PID,  AUTOMATIC);
	Initialize(&Angle_PID);

	//setup encoder
	ORBIS_init((void*)&Encoders_IRQ);

	//setup current
	current_init((void*)&Current_IRQ);

	//calibrate DC current offset
	HAL_Delay(100); //let thing settle before starting

	uint16_t current_offset_averaging = 100;
	volatile int32_t current_offset = 0;
	while (current_offset_averaging){
		while(!Current_Callback_flag);
		Current_Callback_flag = 0;
		current_offset += IRQ_Current.Current_DC;
		current_offset_averaging--;
	}
	current_offset = current_offset/100;

	//setup voltage and temperature readings
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


	BLDC_STATUS_Feedback Status = BLDC_STOPPED_WITH_BREAK;

	float velocity_temp[200] = {0};
	uint32_t last_pos = 0;
	uint8_t velocity_index = 0;
	float velocity = 0;

	int32_t position_overflow = 0;

	int32_t pos_set_test = 360000*10;

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
		IRQ_Current_BUFF.Current_DC -= current_offset;

		//start calibration
		if(Status == BLDC_STOPPED_WITH_BREAK && IRQ_STATUS_BUFF.status == INPUT_CALIBRATE_ENCODER)Status = BLDC_CALIBRATING_ENCODER;

		//reset errors
		else if(Status == BLDC_STOPPED_WITH_BREAK && IRQ_STATUS_BUFF.status == INPUT_RESET_ERRORS)error = 0;

		//start motor when not running
		else if(Status == BLDC_STOPPED_WITH_BREAK && IRQ_STATUS_BUFF.status == INPUT_START){
			Status = BLDC_RUNNING;
			SetMode(&Current_PID,  AUTOMATIC);
			SetMode(&Velocity_PID,  AUTOMATIC);
			SetMode(&Angle_PID,  AUTOMATIC);
		}

		//stop motor when running
		else if(Status == BLDC_RUNNING && IRQ_STATUS_BUFF.status == INPUT_STOP_WITH_BREAK){
			Status = BLDC_STOPPED_WITH_BREAK;
			SetMode(&Current_PID,  MANUAL);
			SetMode(&Velocity_PID,  MANUAL);
			SetMode(&Angle_PID,  MANUAL);
		}
		else if(Status == BLDC_RUNNING && IRQ_STATUS_BUFF.status == INPUT_STOP_AND_SHUTDOWN)Status = BLDC_STOPPED_AND_SHUTDOWN;

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

		if (Status == BLDC_RUNNING && Angle_PID.Input > IRQ_STATUS_BUFF.setpoint - 90000 && Angle_PID.Input < IRQ_STATUS_BUFF.setpoint + 90000) warning &= ~(1 << 8); //warning
		else if (Status == BLDC_RUNNING)warning |= 1 << 8; //warning

		//-------------------RUN FIR FILTER---------------------
		float test = Update_FIR_filter((float)(IRQ_Current_BUFF.Current_DC));

		//----------------------position-----------------
		if (last_pos > 270000 && IRQ_Encoders_BUFF.Encoder1_pos < 90000)position_overflow++;
		else if (last_pos < 90000 && IRQ_Encoders_BUFF.Encoder1_pos > 270000)position_overflow--;

		//-------------------calculate velocity------------------
		velocity_temp[velocity_index] = (IRQ_Encoders_BUFF.Encoder1_pos - last_pos);
		last_pos = IRQ_Encoders_BUFF.Encoder1_pos;
		velocity_index++;
		velocity_index = velocity_index % 200;
		for(int i = 0; i > 200; i++){
			velocity += velocity_temp[i];
		}
		velocity = velocity;

		//------------------calculate PID----------------------- 6.52us
		#ifdef RUNNING_LED_DEBUG
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif




		Angle_PID.Input = (float)IRQ_Encoders_BUFF.Encoder1_pos + position_overflow*360000;
		Velocity_PID.Input = (float)(abs(IRQ_Encoders_BUFF.Velocity));
		Current_PID.Input = test;

		Angle_PID.Setpoint = pos_set_test;

		Angle_PID.Setpoint = IRQ_STATUS_BUFF.setpoint;
		pos_set_test += 20;
		Compute(&Angle_PID);

		Velocity_PID.Setpoint = (abs(Angle_PID.Output));

		Compute(&Velocity_PID);

		int8_t direction = -1;
		#ifndef ZERO_GRAVITY
//		if(IRQ_Voltage_Temp_BUFF.V_Bat > 10000)SetMode(&Current_PID,  AUTOMATIC);//Limit(&LIMIT_Current, Velocity_PID.Output);
//		else SetMode(&Current_PID,  MANUAL);
		//SetMode(&Angle_PID,  AUTOMATIC);
		Current_PID.Setpoint = 1000;
//		Current_PID.Setpoint = Velocity_PID.Output;

		if(Angle_PID.Output > 0) direction = 1;
		else direction = -1;

		#else
		Current_PID.Setpoint = weight*(fast_sin_2((abs)((float)IRQ_Encoders_BUFF.Encoder1_pos)/1000));
		if(IRQ_Encoders_BUFF.Encoder1_pos > 180000) direction = -1;
		else direction = 1;
		#endif

		Compute(&Current_PID);

		//-----------------set PWM--------------------- 3.12us
		#ifdef RUNNING_LED_DEBUG
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif

		if(error){
			Status = BLDC_ERROR;
			shutoff();
		}
		else if (Status == BLDC_STOPPED_AND_SHUTDOWN){
			shutoff();
			shutdown();
		}
		else if (Status == BLDC_STOPPED_WITH_BREAK){
			shutoff();
			//inverter(mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, offset)+(direction*90), Current_PID.Output);
//			inverter(0, 200);



			//inverter(mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, offset)+(direction*90), (uint16_t)Current_PID.Output);
			//inverter(mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, offset)+(1*90), (uint16_t)Limit(&LIMIT_V_motor, Current_PID.Output));
		}
		else if (Status == BLDC_RUNNING){
			inverter(mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, offset)+(direction*90), (uint16_t)Velocity_PID.Output);
//			inverter(mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, offset)+(direction*90), (uint16_t)Current_PID.Output);
		}
		else if (Status == BLDC_CALIBRATING_ENCODER){
			//inverter(0, (uint16_t)Limit(&LIMIT_V_motor, Velocity_PID.Output));
			Status = BLDC_STOPPED_WITH_BREAK;
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
			Feedback.Status_mode = Status;

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
//			Feedback.Position_Calculated_pos = IRQ_Encoders_BUFF.Calculated_pos;
			Feedback.Position_Calculated_pos = mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, offset);
			Feedback.Position_Velocity = IRQ_Encoders_BUFF.Velocity;
//			Feedback.Position_Velocity = (int32_t)velocity;
			FDCAN_sendData(&hfdcan1, (CAN_FEEDBACK_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (uint8_t*)&Feedback);

			//-----------------PRINTF DEBUGGING-------------------
			//will print same info as on CAN-BUS
			#ifdef PRINT_DEBUG
			PrintServerPrintf(
					#ifdef Current_debug
					"CURRENT[M1:%7d M2:%7d M3:%7d DC:%7d DC(FIR):%7d] "
					#endif
					#ifdef Voltage_debug
					"VOLTAGE[Vbat:%5d Vaux:%5d]  "
					#endif
					#ifdef Temperature_debug
					"TEMPERATURE[NTC1:%5d NTC2:%5d]  "
					#endif
					#ifdef Status_debug
					"STATUS[MODE:%s SP:%8d WARN:0x%02x ERROR:0x%02x]"
					#endif
					#ifdef Position_debug
					"POSITION[EN1:%7d EN2:%7d CALC:%7d VELOCITY:%7i]"
					#endif
					"\r\n"
					#ifdef Current_debug
					, Feedback.Current_M1,  Feedback.Current_M2, Feedback.Current_M3, Feedback.Current_DC, (int32_t)test
					#endif
					#ifdef Voltage_debug
					, Feedback.Voltage_BAT, Feedback.Voltage_AUX
					#endif
					#ifdef Temperature_debug
					, Feedback.Temp_NTC1, Feedback.Temp_NTC2
					#endif
					#ifdef Status_debug
					, status_sting[Feedback.Status_mode], Feedback.Status_setpoint, Feedback.Status_warning, Feedback.Status_faults
						#endif
					#ifdef Position_debug
					, Feedback.Position_Encoder1_pos, Feedback.Position_Encoder2_pos, Feedback.Position_Calculated_pos, Feedback.Position_Velocity
					#endif
					); // \r only goes back not to next line!
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

		//-----------------update dac---------------------------
		#ifdef DAC_DEBUG
//		dac_value(Current_PID.Output);
		dac_value(test);
		#endif
	}
}

int16_t mech_to_el_deg(int32_t angle_deg, int32_t offset_deg){
	//return (int16_t)((int32_t)((int32_t)(((int32_t)((int32_t)angle_deg)-offset_deg)%deg_pr_pole))/(deg_pr_pole/360));
	return (int16_t)((abs(((angle_deg)-offset_deg+360000)%(deg_pr_pole)))/(1000/17))%360;
//	return (int16_t)((((int32_t)((int32_t)19400)-offset_deg)%(deg_pr_pole))/(1000/17));
}

float fast_sin_2(float deg){
	return (4*deg*(180-deg)/(40500 - deg*(180-deg)));
}
