/*
 * BLDC_FOC.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Daniel
 */
/*TODO:
 * Voltage input on inverter
 * Temp motor NTC
 * LPF Fmac
 * Zero voltage pause calibration
 *
 * Encoder/can timeout
 * encoder CRC
 *
 *
 * create IIR or FIR filter with built in STM hardware
 * stop if no data or move to another position?
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
#include "Flash.h"

#include "CORDIC_math.h"

#include "Calibration.h"

#define LOOP_FREQ_KHZ 20

//#define RUNNING_LED_DEBUG
#define RUNNING_LED_DEBUG2
#define PRINT_DEBUG

//#define Current_debug
//#define Voltage_debug
//#define Temperature_debug
//#define Status_debug
//#define Position_debug
#define print

//#define CALIBRATE_ON_STARTUP

#define DAC_DEBUG

//#define ZERO_GRAVITY

//-------------------MISC-----------------
volatile uint8_t Current_Callback_flag = 0;
uint32_t timing_CAN_feedback = 0;
uint32_t running_LED_timing = 0;
uint32_t error = 0;					//error bits


int16_t mech_to_el_deg(int32_t angle_deg, int32_t offset_deg);
void calculate();

//----------------------IRQ--------------------
Current IRQ_Current = {0};
Voltage_Temp IRQ_Voltage_Temp = {0};
Encoders IRQ_Encoders = {0};

//----------------------FIR-------------------
#define FIR_FILTER_LENGTH 10
uint8_t FIR_index = 0;
float FIR_value = 0;
float FIR_Values[FIR_FILTER_LENGTH] = {0};
float Update_FIR_filter(float input){
	FIR_value -= FIR_Values[FIR_index];
	FIR_Values[FIR_index] = input/FIR_FILTER_LENGTH;
	FIR_value += FIR_Values[FIR_index];
	if(FIR_index < FIR_FILTER_LENGTH-1)FIR_index++;
	else FIR_index = 0;
	return FIR_value;
}

#define FIR_FILTER_LENGTH2 100
uint8_t FIR_index2 = 0;
float FIR_Values2[FIR_FILTER_LENGTH2] = {0};
float FIR2_value = 0;
float Update_FIR_filter2(float input){
	FIR2_value -= FIR_Values2[FIR_index];
	FIR_Values2[FIR_index] = input/FIR_FILTER_LENGTH2;
	FIR2_value += FIR_Values2[FIR_index] ;
	if(FIR_index2 < FIR_FILTER_LENGTH2-1)FIR_index2++;
	else FIR_index2 = 0;
	return FIR2_value;
}

//----------------------CAN--------------------
CAN_Status IRQ_Status;
CAN_Feedback Feedback;

//PID
CAN_PID PID_Current;

//---------------------PID---------------------
PID_instance Current_PID_offset = {0};
PID_instance Current_PID = {0};
PID_instance Velocity_PID = {0};
PID_instance Angle_PID = {0};

//-------------------IRQ handlers---------------------
void Current_IRQ(Current* ptr){
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

//--------------------TIMERS---------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim == &htim2){
		#ifdef RUNNING_LED_DEBUG2
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		#endif

		calculate();
		//time keepers
		timing_CAN_feedback++;
		running_LED_timing++;

		#ifdef RUNNING_LED_DEBUG2
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
		#endif
	}

	if(htim == &htim3)
		ENCODER_TIM_PeriodElapsedCallback();
}
//-----------------POSITION LIMITS-----------------------
typedef struct CAN_LIMITS{ //if variable = NAN == Inactive
	float min_warning;
	float min_error;
	float max_warning;
	float max_error;
}CAN_LIMITS;

CAN_LIMITS LIMIT_Encoder_1 = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = NAN,
	.min_warning = NAN,
};
CAN_LIMITS LIMIT_Encoder_2 = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = 95,
	.min_warning = -5,
};
//--------------ERROR LIMITS---------------------
CAN_LIMITS LIMIT_V_BAT = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = NAN,
	.min_warning = NAN,
};
CAN_LIMITS LIMIT_V_AUX = {
	.max_error = NAN, //18000,
	.min_error = NAN,
	.max_warning = 16000,
	.min_warning = 9000,
};
CAN_LIMITS LIMIT_temp = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = 30,
	.min_warning = -5,
};

//------------PID LIMITS------------------
CAN_LIMITS LIMIT_Current = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = NAN,
	.min_warning = NAN,
};
CAN_LIMITS LIMIT_Velocity = {
	.max_error = NAN,
	.min_error = NAN,
	.max_warning = NAN,
	.min_warning = NAN,
};

//check value OK
void check_value(CAN_LIMITS* ptr, int32_t value, uint32_t *warning_ptr, uint32_t *error_ptr, uint8_t bit_pos){
	if(value >= ptr->max_error   || value <= ptr->min_error)   *error_ptr   |= (1 << bit_pos);//error
	if(value >= ptr->max_warning || value <= ptr->min_warning) *warning_ptr |= (1 << bit_pos);//warning
}

//------------------------MAIN-------------------------
Flash *storage;

uint16_t mech_offset;
Current IRQ_Current_BUFF = {0};
Voltage_Temp IRQ_Voltage_Temp_BUFF = {0};
Encoders IRQ_Encoders_BUFF = {0};
CAN_Status  IRQ_STATUS_BUFF = {0};

BLDC_STATUS_Feedback Status = BLDC_STOPPED_WITH_BREAK;

uint32_t last_pos_encoder = 0;
int32_t position_overflow = 0;
uint32_t warning = 0;

volatile float theta;
volatile float mag;


void BLDC_main(void){
	Flash_init();
	storage = Flash_get_values();

	HAL_Delay(100);
	//----------------PID---------
	SetSampleTime(&Current_PID, 50); //50us = 20kHz
	SetTunings(&Current_PID, storage->Current_kp, storage->Current_ki, storage->Current_kd, 1);
	SetOutputLimits(&Current_PID, -1500, 1500);
	SetControllerDirection(&Current_PID, DIRECT);
	SetMode(&Current_PID,  AUTOMATIC);
	Initialize(&Current_PID);

	SetSampleTime(&Velocity_PID, 50); //50us = 20kHz
	SetTunings(&Velocity_PID, storage->Velocity_kp, storage->Velocity_ki, storage->Velocity_kd, 1);
	SetOutputLimits(&Velocity_PID, -(storage->Current_limit*1000.0f), (storage->Current_limit*1000.0f));
	SetControllerDirection(&Velocity_PID, DIRECT);
	SetMode(&Velocity_PID,  AUTOMATIC);
	Initialize(&Velocity_PID);

	SetSampleTime(&Angle_PID, 50); //50us = 20kHz
	SetTunings(&Angle_PID, storage->Angle_kp, storage->Angle_ki, storage->Angle_kd, 1);
	SetOutputLimits(&Angle_PID, -(storage->Velocity_limit*1000.0f), (storage->Velocity_limit*1000.0f));
	SetControllerDirection(&Angle_PID, DIRECT);
	SetMode(&Angle_PID,  AUTOMATIC);
	Initialize(&Angle_PID);

	SetSampleTime(&Current_PID_offset, 50); //50us = 20kHz
	SetTunings(&Current_PID_offset, storage->Current_offset_kp, storage->Current_offset_ki, storage->Current_offset_kd, 1);
	SetOutputLimits(&Current_PID_offset, -1500, 1500);
	SetControllerDirection(&Current_PID_offset, DIRECT);
	SetMode(&Current_PID_offset,  AUTOMATIC);
	Initialize(&Current_PID_offset);

	//setup encoder
	ORBIS_init((void*)&Encoders_IRQ);

	//setup current
	current_init((void*)&Current_IRQ);

	//setup voltage and temperature readings
	voltage_temperature_init((void*)&Voltage_Temp_IRQ);
	//setup CAN
	//-----------------CAN----------------------
	FDCAN_addCallback(&hfdcan1, (CAN_STATUS_ID << 8) 		| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_Status_IRQ);
//	FDCAN_addCallback(&hfdcan1, (CAN_PID_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_PID_IRQ);

	FDCAN_Start(&hfdcan1);

	//--------------setup PWM------------------
	electrical_offset = storage->electrical_offset;
	PHASE_ORDER = storage->PHASE_ORDER;
	mech_offset = storage->mech_offset;//storage->mech_offset;
	if(mech_offset > 400)mech_offset = 0;
	uint8_t flash_nan = 0;
	for(int i = 0; i < SIZE*NPP; i++){
		if (isnan(storage->error_filt[i]))flash_nan = 1;
	}
	if(!flash_nan)memcpy(error_filt, storage->error_filt,sizeof(error_filt));

	uint32_t test = 1499;
	CTRL_init_PWM(&test);

	#ifdef CALIBRATE_ON_STARTUP
	Status = BLDC_CALIBRATING_ENCODER;
	#endif

	HAL_TIM_Base_Start_IT(&htim2); //20khz update rate for PID loops
	while(1){
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
			Feedback.Temp_ENCODER1 = IRQ_Encoders_BUFF.Encoder1_temp_x10;
			Feedback.Temp_ENCODER2 = IRQ_Encoders_BUFF.Encoder2_temp_x10;

			Feedback.Position_Encoder1_pos = IRQ_Encoders_BUFF.Encoder1_pos;
			Feedback.Position_Encoder2_pos = IRQ_Encoders_BUFF.Encoder2_pos;
			Feedback.Position_Calculated_pos = Angle_PID.Input;
			Feedback.Position_Velocity = IRQ_Encoders_BUFF.Velocity;
			FDCAN_sendData(&hfdcan1, (CAN_FEEDBACK_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (uint8_t*)&Feedback);

			//-----------------PRINTF DEBUGGING-------------------

			//will print same info as on CAN-BUS
			#ifdef PRINT_DEBUG
			PrintServerPrintf(
					#ifdef Current_debug
					"CURRENT[M1:%7d M2:%7d M3:%7d DC:%7d DC(FIR):%7d D%7d Q%7d POS %4d out %3d] "
					#endif
					#ifdef Voltage_debug
					"VOLTAGE[Vbat:%5d Vaux:%5d]  "
					#endif
					#ifdef Temperature_debug
					"TEMPERATURE[NTC1:%5d NTC2:%5d ENCODER1:%5d ENCODER2:%5d]  "
					#endif
					#ifdef Status_debug
					"STATUS[MODE:%s SP:%8d WARN:0x%02x ERROR:0x%02x]"
					#endif
					#ifdef Position_debug
					"POSITION[EN1:%7d EN2:%7d CALC:%7i VELOCITY:%7i]"
					#endif
					#ifdef print
					"test[c:%f s:%f]"
					#endif
					"\r\n"
					#ifdef Current_debug
					, Feedback.Current_M1,  Feedback.Current_M2, Feedback.Current_M3, Feedback.Current_DC, (int32_t)Current_PID.Output, (int16_t)d_lpf, (int16_t)q_lpf, (int16_t)theta , (int16_t)Current_PID_offset.Output // test
					#endif
					#ifdef Voltage_debug
					, Feedback.Voltage_BAT, Feedback.Voltage_AUX
					#endif
					#ifdef Temperature_debug
					, Feedback.Temp_NTC1, Feedback.Temp_NTC2, Feedback.Temp_ENCODER1, Feedback.Temp_ENCODER2
					#endif
					#ifdef Status_debug
					, status_sting[Feedback.Status_mode], Feedback.Status_setpoint, Feedback.Status_warning, Feedback.Status_faults
						#endif
					#ifdef Position_debug
					, Feedback.Position_Encoder1_pos, (int32_t)Velocity_PID.Input, Feedback.Position_Calculated_pos, Feedback.Position_Velocity/1000
					#endif
					#ifdef print
					, theta, mag
					#endif
			); // \r only goes back not to next line!
			#endif
		}

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
void calculate(){
	//-----------------MEMCPY---------------------- 1.84us
	#ifdef RUNNING_LED_DEBUG
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
	#endif

	memcpy(&IRQ_Current_BUFF, &IRQ_Current, sizeof(Current));
	memcpy(&IRQ_Voltage_Temp_BUFF, &IRQ_Voltage_Temp, sizeof(Voltage_Temp));
	memcpy(&IRQ_Encoders_BUFF, &IRQ_Encoders, sizeof(Encoders));
	memcpy(&IRQ_STATUS_BUFF, &IRQ_Status, sizeof(CAN_Status));

	//FSM
	if(Status == BLDC_STOPPED_WITH_BREAK && IRQ_STATUS_BUFF.status == INPUT_CALIBRATE_ENCODER)Status = BLDC_CALIBRATING_ENCODER;
	else if(Status == BLDC_STOPPED_WITH_BREAK && IRQ_STATUS_BUFF.status == INPUT_RESET_ERRORS)error = 0;
	else if(Status == BLDC_STOPPED_WITH_BREAK && IRQ_STATUS_BUFF.status == INPUT_START){
		Status = BLDC_RUNNING;
		SetMode(&Current_PID,  AUTOMATIC);
		SetMode(&Velocity_PID,  AUTOMATIC);
		SetMode(&Angle_PID,  AUTOMATIC);
	}
	else if(Status == BLDC_RUNNING && IRQ_STATUS_BUFF.status == INPUT_STOP_WITH_BREAK){
		Status = BLDC_STOPPED_WITH_BREAK;
		SetMode(&Current_PID,  MANUAL);
		SetMode(&Velocity_PID,  MANUAL);
		SetMode(&Angle_PID,  MANUAL);
	}
	else if(Status == BLDC_RUNNING && IRQ_STATUS_BUFF.status == INPUT_STOP_AND_SHUTDOWN)Status = BLDC_STOPPED_AND_SHUTDOWN;

	if (last_pos_encoder > 270000 && IRQ_Encoders_BUFF.Encoder1_pos < 90000)position_overflow++;
	else if (last_pos_encoder < 90000 && IRQ_Encoders_BUFF.Encoder1_pos > 270000)position_overflow--;
	last_pos_encoder = IRQ_Encoders_BUFF.Encoder1_pos;

	//------------------calculate PID input----------------------- 19.64us
	#ifdef RUNNING_LED_DEBUG
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
	#endif

	float d;
	float q;
	int16_t index_error = (int16_t)(IRQ_Encoders_BUFF.Encoder1_pos/1000)%360;// - electrical_offset);
	uint16_t index_error2 = ((((index_error-mech_offset+360)%360)*(SIZE*NPP))/360)%(SIZE*NPP);
	int32_t error_pos = ((int32_t)(error_filt[index_error2] - (int32_t)error_filt[0])*17);

	int16_t angle_temp_1 = mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, 0);
	int16_t angle = (angle_temp_1 + error_pos + (int32_t)electrical_offset + 2*360)%360;

	dq0((float)angle*3.14159264f/180, (float)IRQ_Current_BUFF.Current_M2, (float)IRQ_Current_BUFF.Current_M3, (float)IRQ_Current_BUFF.Current_M1, &d, &q);

	float q_lpf = Update_FIR_filter(q);
	float d_lpf = Update_FIR_filter2(d);

	//------------------calculate PID----------------------- 6.88
	#ifdef RUNNING_LED_DEBUG
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
	#endif
	Angle_PID.Input = (float)IRQ_Encoders_BUFF.Encoder1_pos + position_overflow*360000.0f + storage->Encoder1_offset*1000.0f;
	Velocity_PID.Input = (float)(IRQ_Encoders_BUFF.Velocity);
	Current_PID.Input = q_lpf;
	Current_PID_offset.Input = d_lpf;

	Angle_PID.Setpoint = (float)IRQ_STATUS_BUFF.setpoint;
	Compute(&Angle_PID);

	Velocity_PID.Setpoint = Angle_PID.Output;
	Compute(&Velocity_PID);

	Current_PID.Setpoint = Velocity_PID.Output;
	Compute(&Current_PID);

	Current_PID_offset.Setpoint = 0;
	Compute(&Current_PID_offset);

	#ifdef ZERO_GRAVITY
	//		Current_PID.Setpoint = weight*(fast_sin_2((abs)((float)IRQ_Encoders_BUFF.Encoder1_pos)/1000));
	//		if(IRQ_Encoders_BUFF.Encoder1_pos > 180000) direction = -1;
	//		else direction = 1;
	#endif

	//-----------------set PWM--------------------- 8.66us
	#ifdef RUNNING_LED_DEBUG
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
	#endif

	float V_d = Current_PID_offset.Output;
	float V_q = Current_PID.Output;

//	V_d = 0; //0 deg offset
//	V_q = -500; //90 deg offset

	V_d= V_d/1500.0f*0.7f;
	V_q= V_q/1500.0f*0.7f;

	RunCordic_inverse(V_d, V_q, &theta, &mag);

	mag = mag*1500.0f/0.7f;
	theta = theta*180.0f;

	//----------------error check---------------5.92us
	#ifdef RUNNING_LED_DEBUG
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
	#endif

	warning = 0;
	check_value(&LIMIT_Current, (int32_t)q_lpf, &warning, &error, 0);
	check_value(&LIMIT_Current, (int32_t)d_lpf, &warning, &error, 0);
	check_value(&LIMIT_Encoder_1, IRQ_Encoders_BUFF.Encoder1_pos, &warning, &error, 1);
	check_value(&LIMIT_Encoder_2, IRQ_Encoders_BUFF.Encoder2_pos, &warning, &error, 2);
	check_value(&LIMIT_Velocity, IRQ_Encoders_BUFF.Velocity, &warning, &error, 3);
	check_value(&LIMIT_V_AUX, IRQ_Voltage_Temp_BUFF.V_aux, &warning, &error, 4);
	check_value(&LIMIT_V_BAT, IRQ_Voltage_Temp_BUFF.V_Bat, &warning, &error, 5);
	check_value(&LIMIT_temp, IRQ_Voltage_Temp_BUFF.Temp_NTC1, &warning, &error, 6);
	check_value(&LIMIT_temp, IRQ_Voltage_Temp_BUFF.Temp_NTC2, &warning, &error, 7);

	//-----------------set PWM--------------------- 2us
	#ifdef RUNNING_LED_DEBUG
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
	}
	else if (Status == BLDC_RUNNING){
		inverter(mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, 0)+error_pos + (int32_t)electrical_offset + (int32_t)theta + 360*2, mag, PHASE_ORDER);
	}
	else if (Status == BLDC_CALIBRATING_ENCODER){
		HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
		order_phases(&IRQ_Encoders, &IRQ_Current);
		calibrate(&IRQ_Encoders, &IRQ_Current);

		//start calibration
		storage->mech_offset = (int16_t)(IRQ_Encoders.Encoder1_pos/1000)%360;
		storage->electrical_offset = electrical_offset;
		storage->PHASE_ORDER = PHASE_ORDER;
		memcpy(storage->error_filt,error_filt,sizeof(error_filt));
		Flash_save();
		Status = BLDC_STOPPED_WITH_BREAK;
	}
	#ifdef RUNNING_LED_DEBUG
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 1);
	#endif

	//-----------------update dac---------------------------
	#ifdef DAC_DEBUG
	dac_value(q/10 +1500);
	#endif

	#ifdef RUNNING_LED_DEBUG
	HAL_GPIO_WritePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin, 0);
	#endif

}
int16_t mech_to_el_deg(int32_t angle_deg, int32_t offset_deg){
	uint32_t temp = angle_deg-offset_deg+360000*2;
	temp = temp % (360000/17);
	temp = temp*17/1000;
	temp = temp % 360;
	return (int16_t)temp;
}
