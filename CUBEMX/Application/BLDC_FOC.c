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
 * OK---->Make driver for encoders (velocity and position)
 * OK---->encoder calibration (finds magnetic 0 DEG and create offset for encoder)(can be done manually)
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
#include "Flash.h"

#include "Calibration.h"

#define LOOP_FREQ_KHZ 30

#define RUNNING_LED_DEBUG
#define PRINT_DEBUG

//#define Current_debug
//#define Voltage_debug
//#define Temperature_debug
//#define Status_debug
#define Position_debug

#define CALIBRATE_ON_STARTUP

#define DAC_DEBUG

int32_t test_val = 0;

//#define ZERO_GRAVITY

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
#define FIR_FILTER_LENGTH 15
static float FIR_INPULSE_RESPONSE[FIR_FILTER_LENGTH] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
uint8_t FIR_index = 0;
float FIR_Values[FIR_FILTER_LENGTH] = {0};
float Update_FIR_filter(float input){
	FIR_Values[FIR_index] = input;
	float temp = 0.0f;
	for(int i = 0; i < FIR_FILTER_LENGTH; i++){
		temp += FIR_INPULSE_RESPONSE[i]*FIR_Values[i];
	}
	if(FIR_index < FIR_FILTER_LENGTH-1)FIR_index++;
	else FIR_index = 0;
	return temp;
}

#define FIR_FILTER_LENGTH2 15
uint8_t FIR_index2 = 0;
float FIR_Values2[FIR_FILTER_LENGTH2] = {0};
float FIR2_value = 0;
float Update_FIR_filter2(float input){
	FIR2_value -= FIR_Values2[FIR_index2];
	FIR_Values2[FIR_index2] = input/FIR_FILTER_LENGTH2;
	FIR2_value += FIR_Values2[FIR_index2] ;
	if(FIR_index2 < FIR_FILTER_LENGTH2-1)FIR_index2++;
	else FIR_index2 = 0;
	return FIR2_value;
}

//----------------------CAN--------------------
CAN_Status IRQ_Status;
CAN_Feedback Feedback;


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
void check_value(CAN_LIMITS* ptr, float value, uint32_t *warning_ptr, uint32_t *error_ptr, uint8_t bit_pos){
	if(value >= ptr->max_error   || value <= ptr->min_error)   *error_ptr   |= (1 << bit_pos);//error
	if(value >= ptr->max_warning || value <= ptr->min_warning) *warning_ptr |= (1 << bit_pos);//warning
}

uint32_t sqrtI(uint32_t sqrtArg)
{
uint32_t answer, x;
uint32_t temp;
if ( sqrtArg == 0 ) return 0; // undefined result
if ( sqrtArg == 1 ) return 1; // identity
answer = 0; // integer square root
for( x=0x8000; x>0; x=x>>1 )
{ // 16 bit shift
answer |= x; // possible bit in root
temp = answer * answer; // fast unsigned multiply
if (temp == sqrtArg) break; // exact, found it
if (temp > sqrtArg) answer ^= x; // too large, reverse bit
}
return answer; // approximate root
}
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
// |error| < 0.005
float atan2_approximation2( float y, float x )
{
	if ( x == 0.0f )
	{
		if ( y > 0.0f ) return PIBY2_FLOAT;
		if ( y == 0.0f ) return 0.0f;
		return -PIBY2_FLOAT;
	}
	float atan;
	float z = y/x;
	if ( fabs( z ) < 1.0f )
	{
		atan = z/(1.0f + 0.28f*z*z);
		if ( x < 0.0f )
		{
			if ( y < 0.0f ) return atan - PI_FLOAT;
			return atan + PI_FLOAT;
		}
	}
	else
	{
		atan = PIBY2_FLOAT - z/(z*z + 0.28f);
		if ( y < 0.0f ) return atan - PI_FLOAT;
	}
	return atan;
}

//------------------------MAIN-------------------------

uint16_t mech_offset;
uint8_t flash_nan;

Current IRQ_Current_BUFF = {0};
Voltage_Temp IRQ_Voltage_Temp_BUFF = {0};
Encoders IRQ_Encoders_BUFF = {0};
CAN_Status  IRQ_STATUS_BUFF = {0};

BLDC_STATUS_Feedback Status = BLDC_STOPPED_WITH_BREAK;

uint32_t last_pos_enc = 0;

int32_t position_overflow = 0;


Flash *storage;

#define PID_TIMING 10
void BLDC_main(void){
	Flash_init(1); // 0 do not update from ram
	storage = Flash_get_values();

	HAL_Delay(100);
	//----------------PID---------
	SetSampleTime(&Current_PID, PID_TIMING); //40us = 25kHz
	SetTunings(&Current_PID, storage->Current_kp, storage->Current_ki, storage->Current_kd, 1);
	SetOutputLimits(&Current_PID, -1500, 1500);
	SetControllerDirection(&Current_PID, DIRECT);
	SetMode(&Current_PID,  AUTOMATIC);
	Initialize(&Current_PID);

	SetSampleTime(&Velocity_PID, PID_TIMING); //100s = 10kHz
	SetTunings(&Velocity_PID, storage->Velocity_kp, storage->Velocity_ki, storage->Velocity_kd, 1);
	SetOutputLimits(&Velocity_PID, (storage->Current_limit*-1.0f), (storage->Current_limit));
	SetControllerDirection(&Velocity_PID, DIRECT);
	SetMode(&Velocity_PID,  AUTOMATIC);
	Initialize(&Velocity_PID);

	SetSampleTime(&Angle_PID, PID_TIMING); //100s = 10kHz
	SetTunings(&Angle_PID, storage->Angle_kp, storage->Angle_ki, storage->Angle_kd, 1);
	SetOutputLimits(&Angle_PID, (storage->Velocity_limit*-1.0f), (storage->Velocity_limit));
	SetControllerDirection(&Angle_PID, DIRECT);
	SetMode(&Angle_PID,  AUTOMATIC);
	Initialize(&Angle_PID);

	SetSampleTime(&Current_PID_offset, PID_TIMING); //100s = 10kHz
	SetTunings(&Current_PID_offset, storage->Current_offset_kp, storage->Current_offset_ki, storage->Current_offset_kd, 1);
	SetOutputLimits(&Current_PID_offset, -1500, 1500);
	SetControllerDirection(&Current_PID_offset, DIRECT);
	SetMode(&Current_PID_offset,  AUTOMATIC);
	Initialize(&Current_PID_offset);


	//setup current
	current_init((void*)&Current_IRQ);
	HAL_Delay(1000);
	//setup encoder
	ORBIS_init((void*)&Encoders_IRQ);

	//setup voltage and temperature readings
	voltage_temperature_init((void*)&Voltage_Temp_IRQ);
	//setup CAN
	//-----------------CAN----------------------
	FDCAN_addCallback(&hfdcan1, CAN_RX_ID, (void*)&Can_RX_Status_IRQ);
//	FDCAN_addCallback(&hfdcan1, (CAN_PID_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_PID_IRQ);

	FDCAN_Start(&hfdcan1);

	//--------------setup PWM------------------
	electrical_offset = storage->electrical_offset;
	PHASE_ORDER = storage->PHASE_ORDER;
	mech_offset = storage->mech_offset;//storage->mech_offset;
	if(mech_offset > 400)mech_offset = 0;
	flash_nan = 0;
	for(int i = 0; i < SIZE*NPP; i++){
		if (isnan(storage->error_filt[i]))flash_nan = 1;
	}
	if(!flash_nan)memcpy(error_filt, storage->error_filt,sizeof(error_filt));


	uint32_t test = 1499;
	CTRL_init_PWM(&test);

	#ifdef CALIBRATE_ON_STARTUP
	Status = BLDC_CALIBRATING_ENCODER;
	#endif


	while(1){
		if (Status == BLDC_CALIBRATING_ENCODER){
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

		#ifdef PRINT_DEBUG
			PrintServerPrintf(
			#ifdef Current_debug
			"CURRENT[M1:%7d M2:%7d M3:%7d DC:%7d DC(FIR):%7d D%7d Q%7d POS %4d out %3d] "
			#endif
			#ifdef Voltage_debug
			"VOLTAGE[Vbat:%.1f Vaux:%.1f]"
			#endif
			#ifdef Temperature_debug
			"TEMPERATURE[NTC1:%5d NTC2:%5d ENCODER1:%5d ENCODER2:%5d]  "
			#endif
			#ifdef Status_debug
			"STATUS[MODE:%s SP:%8d WARN:0x%02x ERROR:0x%02x]"
			#endif
			#ifdef Position_debug
			"POSITION[EN1:%7d EN2:%7d CALC:%7d VELOCITY:%7d]"
			#endif
			"\r\n"
			#ifdef Current_debug
			, IRQ_Current_BUFF.Current_M1,  IRQ_Current_BUFF.Current_M2, IRQ_Current_BUFF.Current_M3, 0, (int32_t)Current_PID.Output, (int16_t)d_lpf, (int16_t)q_lpf, (int16_t)angle , (int16_t)Current_PID_offset.Output // test
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
			, (int32_t)Feedback.Position_Encoder1_pos, (int32_t)Feedback.Position_Encoder2_pos, (int32_t)Feedback.Position_Calculated_pos, (int32_t)Feedback.Position_Velocity//Velocity_PID.Setpoint
			#endif
			); // \r only goes back not to next line!
		#endif
	}
}

void run(){

	#ifdef RUNNING_LED_DEBUG
//	HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
//	HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
	#endif

	memcpy(&IRQ_Current_BUFF, &IRQ_Current, sizeof(Current));
	memcpy(&IRQ_Voltage_Temp_BUFF, &IRQ_Voltage_Temp, sizeof(Voltage_Temp));
	memcpy(&IRQ_Encoders_BUFF, &IRQ_Encoders, sizeof(Encoders));
	memcpy(&IRQ_STATUS_BUFF, &IRQ_Status, sizeof(CAN_Status));

	//----------------------TEST-----------------
	if(Status != BLDC_CALIBRATING_ENCODER){
		IRQ_STATUS_BUFF.setpoint = 50;
		Status == BLDC_RUNNING;
		IRQ_STATUS_BUFF.status = INPUT_CTRL_VOLTAGE_Q_SETPOINT;
	}



	//FSM
	if(Status == BLDC_STOPPED_WITH_BREAK && IRQ_STATUS_BUFF.status == INPUT_CALIBRATE)Status = BLDC_CALIBRATING_ENCODER;
	else if(Status == BLDC_STOPPED_WITH_BREAK && IRQ_STATUS_BUFF.status == INPUT_RESET_ERRORS)error = 0;
	else if(Status == BLDC_STOPPED_WITH_BREAK && (
			IRQ_STATUS_BUFF.status >= INPUT_CTRL_VOLTAGE_Q_SETPOINT &&
			IRQ_STATUS_BUFF.status <= INPUT_CTRL_ZERO_GRAVITY)){
		Status = BLDC_RUNNING;
		SetMode(&Current_PID,  AUTOMATIC);
		SetMode(&Velocity_PID,  AUTOMATIC);
		SetMode(&Angle_PID,  AUTOMATIC);
	}
	else if(Status == BLDC_RUNNING && IRQ_STATUS_BUFF.status == INPUT_CTRL_OFF){
		Status = BLDC_STOPPED_WITH_BREAK;
//		SetMode(&Current_PID,  MANUAL);
//		SetMode(&Velocity_PID,  MANUAL);
//		SetMode(&Angle_PID,  MANUAL);
	}


	//----------------------position-----------------
	if (last_pos_enc > 270000 && IRQ_Encoders_BUFF.Encoder1_pos < 90000)position_overflow++;
	else if (last_pos_enc < 90000 && IRQ_Encoders_BUFF.Encoder1_pos > 270000)position_overflow--;
	last_pos_enc = IRQ_Encoders_BUFF.Encoder1_pos;

	//------------------calculate PID----------------------- 6.52us
	float d;
	float q;
	int16_t index_error = (int16_t)(IRQ_Encoders_BUFF.Encoder1_pos/1000)%360;// - electrical_offset);
	uint16_t index_error2 = ((((index_error-mech_offset+360)%360)*(SIZE*NPP))/360)%(SIZE*NPP);
	int32_t error_pos = (int32_t)(((error_filt[index_error2] - error_filt[0])/**NPP*/)); //((int32_t)(error_filt[index_error2] - (int32_t)error_filt[0])*17);

	//mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, 0) + (int32_t)electrical_offset
	int16_t angle = (mech_to_el_deg(IRQ_Encoders_BUFF.Encoder1_pos, 0)  + error_pos  + (int32_t)electrical_offset + 2*360)%360;
	dq0((float)angle*3.14159264f/180.0f, ((float)IRQ_Current_BUFF.Current_M3/1000.0f), ((float)IRQ_Current_BUFF.Current_M2/1000.0f), ((float)IRQ_Current_BUFF.Current_M1/1000.0f), &d, &q);

	float q_lpf = Update_FIR_filter(q);
	float d_lpf = Update_FIR_filter2(d);

	//------------------calculate PID----------------------- 6.52us
	Angle_PID.Input = ((float)IRQ_Encoders_BUFF.Encoder1_pos)/1000.0f + position_overflow*360.0f + storage->Encoder1_offset;
	Velocity_PID.Input = ((float)(IRQ_Encoders_BUFF.Velocity))/1000.0f;
	Current_PID.Input = q_lpf;
	Current_PID_offset.Input = d_lpf;



	//-------------------------------------------

	if(IRQ_STATUS_BUFF.status == INPUT_CTRL_ANGLE_SETPOINT){
		Angle_PID.Setpoint = IRQ_STATUS_BUFF.setpoint;
		Compute(&Angle_PID);
		Velocity_PID.Setpoint = Angle_PID.Output;
		Compute(&Velocity_PID);
		Current_PID.Setpoint = Velocity_PID.Output;
		Compute(&Current_PID);
		Current_PID_offset.Setpoint = 0;
		Compute(&Current_PID_offset);
	}
	else if(IRQ_STATUS_BUFF.status == INPUT_CTRL_VELOCITY_SETPOINT){
		Velocity_PID.Setpoint = IRQ_STATUS_BUFF.setpoint;
		Compute(&Velocity_PID);
		Current_PID.Setpoint = Velocity_PID.Output;
		Compute(&Current_PID);
		Current_PID_offset.Setpoint = 0;
		Compute(&Current_PID_offset);
	}
	else if(IRQ_STATUS_BUFF.status == INPUT_CTRL_CURRENT_Q_SETPOINT){
		Current_PID.Setpoint = IRQ_STATUS_BUFF.setpoint;
		Compute(&Current_PID);
		Current_PID_offset.Output = 0;
	}
	else if(IRQ_STATUS_BUFF.status == INPUT_CTRL_CURRENT_D_SETPOINT){
		Current_PID.Output = 0.0f;
		Current_PID_offset.Setpoint = IRQ_STATUS_BUFF.setpoint;
		Compute(&Current_PID_offset);
	}
	else if(IRQ_STATUS_BUFF.status == INPUT_CTRL_ZERO_GRAVITY){
		float weight = IRQ_STATUS_BUFF.setpoint; //amps at 90 degrees;
		Current_PID.Setpoint = weight*(sinf((((float)IRQ_Encoders_BUFF.Encoder1_pos)/1000+storage->Encoder1_offset)*3.14159264/180));
	}
	else {
		Current_PID_offset.Output = 0.0f;
		Current_PID.Output = 0.0f;
	}

	//-----------------set PWM--------------------- 3.12us

	float V_d = 0.0f;
	float V_q = 0.0f;
	if(IRQ_STATUS_BUFF.status == INPUT_CTRL_VOLTAGE_Q_SETPOINT){
		V_q = IRQ_STATUS_BUFF.setpoint;
	}
	else if(IRQ_STATUS_BUFF.status == INPUT_CTRL_VOLTAGE_D_SETPOINT){
		V_d = IRQ_STATUS_BUFF.setpoint;
	}
	else{
		V_d = Current_PID_offset.Output;
		V_q = Current_PID.Output;
	}
	float theta = atan2_approximation2(V_q, V_d)*180.0f/3.14159264f;
	uint32_t mag = sqrtI((uint32_t)(V_q*V_q+V_d*V_d));
	mag *= 0.7;
	if (mag > 1499)mag = 1499;

	//----------------error check---------------
	uint32_t warning = 0;
	check_value(&LIMIT_Current, (float)q_lpf, &warning, &error, 0);
	check_value(&LIMIT_Current, (float)d_lpf, &warning, &error, 0);
	check_value(&LIMIT_Encoder_1, (float)IRQ_Encoders_BUFF.Encoder1_pos, &warning, &error, 1);
	check_value(&LIMIT_Encoder_2, (float)IRQ_Encoders_BUFF.Encoder2_pos, &warning, &error, 2);
	check_value(&LIMIT_Velocity, (float)IRQ_Encoders_BUFF.Velocity, &warning, &error, 3);
	check_value(&LIMIT_V_AUX, (float)IRQ_Voltage_Temp_BUFF.V_aux, &warning, &error, 4);
	check_value(&LIMIT_V_BAT, (float)IRQ_Voltage_Temp_BUFF.V_Bat, &warning, &error, 5);
	check_value(&LIMIT_temp, (float)IRQ_Voltage_Temp_BUFF.Temp_NTC1, &warning, &error, 6);
	check_value(&LIMIT_temp, (float)IRQ_Voltage_Temp_BUFF.Temp_NTC2, &warning, &error, 7);

	//-----------------set PWM---------------------
	if(error){
		Status = BLDC_ERROR;
		shutoff();
	}
	else if (Status == BLDC_STOPPED_WITH_BREAK){
	    shutoff();
//		inverter(angle + (int32_t)theta + 360*2, mag, PHASE_ORDER);
		}
	else if (Status == BLDC_RUNNING){
		if(mag > 100)shutoff;
		else
			inverter(angle + (int32_t)theta + 360*2, mag, PHASE_ORDER);
	}
	//--------------send can message------------------ 1us
	//time keepers
	timing_CAN_feedback++;
	running_LED_timing++;

	float CAN_value_fb = IRQ_STATUS_BUFF.setpoint;

	if(Status != BLDC_RUNNING){
		if(IRQ_STATUS_BUFF.status == INPUT_GET_ANGLE_MAX_LIMIT)CAN_value_fb = 0;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_ANGLE_MIN_LIMIT)CAN_value_fb = 0;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_ANGLE_kd)CAN_value_fb = storage->Angle_kd;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_ANGLE_ki)CAN_value_fb = storage->Angle_ki;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_ANGLE_kp)CAN_value_fb = storage->Angle_kp;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_CURRENT_D_kd)CAN_value_fb = storage->Current_kd;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_CURRENT_D_ki)CAN_value_fb = storage->Current_ki;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_CURRENT_D_kp)CAN_value_fb = storage->Current_kp;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_CURRENT_LIMIT)CAN_value_fb = storage->Current_limit;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_CURRENT_Q_kd)CAN_value_fb = storage->Current_offset_kd;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_CURRENT_Q_ki)CAN_value_fb = storage->Current_offset_ki;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_CURRENT_Q_kp)CAN_value_fb = storage->Current_offset_kp;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_ENCODER_1_OFFSET)CAN_value_fb = storage->Encoder1_offset;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_ENCODER_2_GEAR_RATIO)CAN_value_fb = 0;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_ENCODER_2_OFFSET)CAN_value_fb = storage->Encoder2_offset;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_MOTOR_KV)CAN_value_fb = 0;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_VELOCITY_LIMIT)CAN_value_fb = storage->Velocity_limit;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_VELOCITY_kd)CAN_value_fb = storage->Velocity_kd;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_VELOCITY_ki)CAN_value_fb = storage->Velocity_ki;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_VELOCITY_kp)CAN_value_fb = storage->Velocity_kp;
		else if(IRQ_STATUS_BUFF.status == INPUT_GET_VOLTAGE_LIMIT)CAN_value_fb = 0;

		else if(IRQ_STATUS_BUFF.status == INPUT_SET_ANGLE_MAX_LIMIT)CAN_value_fb = 0;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_ANGLE_MIN_LIMIT)CAN_value_fb = 0;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_ANGLE_kd)storage->Angle_kd = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_ANGLE_ki)storage->Angle_ki = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_ANGLE_kp)storage->Angle_kp = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_CURRENT_D_kd)storage->Current_kd = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_CURRENT_D_ki)storage->Current_ki = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_CURRENT_D_kp)storage->Current_kp = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_CURRENT_LIMIT){
			storage->Current_limit = IRQ_STATUS_BUFF.setpoint;
			SetOutputLimits(&Velocity_PID, (storage->Current_limit*-1.0f), (storage->Current_limit));
		}
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_CURRENT_Q_kd)storage->Current_offset_kd = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_CURRENT_Q_ki)storage->Current_offset_ki = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_CURRENT_Q_kp)storage->Current_offset_kp = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_ENCODER_1_OFFSET)storage->Encoder1_offset = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_ENCODER_2_GEAR_RATIO)CAN_value_fb = 0;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_ENCODER_2_OFFSET)storage->Encoder2_offset = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_MOTOR_KV)CAN_value_fb = 0;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_VELOCITY_LIMIT){
			storage->Velocity_limit = IRQ_STATUS_BUFF.setpoint;
			SetOutputLimits(&Angle_PID, (storage->Velocity_limit*-1.0f), (storage->Velocity_limit));
		}
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_VELOCITY_kd)storage->Velocity_kd = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_VELOCITY_ki)storage->Velocity_ki = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_VELOCITY_kp)storage->Velocity_kp = IRQ_STATUS_BUFF.setpoint;
		else if(IRQ_STATUS_BUFF.status == INPUT_SET_VOLTAGE_LIMIT)CAN_value_fb = 0;

		else if(IRQ_STATUS_BUFF.status == INPUT_SAVE_TO_FLASH)Flash_save();

		else CAN_value_fb = NAN;
	}


	if(timing_CAN_feedback >= LOOP_FREQ_KHZ*5){ //every 5ms
		timing_CAN_feedback = 0;
		Feedback.Status_warning = warning;
		Feedback.Status_faults = error;
		Feedback.Status_setpoint = CAN_value_fb;
		Feedback.Status_mode = Status;

		Feedback.Current_Q = q_lpf;
		Feedback.Current_D = d_lpf;

		Feedback.Voltage_AUX = ((float)IRQ_Voltage_Temp_BUFF.V_aux)/1000;
		Feedback.Voltage_BAT = ((float)IRQ_Voltage_Temp_BUFF.V_Bat)/1000;

		Feedback.Temp_ENCODER1 = IRQ_Encoders_BUFF.Encoder1_temp_x10/10;
		Feedback.Temp_ENCODER2 = IRQ_Encoders_BUFF.Encoder2_temp_x10/10;

		Feedback.Position_Encoder1_pos = IRQ_Encoders_BUFF.Encoder1_pos/1000.0f;
		Feedback.Position_Encoder2_pos = IRQ_Encoders_BUFF.Encoder2_pos/1000.0f;
		Feedback.Position_Calculated_pos = Angle_PID.Input;
		Feedback.Position_Velocity = IRQ_Encoders_BUFF.Velocity/1000.0f;
		FDCAN_sendData(&hfdcan1, CAN_TX_ID, (uint8_t*)&Feedback);

		//-----------------PRINTF DEBUGGING-------------------
		//will print same info as on CAN-BUS
	}

	//----------------set status LEDs---------------------
//	if(error)HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
//	else HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
//	if(warning)HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 1);
//	else HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, 0);

	#ifndef RUNNING_LED_DEBUG
	if(running_LED_timing >= LOOP_FREQ_KHZ*100){
		running_LED_timing = 0;
		HAL_GPIO_TogglePin(RUNNING_LED_GPIO_Port, RUNNING_LED_Pin);
	}
	#endif

	//-----------------update dac---------------------------
	#ifdef DAC_DEBUG
	/*
	 *  DAC min
	 *  DAC max
	 *
	 *  current fase a b c
	 *  current q
	 *  current d
	 *  position encoder 1
	 *  position encoder 2
	 *  velocity
	 *
	 */
//	float scale_dac_max = 0.0f;
//	float scale_dac_min = 100.0f;
//	float scale_in = 0;
//	uint32_t DAC_in;
//	if(scale_in > scale_dac_max)scale_out = 4095;
//	else if(scale_in < scale_dac_min)scale_out = 0;
//	else{
//		DAC_in = scale_in
//	}
//	dac_value(q/10 +1500);
	#endif
}

int16_t mech_to_el_deg(int32_t angle_deg, int32_t offset_deg){
	float temp = (float)(angle_deg-offset_deg+360000*2);
	while (temp > 360000) temp = temp-360000;
	while (temp > (360000/17)) temp = temp-deg_pr_pole;
	temp = temp*17/1000;
	while (temp > 360) temp -= 360;
	if(temp < 0) return 0;
	else if(temp > 360) return 360;
	else return (int16_t)temp;
}
