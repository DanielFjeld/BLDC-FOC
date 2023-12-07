/*
 * BLDC_FOC.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Daniel
 */
/*TODO:
 * Make driver for encoders (velocity and position)
 * encoder calibration (finds magnetic 0 DEG and create offset for encoder)(can be done manually)
 * OK---->Read voltage and temperature (in Current.c)
 * OK---->use buffer after callback to store all values that get changed in IRQ
 * create IIR or FIR filter with built in STM hardware
 * stop if no data or move to another position?
 * Use DAC to measure current
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
#include "CURRENT_adc.h"
#include "fdcandriver.h"
#include "print_server.h"

uint8_t  Current_Callback_flag = 0;


//----------------------IRQ--------------------
Current IRQ_Current = {0};
Voltage_Temp IRQ_Voltage_Temp = {0};

//----------------------CAN--------------------
CAN_Status IRQ_Status;
CAN_Status Status_send;
CAN_Feedback Feedback;

//PID
//CAN_PID PID_Current;
//CAN_PID PID_Velocity;
//CAN_PID PID_Angle;
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


//------------------------MAIN-------------------------
void BLDC_main(void){
	HAL_Delay(1000);

	//setup current
	current_init((void*)&Current_IRQ);
	//setup voltage and temperature readings
	voltage_temperature_init((void*)&Voltage_Temp_IRQ);

	//setup CAN
	//-----------------CAN----------------------
	//FDCAN_addCallback(&hfdcan1, (CAN_STATUS_ID << 8) 		| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_Status_IRQ);


//	FDCAN_addCallback(&hfdcan1, (CAN_LIMITS_ID << 8) 		| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_Limits_IRQ);
//	FDCAN_addCallback(&hfdcan1, (CAN_PID_CURRENT_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_PID_Current_IRQ);
//	FDCAN_addCallback(&hfdcan1, (CAN_PID_VELOCITY_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_PID_Velocity_IRQ);
//	FDCAN_addCallback(&hfdcan1, (CAN_PID_ANGLE_ID << 8) 	| (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (void*)&Can_RX_PID_Angle_IRQ);

	FDCAN_Start(&hfdcan1);

	//--------------setup PWM------------------

	while(1){
		HAL_Delay(10);

		Status_send.setpoint = IRQ_Voltage_Temp.V_Bat*2-360000;

		if(HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin))Status_send.setpoint = 0;
		else Status_send.setpoint = 360000*5;
		Status_send.status = INPUT_START;

//		if(HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin))Status_send.status = INPUT_STOP_WITH_BREAK;
//		else Status_send.status = INPUT_START;
		Status_send.reset_faults = 0;

		FDCAN_sendData(&hfdcan1, (CAN_STATUS_ID << 8) | (CAN_DEVICE_ID << 4) | (CAN_BLDC_ID << 0), (uint8_t*)&Status_send);

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
				"STATUS[MODE:%s SP:%2d WARN:0x%02x ERROR:0x%02x]"
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
}
