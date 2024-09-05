/*
 * Encoders_SPI.c
 *
 *  Created on: Nov 11, 2023
 *      Author: Daniel
 */

#include "main.h"
#include <stdlib.h>
#include "spi.h"
#include "tim.h"

#include "Encoders_SPI.h"
#include "BLDC_FOC.h"
#include "Calibration.h"

uint8_t CRC_SPI_97_64bit(uint64_t dw_InputData);

#define ORBIS_SPI_SIZE 5
#define ORBIS_NORNAL_OPERATION 't'

uint8_t ORBIS_CALIBRATE_SEQ[] = {0xCD, 0xEF, 0x89, 0xAB, 0x41};
//uint8_t ORBIS_CALIBRATE_SEQ[] = {0x41, 0xAB, 0x89, 0xEF, 0xCD};
//uint8_t ORBIS_CALIBRATE_SEQ[] = {0, 0, 0, 0, 'A'};
//#define ORBIS_ERROR_OPERATION 'd'

uint8_t SPI1_tx_buff[ORBIS_SPI_SIZE] = {0};
uint8_t SPI1_rx_buff[ORBIS_SPI_SIZE];
uint8_t SPI3_tx_buff[ORBIS_SPI_SIZE] = {0};
uint8_t SPI3_rx_buff[ORBIS_SPI_SIZE];

Encoders_Callback Encoders_IRQ_callback;
Encoders data_encoders;

uint32_t velocity_calc_encoder1[10] = {0};
uint8_t velocity_calc_index_encoder1 = 0;


/* 10kHz update
 *
 * SPI ORBIS ENCODERS
 *
 * request command "t"=0x74 (temperature)
 * b15-b2   14bit encoder position
 * b1-b0    2bit status
 *
 *  r15-r0  temperature "t"
 *  r7-r0   status "d"
 *
 *  c7-c0   CRC
 *
 *  can transmit the command while receiving position
 *
 *  max 4MHz
 */
void ORBIS_init(Encoders_Callback __IRQ_callback, uint8_t calibrate_on_start){
	HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 1);
	HAL_GPIO_WritePin(ENCODER2_CS_GPIO_Port, ENCODER2_CS_Pin, 1);
	//setup callback
	Encoders_IRQ_callback = __IRQ_callback;
	//while(1){

	//calibrate
	if(calibrate_on_start){
		for(int i = 0; i < 5; i++){
			HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 0);
			HAL_Delay(1);
			HAL_SPI_Transmit(&hspi1, &ORBIS_CALIBRATE_SEQ[i], 1, 100);
			HAL_Delay(1);
			HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 1);
			HAL_Delay(1);
		}
		HAL_Delay(1000);
		rotate_inverter(5);
		HAL_Delay(1000);
	}



	//}
	//setup DMA
	SPI1_tx_buff[0] = ORBIS_NORNAL_OPERATION;
	HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 0);
	HAL_GPIO_WritePin(ENCODER2_CS_GPIO_Port, ENCODER2_CS_Pin, 0);
	HAL_SPI_TransmitReceive_DMA(&hspi1, SPI1_tx_buff, SPI1_rx_buff, ORBIS_SPI_SIZE);
	HAL_SPI_TransmitReceive_DMA(&hspi3, SPI3_tx_buff, SPI3_rx_buff, ORBIS_SPI_SIZE);


}

float velocity_accumulate;
int32_t last_pos = 0;
#define velocity_lpf_size 30
uint8_t velocity_index = 0;
float velocity_array[velocity_lpf_size] = {0};

int32_t last_pos_dt = -1;

uint32_t aaa_test_pos_delete1 = 0;
uint32_t aaa_test_pos_delete2 = 0;
uint32_t aaa_test_pos_delete3 = 0;

float velocity_calc;
int16_t crc_error_count = 0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if (hspi == &hspi1) {
		HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 1);

		uint64_t dw_CRCinputData = 0;
		uint8_t calculated_crc=0;

		dw_CRCinputData = ((uint64_t)SPI1_rx_buff[0] << 24) +
				 ((uint64_t)SPI1_rx_buff[1] << 16) + ((uint64_t)SPI1_rx_buff[2] << 8) +
				 ((uint64_t)SPI1_rx_buff[3] << 0); //data without crc on the end!!
		calculated_crc = ~(CRC_SPI_97_64bit(dw_CRCinputData))& 0xFF; //inverted CRC

		uint8_t crc = SPI1_rx_buff[4];

		if(crc_error_count > 10){
			//encoder timeout CRC 1ms

			//TODO: add abort sequence
		}
		if(calculated_crc != crc){
			crc_error_count++;
			return;
		}
		if(crc_error_count > 0)crc_error_count--;


		//data_encoders.Calculated_pos = SPI1_rx_buff[0]; //(SPI1_rx_buff[3] << 8) | (SPI1_rx_buff[2]);
		int32_t temp_Encoder1_pos = (int32_t)(((uint32_t)(SPI1_rx_buff[0] << 6) | (SPI1_rx_buff[1] >> 2)) * 5625) >> 8;
		data_encoders.Encoder1_pos_raw = (uint32_t)(SPI1_rx_buff[0] << 6) | (SPI1_rx_buff[1] >> 2);

		if(last_pos_dt < 0)last_pos_dt = temp_Encoder1_pos; //startup

		if(temp_Encoder1_pos > 0){
			last_pos_dt = temp_Encoder1_pos;}

		data_encoders.Encoder1_pos = last_pos_dt;
		aaa_test_pos_delete1 = last_pos_dt;
		aaa_test_pos_delete2 = last_pos_dt;
		aaa_test_pos_delete3 = last_pos_dt;

		float velocity_temp;
		if(((int32_t)data_encoders.Encoder1_pos - last_pos) > 180000)velocity_temp = ((int32_t)data_encoders.Encoder1_pos-last_pos - 360000);
		else if(((int32_t)data_encoders.Encoder1_pos - last_pos) < -180000)velocity_temp = ((int32_t)data_encoders.Encoder1_pos-last_pos + 360000);
		else velocity_temp = ((int32_t)data_encoders.Encoder1_pos-last_pos);
		last_pos = data_encoders.Encoder1_pos;

		velocity_accumulate += velocity_temp;
		velocity_accumulate -= velocity_array[velocity_index];
		velocity_array[velocity_index] = velocity_temp;
		velocity_index++;
		if (velocity_index == velocity_lpf_size)velocity_index = 0;
		velocity_calc = ((velocity_accumulate*10000.0f*60.0f)/360.0f/velocity_lpf_size)/1000.0f;
		data_encoders.Velocity = velocity_calc;
		data_encoders.Encoder1_temp_x10 = (int16_t)(((uint16_t)(SPI1_rx_buff[2] << 8 | (SPI1_rx_buff[3]))));
	}
	if (hspi == &hspi3) {
		HAL_GPIO_WritePin(ENCODER2_CS_GPIO_Port, ENCODER2_CS_Pin, 1);
		data_encoders.Encoder2_temp_x10 = (SPI3_rx_buff[3] << 8) | (SPI3_rx_buff[2]);
		data_encoders.Encoder2_pos = (SPI3_rx_buff[0] << 6) | (SPI3_rx_buff[1] >> 2);

	}
	//data_encoders.Calculated_pos = 10;

	Encoders_IRQ_callback(&data_encoders);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 0);
	HAL_GPIO_WritePin(ENCODER2_CS_GPIO_Port, ENCODER2_CS_Pin, 0);
	HAL_SPI_TransmitReceive_DMA(&hspi1, SPI1_tx_buff, SPI1_rx_buff, ORBIS_SPI_SIZE);
	HAL_SPI_TransmitReceive_DMA(&hspi3, SPI3_tx_buff, SPI3_rx_buff, ORBIS_SPI_SIZE);

	//HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
	run();
	//HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
}


//Lookup table for polynome = 0x97
static const uint8_t ab_CRC8_LUT[256] = {
0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C};
/* CRC 0x97 Polynomial, 64-bit input data, right alignment, calculation over 64 bits */
uint8_t CRC_SPI_97_64bit(uint64_t dw_InputData)
{
uint8_t b_Index = 0;
uint8_t b_CRC = 0;
 b_Index = (uint8_t)((dw_InputData >> 56u) & (uint64_t)0x000000FFu);
 b_CRC = (uint8_t)((dw_InputData >> 48u) & (uint64_t)0x000000FFu);
 b_Index = b_CRC ^ ab_CRC8_LUT[b_Index];
 b_CRC = (uint8_t)((dw_InputData >> 40u) & (uint64_t)0x000000FFu);
 b_Index = b_CRC ^ ab_CRC8_LUT[b_Index];
 b_CRC = (uint8_t)((dw_InputData >> 32u) & (uint64_t)0x000000FFu);
 b_Index = b_CRC ^ ab_CRC8_LUT[b_Index];
 b_CRC = (uint8_t)((dw_InputData >> 24u) & (uint64_t)0x000000FFu);
 b_Index = b_CRC ^ ab_CRC8_LUT[b_Index];
 b_CRC = (uint8_t)((dw_InputData >> 16u) & (uint64_t)0x000000FFu);
 b_Index = b_CRC ^ ab_CRC8_LUT[b_Index];
 b_CRC = (uint8_t)((dw_InputData >> 8u) & (uint64_t)0x000000FFu);
 b_Index = b_CRC ^ ab_CRC8_LUT[b_Index];
 b_CRC = (uint8_t)(dw_InputData & (uint64_t)0x000000FFu);
 b_Index = b_CRC ^ ab_CRC8_LUT[b_Index];
 b_CRC = ab_CRC8_LUT[b_Index];

 return b_CRC;
}




