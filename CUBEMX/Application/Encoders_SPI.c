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

#define ORBIS_SPI_SIZE 5
#define ORBIS_NORNAL_OPERATION 't'
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
void ORBIS_init(Encoders_Callback __IRQ_callback){
	HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 1);
	HAL_GPIO_WritePin(ENCODER2_CS_GPIO_Port, ENCODER2_CS_Pin, 1);
	//setup callback
	Encoders_IRQ_callback = __IRQ_callback;

	//setup DMA
	SPI1_tx_buff[0] = ORBIS_NORNAL_OPERATION;
	HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 0);
	HAL_GPIO_WritePin(ENCODER2_CS_GPIO_Port, ENCODER2_CS_Pin, 0);
	HAL_SPI_TransmitReceive_DMA(&hspi1, SPI1_tx_buff, SPI1_rx_buff, ORBIS_SPI_SIZE);
	HAL_SPI_TransmitReceive_DMA(&hspi3, SPI3_tx_buff, SPI3_rx_buff, ORBIS_SPI_SIZE);

	//setup timer
	HAL_TIM_Base_Start_IT(&htim3);

}

float velocity_accumulate;
int32_t last_pos = 0;
#define velocity_lpf_size 4
uint8_t velocity_index = 0;
float velocity_array[velocity_lpf_size] = {0};


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if (hspi == &hspi1) {
		HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 1);
		data_encoders.Calculated_pos = SPI1_rx_buff[0]; //(SPI1_rx_buff[3] << 8) | (SPI1_rx_buff[2]);
		data_encoders.Encoder1_pos = (uint32_t)(((uint32_t)(SPI1_rx_buff[0] << 6) | (SPI1_rx_buff[1] >> 2)) * 5625) >> 8;
		data_encoders.Encoder1_pos_raw = (uint32_t)(SPI1_rx_buff[0] << 6) | (SPI1_rx_buff[1] >> 2);

		float velocity_temp;
		if(((int32_t)data_encoders.Encoder1_pos - last_pos) > 180000)velocity_temp = ((int32_t)data_encoders.Encoder1_pos-last_pos - 360000);
		else if(((int32_t)data_encoders.Encoder1_pos - last_pos) < -180000)velocity_temp = ((int32_t)data_encoders.Encoder1_pos-last_pos + 360000);
		else velocity_temp = ((int32_t)data_encoders.Encoder1_pos-last_pos);
		last_pos = (int32_t)data_encoders.Encoder1_pos;

		velocity_accumulate += velocity_temp;
		velocity_accumulate -= velocity_array[velocity_index];
		velocity_array[velocity_index] = velocity_temp;
		velocity_index++;
		if (velocity_index == velocity_lpf_size)velocity_index = 0;

		data_encoders.Velocity = (int32_t)((velocity_accumulate*10000.0f*60.0f)/360.0f/velocity_lpf_size);
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

	HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
	run();
	HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
}
