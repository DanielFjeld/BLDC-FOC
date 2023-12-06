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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if (hspi == &hspi1) {
		HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 1);
		data_encoders.Encoder1_temp_x10 = (SPI1_rx_buff[3] << 8) | (SPI1_rx_buff[2]);
		data_encoders.Encoder1_pos = (SPI1_rx_buff[1] << 6) | (SPI1_rx_buff[0] >> 2);
	}
	if (hspi == &hspi3) {
		HAL_GPIO_WritePin(ENCODER2_CS_GPIO_Port, ENCODER2_CS_Pin, 1);
		data_encoders.Encoder2_temp_x10 = (SPI3_rx_buff[3] << 8) | (SPI3_rx_buff[2]);
		data_encoders.Encoder2_pos = (SPI3_rx_buff[1] << 6) | (SPI3_rx_buff[0] >> 2);

	}
	data_encoders.Calculated_pos = 10;

	Encoders_IRQ_callback(&data_encoders);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	HAL_GPIO_WritePin(ENCODER1_CS_GPIO_Port, ENCODER1_CS_Pin, 0);
	HAL_GPIO_WritePin(ENCODER2_CS_GPIO_Port, ENCODER2_CS_Pin, 0);
//	HAL_SPI_TransmitReceive_DMA(&hspi1, SPI1_tx_buff, SPI1_rx_buff, ORBIS_SPI_SIZE);
//	HAL_SPI_TransmitReceive_DMA(&hspi3, SPI1_tx_buff, SPI1_rx_buff, ORBIS_SPI_SIZE);
}
