/*
 * Encoders_SPI.c
 *
 *  Created on: Nov 11, 2023
 *      Author: Daniel
 */

#include "main.h"
#include <stdlib.h>
#include "spi.h"

#include "Encoders_SPI.h"

#define ORBIS_SPI_SIZE 10
#define ORBIS_NORNAL_OPERATION 't'
#define ORBIS_ERROR_OPERATION 'd'

uint8_t SPI1_tx_buff[ORBIS_SPI_SIZE] = {0};
uint8_t SPI1_rx_buff[ORBIS_SPI_SIZE];


/*
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
 *
 *
 *
 */
void ORBIS_init(){
	//setup callback

	//setup DMA
	SPI1_tx_buff[0] = ORBIS_NORNAL_OPERATION;
	HAL_SPI_TransmitReceive_DMA(&hspi1, SPI1_tx_buff, SPI1_rx_buff, ORBIS_SPI_SIZE); //
	//setup timer

}
