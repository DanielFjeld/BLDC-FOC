/*
 * Encoders_SPI.h
 *
 *  Created on: Nov 11, 2023
 *      Author: Daniel
 */

#ifndef ENCODERS_SPI_H_
#define ENCODERS_SPI_H_

	typedef struct Encoders{
		uint32_t Encoder1_pos; 	//0 to 360 000
		uint32_t Encoder2_pos; 	//0 to 360 000
		int32_t Calculated_pos; //-2,147,483,648 to 2,147,483,647 	//DEG/1000
		int32_t Velocity; 		//-2,147,483,648 to 2,147,483,647 	//RPM/100
	}Encoders;

#endif /* ENCODERS_SPI_H_ */
