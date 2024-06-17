/*
 * IIR.h
 *
 *  Created on: Jun 16, 2024
 *      Author: Daniel
 */
#include "main.h"

#ifndef IIR_H_
#define IIR_H_



#define MAX_IIR 16

typedef struct IIR{
	uint8_t size;
	float Coef_a[MAX_IIR];
	float Coef_b[MAX_IIR];

	float last_x[MAX_IIR];
	float last_y[MAX_IIR];
}IIR_t;

float IIR(IIR_t *handler, float in);

//-----Define filters
// Current D
// Current Q
//

//IIR_t LPF_CURRENT_1 = {
//		.size = 3,
//		.Coef_a = {
//			1.0f,
//			-1.889f,
//			0.8949f
//		},
//		.Coef_b = {
//			0.0015f,
//			0.0029f,
//			0.0015f
//		},
//		.last_x = {0},
//		.last_y = {0}
//};



//float data_out= IIR(&LPF_CURRENT_1, data_in);

#endif /* IIR_H_ */
