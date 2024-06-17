/*
 * IIR.c
 *
 *  Created on: Jun 16, 2024
 *      Author: Daniel
 */

#include "IIR.h"

float IIR(IIR_t *handler, float in){

	//----------SHIFT LAST VALUES
	for(int i = handler->size; i > 0; i--)handler->last_y[i] = handler->last_y[i-1]; //delay input
	for(int i = handler->size; i > 0; i--)handler->last_x[i] = handler->last_x[i-1]; //delay output
	handler->last_x[0] = in;

	//----------CALCULATE NEXT OUTPUT
	float y = 0;
	for(int i = 0; i <= handler->size; i++)y +=  handler->Coef_b[i]*handler->last_x[i];
	for(int i = 1; i <= handler->size; i++)y -=  handler->Coef_a[i]*handler->last_y[i];
	handler->last_y[0] = y;

	//----------RETURN OUTPUT
	return y;
}
