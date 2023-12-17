/*
 * CORDIC_math.h
 *
 *  Created on: Dec 17, 2023
 *      Author: Daniel
 */
#include "cordic.h"
#include "CORDIC_math.h"

void RunCordic(float theta, float *cos_out, float *sin_out) {
	unsigned int cordicin = 0x7fff0000;  //  mag = 1
	short thetashort = theta*10430;       // wrap it
	cordicin += thetashort;
	CORDIC->WDATA = cordicin;
	unsigned int out0 = CORDIC->RDATA;
	short out2 = (out0&0xffff0000)>>16;
	short out1 = out0&0xffff;  //
	*cos_out = (float)out1/32768.0f;
	*sin_out = (float)out2/32768.0f;
}
