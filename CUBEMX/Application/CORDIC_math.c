/*
 * CORDIC_math.h
 *
 *  Created on: Dec 17, 2023
 *      Author: Daniel
 */
#include "cordic.h"
#include "CORDIC_math.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_cordic.h"


#define CORDIC_CONFIG_COSINE (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_4CYCLES | LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 | LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)
#define CORDIC_CONFIG_MODULUS (LL_CORDIC_FUNCTION_MODULUS | LL_CORDIC_PRECISION_8CYCLES | LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 | LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)

void RunCordic(float theta, float *cos_out, float *sin_out) {
	CORDIC->CSR = CORDIC_CONFIG_COSINE;
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
void RunCordic2(float theta, float *cos_out, float *sin_out) {
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
	/* CORDIC FUNCTION: COSINE q1.15 */

	unsigned int cordicin = 0x7fff0000;  //  mag = 1
	short thetashort = theta*10430;       // wrap it
	cordicin += thetashort;

	LL_CORDIC_WriteData(CORDIC, 0x7FFF0000 + (uint32_t) cordicin);
	int16_t out0 = LL_CORDIC_ReadData(CORDIC);

	short out2 = (out0&0xffff0000)>>16;
	short out1 = out0&0xffff;  //
	*cos_out = (float)out1/32768.0f;
	*sin_out = (float)out2/32768.0f;
}

void RunCordic_inverse(float x, float y, volatile float *theta, volatile float *mag) {
	CORDIC->CSR = CORDIC_CONFIG_MODULUS;

	CORDIC->WDATA = (uint32_t)((int16_t)(x * 32768.0f)) | ((uint32_t)((int16_t)(y * 32768.0f ))<<16);
	uint32_t out1 =  CORDIC->RDATA;

	*theta = (float)(int16_t)(out1>>16)/32768.0f;
	*mag  = (float)(out1&0xFFFF)/32768.0f;
}
