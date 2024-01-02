/*
 * FMAC_filter.c
 *
 *  Created on: Dec 21, 2023
 *      Author: Daniel
 */
#include "main.h"
#include "stdint.h"
#include "FMAC_filter.h"
#include "fmac.h"
#include "stm32g4xx_ll_fmac.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal.h"
#include "time.h"

/* Declare an array to hold the filter coefficients */
static int16_t aFilterCoeffB[51];

void FMAC_init(){
	__HAL_RCC_FMAC_CLK_ENABLE();

	/* declare a filter configuration structure */
	FMAC_FilterConfigTypeDef sFmacConfig;
	/* Set the coefficient buffer base address */
	sFmacConfig.CoeffBaseAddress = 0;
	/* Set the coefficient buffer size to the number of coeffs */
	sFmacConfig.CoeffBufferSize = 51;
	/* Set the Input buffer base address to the next free address */
	sFmacConfig.InputBaseAddress = 51;
	/* Set the input buffer size greater than the number of coeffs */
	sFmacConfig.InputBufferSize = 100;
	/* Set the input watermark to zero since we are using DMA */
	sFmacConfig.InputThreshold = 0;
	/* Set the Output buffer base address to the next free address */
	sFmacConfig.OutputBaseAddress = 151;
	/* Set the output buffer size */
	sFmacConfig.OutputBufferSize = 100;
	/* Set the output watermark to zero since we are using DMA */
	sFmacConfig.OutputThreshold = 0;
	/* No A coefficients since FIR */
	sFmacConfig.pCoeffA = NULL;
	sFmacConfig.CoeffASize = 0;
	/* Pointer to the coefficients in memory */
	sFmacConfig.pCoeffB = aFilterCoeffB;
	/* Number of coefficients */
	sFmacConfig.CoeffBSize = 51;
	/* Select FIR filter function */
	sFmacConfig.Filter = FMAC_FUNC_CONVO_FIR;
	/* Enable DMA input transfer */
	sFmacConfig.InputAccess = FMAC_BUFFER_ACCESS_DMA;
	/* Enable DMA output transfer */
	sFmacConfig.OutputAccess = FMAC_BUFFER_ACCESS_DMA;
	/* Enable clipping of the output at 0x7FFF and 0x8000 */
	sFmacConfig.Clip = FMAC_CLIP_ENABLED;
	/* P parameter contains number of coefficients */
	sFmacConfig.P = 51;
	/* Q parameter is not used */
	sFmacConfig.Q = 0;
	/* R parameter contains the post-shift value (none) */
	sFmacConfig.R = 0;
	/* Configure the FMAC */
	if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK)
		/* Configuration Error */
		Error_Handler();
}

// Structure to hold the state of the low pass filter
typedef struct {
    float y_prev;
    unsigned long timestamp_prev;
    float Tf; // Time constant of the filter
    float dt;
    float input;
} LowPassFilter;

// Low pass filtering function
float lowPassFilter(LowPassFilter* filter) {
    unsigned long timestamp = micros();
    float dt = (timestamp - filter->timestamp_prev) * 1e-6f;
    // Quick fix for strange cases (micros overflow)
    if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;

    // Calculate the filtering
    float alpha = filter->Tf / (filter->Tf + filter->dt);
    float y = alpha * filter->y_prev + (1.0f - alpha) * filter->input;

    // Save the variables
    filter->y_prev = y;
    filter->timestamp_prev = timestamp;
    return y;
}
