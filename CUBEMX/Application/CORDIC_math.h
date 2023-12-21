/*
 * CORDIC_math.h
 *
 *  Created on: Dec 17, 2023
 *      Author: Daniel
 */

#ifndef CORDIC_MATH_H_
#define CORDIC_MATH_H_

void RunCordic(float theta, float *cos_out, float *sin_out);
void RunCordic2(float theta, float *cos_out, float *sin_out);
void RunCordic_inverse(float x, float y, volatile float *theta, volatile float *mag);

#endif /* CORDIC_MATH_H_ */
