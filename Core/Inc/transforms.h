/*
 * transforms.h
 *
 *  Created on: Jul 10, 2023
 *      Author: evant
 */

#include "math.h"

#ifndef TRANSFORMS_H_
#define TRANSFORMS_H_

void Clarke_Transform(float* In_a, float* In_b, float* In_c, float* Out_alpha, float* Out_beta);
void InvClarke_Transform(float* In_alpha, 	float* In_beta, float* Out_a, float* Out_b, float* Out_c);
void Park_Transform(float* In_alpha, float* In_beta, float* In_angle, float* Out_d, float* Out_q);
void InvPark_Transform(float* In_d, float* In_q, float* In_angle, float* Out_alpha, float* Out_beta);

#endif /* TRANSFORMS_H_ */
