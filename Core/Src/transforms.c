/*
 * transforms.c
 *
 *  Created on: Jul 10, 2023
 *      Author: evant
 */

#include "transforms.h"

void Clarke_Transform(float* In_a, float* In_b, float* In_c, float* Out_alpha, float* Out_beta)
{
	(*Out_alpha) = (*In_a) - (*In_b) * 0.5 - (*In_c) * 0.5;
	(*Out_beta) = ((*In_b) - (*In_c)) * 0.866;
}
void InvClarke_Transform(float* In_alpha, float* In_beta, float* Out_a, float* Out_b, float* Out_c)
{
	(*Out_a) = (*In_alpha);
	(*Out_b) = -0.5 * (*In_alpha) + 0.8660 * (*In_beta);
	(*Out_c) = -0.5 * (*In_alpha) - 0.8660 * (*In_beta);
}
void Park_Transform(float* In_alpha, float* In_beta, float* In_angle, float* Out_d, float* Out_q)
{
	(*Out_d) = (*In_alpha) * cosf((*In_angle)) + (*In_beta) * sinf((*In_angle));
	(*Out_q) = (*In_beta) * cosf((*In_angle)) - (*In_alpha) * sinf((*In_angle));
}
void InvPark_Transform(float* In_d, float* In_q, float* In_angle, float* Out_alpha, float* Out_beta)
{
	(*Out_alpha) = (*In_d) * cosf((*In_angle)) - (*In_q) * sinf((*In_angle));
	(*Out_beta) = (*In_d) * sinf((*In_angle)) + (*In_q) * cosf((*In_angle));
}
