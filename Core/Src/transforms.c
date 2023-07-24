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

//void Test_Transforms()
//{
//	float a_exp = 0, b_exp = 1.225, d_exp = 1, q_exp = 1, alpha_exp = 0, beta_exp = 1.414;
//	float error = 0.01;
//	float theta = 45;
//	float a, b, d , q , alpha , beta , sinval, cosval;
//
//	// calculate sinf and cosf values from theta
//	arm_sinf_cosf_f32(theta, &sinval, &cosval);
//
//	// test clarke transform
//	arm_clarke_f32(a_exp, b_exp, &alpha, &beta);
//	if ((alpha - alpha_exp) > error || (alpha_exp - alpha) > error || (beta - beta_exp) > error || (beta_exp - beta) > error)
//	{
//		printf("Clarke Transform Failed");
//	}
//
//	// test inverse clarke transform
//	arm_inv_clarke_f32(alpha_exp, beta_exp, &a, &b);
//	if ((a - a_exp) > error || (a_exp - a) > error || (b - b_exp) > error || (b_exp - b) > error)
//	{
//		printf("InvClarke_Transform Failed");
//	}
//
//	// test park transform
//	arm_park_f32(alpha_exp, beta_exp, &d, &q, sinval, cosval);
//	if ((d - d_exp) > error || (d_exp - d) > error || (q - q_exp) > error || (q_exp - q) > error)
//	{
//		printf("Park_Transform Failed");
//	}
//
//	// test inverse park transform
//	arm_inv_park_f32(d_exp, q_exp, &alpha, &beta, sinval, cosval);
//	if ((alpha - alpha_exp) > error || (alpha_exp - alpha) > error || (beta - beta_exp) > error || (beta_exp - beta) > error)
//	{
//		printf("InvPark_Transform Failed");
//	}
//}
