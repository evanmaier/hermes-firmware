/*
 * transforms.c
 *
 *  Created on: Jul 10, 2023
 *      Author: evant
 */

#include "transforms.h"
#include "stdio.h"

//void Clarke_Transform(double* In_a, double* In_b, double* In_c, double* Out_alpha, double* Out_beta)
//{
//	(*Out_alpha) = (*In_a) - (*In_b) * 0.5 - (*In_c) * 0.5;
//	(*Out_beta) = ((*In_b) - (*In_c)) * 0.866;
//}
//void InvClarke_Transform(double* In_alpha, double* In_beta, double* Out_a, double* Out_b, double* Out_c)
//{
//	(*Out_a) = (*In_alpha);
//	(*Out_b) = -0.5 * (*In_alpha) + 0.8660 * (*In_beta);
//	(*Out_c) = -0.5 * (*In_alpha) - 0.8660 * (*In_beta);
//}
//void Park_Transform(double* In_alpha, double* In_beta, double* In_angle, double* Out_d, double* Out_q)
//{
//	(*Out_d) = (*In_alpha) * cos((*In_angle)) + (*In_beta) * sin((*In_angle));
//	(*Out_q) = (*In_beta) * cos((*In_angle)) - (*In_alpha) * sin((*In_angle));
//}
//void InvPark_Transform(double* In_d, double* In_q, double* In_angle, double* Out_alpha, double* Out_beta)
//{
//	(*Out_alpha) = (*In_d) * cos((*In_angle)) - (*In_q) * sin((*In_angle));
//	(*Out_beta) = (*In_d) * sin((*In_angle)) + (*In_q) * cos((*In_angle));
//}
void Test_Transforms()
{
	float a_exp = 0, b_exp = 1.225, d_exp = 1, q_exp = 1, alpha_exp = 0, beta_exp = 1.414;
	float error = 0.01;
	float theta = 45;
	float a, b, d , q , alpha , beta , sinval, cosval;

	// calculate sin and cos values from theta
	arm_sin_cos_f32(theta, &sinval, &cosval);

	// test clarke transform
	arm_clarke_f32(a_exp, b_exp, &alpha, &beta);
	if ((alpha - alpha_exp) > error || (alpha_exp - alpha) > error || (beta - beta_exp) > error || (beta_exp - beta) > error)
	{
		printf("Clarke Transform Failed");
	}

	// test inverse clarke transform
	arm_inv_clarke_f32(alpha_exp, beta_exp, &a, &b);
	if ((a - a_exp) > error || (a_exp - a) > error || (b - b_exp) > error || (b_exp - b) > error)
	{
		printf("InvClarke_Transform Failed");
	}

	// test park transform
	arm_park_f32(alpha_exp, beta_exp, &d, &q, sinval, cosval);
	if ((d - d_exp) > error || (d_exp - d) > error || (q - q_exp) > error || (q_exp - q) > error)
	{
		printf("Park_Transform Failed");
	}

	// test inverse park transform
	arm_inv_park_f32(d_exp, q_exp, &alpha, &beta, sinval, cosval);
	if ((alpha - alpha_exp) > error || (alpha_exp - alpha) > error || (beta - beta_exp) > error || (beta_exp - beta) > error)
	{
		printf("InvPark_Transform Failed");
	}
}
