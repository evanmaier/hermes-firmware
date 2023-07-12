/*
 * transforms.c
 *
 *  Created on: Jul 10, 2023
 *      Author: evant
 */

#include "transforms.h"
#include "stdio.h"

void Clarke_Transform(double* In_a, double* In_b, double* In_c, double* Out_alpha, double* Out_beta)
{
	(*Out_alpha) = (*In_a) - (*In_b) * 0.5 - (*In_c) * 0.5;
	(*Out_beta) = ((*In_b) - (*In_c)) * 0.866;
}
void InvClarke_Transform(double* In_alpha, double* In_beta, double* Out_a, double* Out_b, double* Out_c)
{
	(*Out_a) = (*In_alpha);
	(*Out_b) = -0.5 * (*In_alpha) + 0.8660 * (*In_beta);
	(*Out_c) = -0.5 * (*In_alpha) - 0.8660 * (*In_beta);
}
void Park_Transform(double* In_alpha, double* In_beta, double* In_angle, double* Out_d, double* Out_q)
{
	(*Out_d) = (*In_alpha) * cos((*In_angle)) + (*In_beta) * sin((*In_angle));
	(*Out_q) = (*In_beta) * cos((*In_angle)) - (*In_alpha) * sin((*In_angle));
}
void InvPark_Transform(double* In_d, double* In_q, double* In_angle, double* Out_alpha, double* Out_beta)
{
	(*Out_alpha) = (*In_d) * cos((*In_angle)) - (*In_q) * sin((*In_angle));
	(*Out_beta) = (*In_d) * sin((*In_angle)) + (*In_q) * cos((*In_angle));
}
void Test_Transforms()
{
	//a_exp = 1, b_exp = 2, c_exp = 3, d_exp = -1.5, q_exp = 0.866025, alpha_exp = -1.5, beta_exp = -0.866025;
	double error = 0.001;
	double a = 1, b = 2, c = 3, d = -1.5, q = 0.866025, alpha = -1.5, beta = -0.866025, angle = 0;
	Clarke_Transform(&a, &b, &c, &alpha, &beta);
	if ((alpha - (-1.5)) > error || (beta - (-0.866025)) > error)
	{
		printf("Clarke Transform Failed");
	}
	InvClarke_Transform(&alpha, &beta, &a, &b, &c);
	if ((a - 1) > error || (b - 2) > error || (c - 3) > error)
	{
		printf("InvClarke_Transform Failed");
	}
	Park_Transform(&alpha, &beta, &angle, &d, &q);
	if ((d - (-1.5)) > error || (q - 0.886025) > error)
	{
		printf("Park_Transform Failed");
	}
	InvPark_Transform(&d, &q, &angle, &alpha, &beta);
	if ((alpha - (-1.5)) > error || (beta - (-0.866025)) > error)
	{
		printf("InvPark_Transform Failed");
	}
}
