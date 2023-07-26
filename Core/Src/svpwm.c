#include "svpwm.h"
#include "math.h"

void svpwm_calc(float Ts, float Valpha, float Vbeta, float* Ta, float* Tb, float* Tc)
{
	float Vref = hypotf(Valpha, Vbeta);
	if (Vref > MMAX) Vref = MMAX;
	float angle = atan2f(Vbeta, Valpha);
	if (angle < 0) angle += M_TWOPI;
	int sector = angle / PIdiv3 + 1;

	float T1 = TWODIVSQRT3 * Ts *(Vref / VDC) * sinf(sector * PIdiv3 - angle);
	float T2 = TWODIVSQRT3 * Ts * (Vref / VDC) * sinf(angle - (sector - 1) * PIdiv3);
	float T0 = Ts - T1 - T2;
	float T0_half = T0/2;

	switch(sector){
		case 1:
			*Ta = T0_half;
			*Tb = T0_half + T1;
			*Tc = Ts - T0_half;
			break;
		case 2:
		   *Ta = T0_half + T2;
		   *Tb = T0_half;
		   *Tc = Ts - T0_half;
		   break;
		 case 3:
		   *Ta = Ts - T0_half;
		   *Tb = T0_half;
		   *Tc = T0_half + T1;
		   break;
		 case 4:
		   *Ta = Ts - T0_half;
		   *Tb = T0_half + T2;
		   *Tc = T0_half;
		   break;
		 case 5:
		   *Ta = T0_half + T1;
		   *Tb = Ts - T0_half;
		   *Tc = T0_half;
		   break;
		 case 6:
		   *Ta = T0_half;
		   *Tb = Ts - T0_half;
		   *Tc = T0_half + T2;;
		   break;
		 default:
		  // possible error state
		   *Ta = 0;
		   *Tb = 0;
		   *Tc = 0;
	}

}

void svpwm_apply(TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim4, float Fcounter, float Ta, float Tb, float Tc)
{
	TIM3->CCR1 = Ta * Fcounter; // Q1
	TIM3->CCR2 = Tb * Fcounter; // Q3
	TIM3->CCR3 = Tc * Fcounter; // Q5
	TIM4->CCR1 = Ta * Fcounter; // Q0
	TIM4->CCR2 = Tb * Fcounter; // Q2
	TIM4->CCR3 = Tc * Fcounter; // Q4
}

void svpwm_start(TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim4)
{
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim4, TIM_CHANNEL_3);
}
