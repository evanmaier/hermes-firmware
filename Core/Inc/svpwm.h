#ifndef __SVPWM_H__
#define __SVPWM_H__

#include "main.h"

#define TWODIVSQRT3 1.15470054f
#define PIdiv3 	    1.04719755f
#define VDC 		1.0f
#define MMAX 		0.866f

void svpwm_calc(float, float , float , float* , float* , float*);

void svpwm_apply(TIM_HandleTypeDef*, TIM_HandleTypeDef*, float, float, float, float);

void svpwm_start(TIM_HandleTypeDef*, TIM_HandleTypeDef*);

#endif
