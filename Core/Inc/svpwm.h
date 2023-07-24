#ifndef __SVPWM_H__
#define __SVPWM_H__

#define TWODIVSQRT3 1.15470054f
#define PIdiv3 	    1.04719755f
#define VDC 		1.0f
#define MMAX 		0.866f

void svpwm_calc(float, float , float , float* , float* , float*);

#endif
