#include "svpwm.h"
#include "math.h"

void svpwm_calc(float Ts, float Valpha, float Vbeta, float* Ta, float* Tb, float* Tc)
{
	float Vref = hypotf(Valpha, Vbeta);
	if (Vref > MMAX) Vref = MMAX;
	float angle = atan2f(Vbeta, Valpha);
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
