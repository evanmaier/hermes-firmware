#include "svpwm.h"
#include "math.h"

void svpwm_calc(float Valpha, float Vbeta, float* Ta, float* Tb, float* Tc)
{
	float Vref = hypotf(Valpha, Vbeta);
	float angle = atan2f(Vbeta, Valpha);
	int sector = angle / PIdiv3 + 1;
	float T1 = SQRT3 * (Vref / VDC) * sinf(sector * PIdiv3 - angle);
	float T2 = SQRT3 * (Vref / VDC) * sinf(angle - (sector - 1) * PIdiv3);
	float T0 = 1 - T1 - T2; // TS = 1
	switch(sector){
		case 1:
			*Ta = T1 + T2 + T0/2;
			*Tb = T2 + T0/2;
			*Tc = T0/2;
			break;
		case 2:
		   *Ta = T1 +  T0/2;
		   *Tb = T1 + T2 + T0/2;
		   *Tc = T0/2;
		   break;
		 case 3:
		   *Ta = T0/2;
		   *Tb = T1 + T2 + T0/2;
		   *Tc = T2 + T0/2;
		   break;
		 case 4:
		   *Ta = T0/2;
		   *Tb = T1+ T0/2;
		   *Tc = T1 + T2 + T0/2;
		   break;
		 case 5:
		   *Ta = T2 + T0/2;
		   *Tb = T0/2;
		   *Tc = T1 + T2 + T0/2;
		   break;
		 case 6:
		   *Ta = T1 + T2 + T0/2;
		   *Tb = T0/2;
		   *Tc = T1 + T0/2;
		   break;
		 default:
		  // possible error state
		   *Ta = 0;
		   *Tb = 0;
		   *Tc = 0;
	}

}
