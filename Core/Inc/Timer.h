#ifndef _TIMER_H
#define _TIMER_H

#include "main.h"
#include "transforms.h"


#define PI (M_PI)


#define BUZZER_TIMER_CHANNEL 	(TIM_CHANNEL_1)
#define BUZZER_ARR				(168-1) // Pre-scalar (1MHz)
#define BUZZER_CRR				(50) // Counter Limit (x% Duty Cycle)

void Buzzer_Start(TIM_HandleTypeDef* htim1);
void Buzzer_Stop(TIM_HandleTypeDef* htim1);
void Buzzer_Routine(TIM_HandleTypeDef* htim1);


/* SVPWM Stuff */
typedef struct {

} sSVPWM;



#endif
