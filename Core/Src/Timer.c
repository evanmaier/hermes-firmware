
#include "Timer.h"

void Buzzer_DutyCycle(){
	TIM1->CCR1 = BUZZER_CRR;
}

void Buzzer_Start(TIM_HandleTypeDef* htim1){
	HAL_TIM_PWM_Start(htim1, BUZZER_TIMER_CHANNEL);
}

void Buzzer_Stop(TIM_HandleTypeDef* htim1){
	HAL_TIM_PWM_Stop(htim1, BUZZER_TIMER_CHANNEL);
}

void Buzzer_Routine(TIM_HandleTypeDef* htim1){
	Buzzer_DutyCycle();
	Buzzer_Start(htim1);
	HAL_Delay(2000);
	Buzzer_Stop(htim1);
}


