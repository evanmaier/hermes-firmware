/*
 * Inputs.c
 *
 *  Created on: Jul 1, 2023
 *      Author: amann
 */

#include "inputs.h"

uint16_t Poll_ADC(ADC_HandleTypeDef hadc)
{
	uint16_t ADC_RES = 0;
	// Start ADC Conversion
	HAL_ADC_Start(&hadc);
	// Poll ADC1 Peripheral & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc, 1);
	// Read The ADC Conversion Result
	ADC_RES = HAL_ADC_GetValue(&hadc);
	// Return result
	return ADC_RES;
}
void Test_Pot(ADC_HandleTypeDef hadc)
{
	float voltage = 0;
	voltage = ( (float)Poll_ADC(hadc)  / 4092.0) * 3.0;
//	printf("voltage = %f", voltage);
}

uint16_t Test_Speed_Knob(ADC_HandleTypeDef* hadc){
	// Start ADC Conversion
	// Pass (The ADC Instance, Result Buffer Address, Buffer Length)
	uint32_t buffer[PERIPHERAL_SIZE];
	HAL_ADC_Start_DMA(hadc, (uint32_t*)&buffer, PERIPHERAL_SIZE);
	HAL_Delay(10);
	return buffer[3];
//	uint32_t res = ADC_GET_RESOLUTION(hadc);
}

