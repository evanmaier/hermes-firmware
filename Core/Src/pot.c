#include "pot.h"
#include "stdio.h"

uint32_t Poll_ADC(ADC_HandleTypeDef hadc)
{
	uint32_t ADC_RES = 0;
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
	printf("voltage = %f", voltage);
}
