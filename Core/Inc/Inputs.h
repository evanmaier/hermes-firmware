/*
 * Inputs.h
 *
 *  Created on: Jul 1, 2023
 *      Author: amann
 */

#ifndef INC_INPUTS_H_
#define INC_INPUTS_H_

#include "main.h"
#define ADC_0V_VALUE                                             0
#define ADC_1V_VALUE                                             1241
#define ADC_2V_VALUE                                             2482
#define ADC_3V_VALUE                                             3723

#define SWITCH_START_PIN (Switch_Start_Pin)
#define SWITCH_START_PORT (Switch_Start_GPIO_Port)

#define SWITCH_BRAKE_PIN (Enable_EBrake_Pin)
#define SWITCH_BRAKE_PORT (Enable_EBrake_GPIO_Port)

#define NUMBER_ADC_CHANNEL (9)
#define NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL (3)
#define ADC_DMA_BUFF_SIZE (NUMBER_ADC_CHANNEL * NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL)

typedef enum {
	Phase_A_BackEMF = 0U,
	Phase_B_BackEMF,
	Phase_C_BackEMF,
	Phase_A_Current,
	Phase_B_Current,
	Phase_C_Current,
	Torque_Knob,
	Speed_Knob,
	Bus_Voltage,
	PERIPHERAL_SIZE
}ADC_Channels;



// Inefficent because it relies on the CPU to perform the conversion.
// Rebuild using the DMA Controller
uint16_t Poll_ADC(ADC_HandleTypeDef hadc1);
void Test_Poll_ADC();


uint16_t Test_Speed_Knob();
void Test_Torque_Knob();

#endif /* INC_INPUTS_H_ */
