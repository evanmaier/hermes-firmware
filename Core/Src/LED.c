/*
 * LED.c
 *
 *  Created on: Jul 16, 2023
 *      Author: amann
 */


#include "LED.h"


void LED_On(uint16_t led, GPIO_TypeDef *Port){
	HAL_GPIO_WritePin(Port, led, GPIO_PIN_SET);
}

void LED_Off(uint16_t led, GPIO_TypeDef *Port){
	HAL_GPIO_WritePin(Port, led, GPIO_PIN_RESET);
}

void LED_Toggle(uint16_t led, GPIO_TypeDef *Port){
	HAL_GPIO_TogglePin(Port, led);
}

// Turn on all the LEDs
void Test_LEDs_On(){
	LED_On(LED_ERROR_PIN, LED_ERROR_PORT);
	LED_On(LED_SYS_POWER_PIN, LED_SYS_POWER_PORT);
	LED_On(LED_START_PIN, LED_START_PORT);
	LED_On(LED_BRAKE_PIN, LED_BRAKE_PORT);
	LED_On(LED_ONBOARD_PIN, LED_ONBOARD_PORT);
}

