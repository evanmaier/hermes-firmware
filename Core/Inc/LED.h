/*
 * LED.h
 *
 *  Created on: Jul 16, 2023
 *      Author: amann
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

#define LED_ERROR_PIN (Red_LED_Pin)
#define LED_SYS_POWER_PIN (Green_LED_Pin)
#define LED_START_PIN (Orange_LED_Pin)
#define LED_BRAKE_PIN (Blue_LED_Pin)
#define LED_ONBOARD_PIN (OnBoard_Green_LED_Pin)

#define LED_ERROR_PORT (Red_LED_GPIO_Port)
#define LED_SYS_POWER_PORT (Green_LED_GPIO_Port)
#define LED_START_PORT (Orange_LED_GPIO_Port)
#define LED_BRAKE_PORT (Blue_LED_GPIO_Port)
#define LED_ONBOARD_PORT (OnBoard_Green_LED_GPIO_Port)

extern void LED_On(uint16_t led, GPIO_TypeDef * Port);

extern void LED_Off(uint16_t led, GPIO_TypeDef * Port);

extern void LED_Toggle(uint16_t led, GPIO_TypeDef *Port);

extern void Test_LEDs_On();

extern void Config_LED();


#endif /* INC_LED_H_ */
