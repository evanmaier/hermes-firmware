/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Control_Signal_1_Pin GPIO_PIN_3
#define Control_Signal_1_GPIO_Port GPIOE
#define Control_Signal_2_Pin GPIO_PIN_4
#define Control_Signal_2_GPIO_Port GPIOE
#define Phase_A_PWM_Low_Pin GPIO_PIN_5
#define Phase_A_PWM_Low_GPIO_Port GPIOE
#define Phase_A_PWM_High_Pin GPIO_PIN_6
#define Phase_A_PWM_High_GPIO_Port GPIOE
#define Control_Signal_3_Pin GPIO_PIN_13
#define Control_Signal_3_GPIO_Port GPIOC
#define Switch_Start_Pin GPIO_PIN_15
#define Switch_Start_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define Dissipative_Brake_Pin GPIO_PIN_0
#define Dissipative_Brake_GPIO_Port GPIOC
#define Enable_Motor_Power_Pin GPIO_PIN_1
#define Enable_Motor_Power_GPIO_Port GPIOC
#define Phase_A_Current_Pin GPIO_PIN_2
#define Phase_A_Current_GPIO_Port GPIOC
#define Bus_Voltage_Pin GPIO_PIN_3
#define Bus_Voltage_GPIO_Port GPIOC
#define Phase_B_PWM_High_Pin GPIO_PIN_0
#define Phase_B_PWM_High_GPIO_Port GPIOA
#define Phase_B_PWM_Low_Pin GPIO_PIN_1
#define Phase_B_PWM_Low_GPIO_Port GPIOA
#define Phase_C_PWM_High_Pin GPIO_PIN_2
#define Phase_C_PWM_High_GPIO_Port GPIOA
#define Phase_C_PWM_Low_Pin GPIO_PIN_3
#define Phase_C_PWM_Low_GPIO_Port GPIOA
#define Phase_B_Current_Pin GPIO_PIN_4
#define Phase_B_Current_GPIO_Port GPIOA
#define Torque_Control_Pin GPIO_PIN_5
#define Torque_Control_GPIO_Port GPIOA
#define Phase_A_Back_EMF_Pin GPIO_PIN_6
#define Phase_A_Back_EMF_GPIO_Port GPIOA
#define Speed_Control_Pin GPIO_PIN_7
#define Speed_Control_GPIO_Port GPIOA
#define Phase_C_Current_Pin GPIO_PIN_5
#define Phase_C_Current_GPIO_Port GPIOC
#define Phase_B_Back_EMF_Pin GPIO_PIN_0
#define Phase_B_Back_EMF_GPIO_Port GPIOB
#define Phase_C_Back_EMF_Pin GPIO_PIN_1
#define Phase_C_Back_EMF_GPIO_Port GPIOB
#define Buzzer_On_Pin GPIO_PIN_9
#define Buzzer_On_GPIO_Port GPIOE
#define Blue_LED_Pin GPIO_PIN_11
#define Blue_LED_GPIO_Port GPIOD
#define OnBoard_Green_LED_Pin GPIO_PIN_12
#define OnBoard_Green_LED_GPIO_Port GPIOD
#define Red_LED_Pin GPIO_PIN_13
#define Red_LED_GPIO_Port GPIOD
#define Green_LED_Pin GPIO_PIN_14
#define Green_LED_GPIO_Port GPIOD
#define Orange_LED_Pin GPIO_PIN_15
#define Orange_LED_GPIO_Port GPIOD
#define Enable_EBrake_Pin GPIO_PIN_1
#define Enable_EBrake_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
