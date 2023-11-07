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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CURRENT_ADC_3_Pin GPIO_PIN_0
#define CURRENT_ADC_3_GPIO_Port GPIOA
#define CURRENT_ADC_2_Pin GPIO_PIN_1
#define CURRENT_ADC_2_GPIO_Port GPIOA
#define CURRENT_ADC_1_Pin GPIO_PIN_2
#define CURRENT_ADC_1_GPIO_Port GPIOA
#define V_BAT_ADC_Pin GPIO_PIN_5
#define V_BAT_ADC_GPIO_Port GPIOA
#define V_AUX_ADC_Pin GPIO_PIN_6
#define V_AUX_ADC_GPIO_Port GPIOA
#define ADC2_NTC1_Pin GPIO_PIN_2
#define ADC2_NTC1_GPIO_Port GPIOB
#define ADC2_NTC2_Pin GPIO_PIN_11
#define ADC2_NTC2_GPIO_Port GPIOB
#define RUNNING_LED_Pin GPIO_PIN_6
#define RUNNING_LED_GPIO_Port GPIOC
#define WARNING_LED_Pin GPIO_PIN_7
#define WARNING_LED_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_9
#define ERROR_LED_GPIO_Port GPIOA
#define ENCODER2_CS_Pin GPIO_PIN_2
#define ENCODER2_CS_GPIO_Port GPIOD
#define ENCODER1_CS_Pin GPIO_PIN_6
#define ENCODER1_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
