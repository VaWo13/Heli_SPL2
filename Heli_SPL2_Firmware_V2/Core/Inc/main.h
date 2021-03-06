/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

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

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern void MX_I2C1_Init(void);
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ONBOARD_READ_3_Pin GPIO_PIN_0
#define ONBOARD_READ_3_GPIO_Port GPIOC
#define ONBOARD_ADC_2_Pin GPIO_PIN_1
#define ONBOARD_ADC_2_GPIO_Port GPIOC
#define ONBOARD_ADC_1_Pin GPIO_PIN_2
#define ONBOARD_ADC_1_GPIO_Port GPIOC
#define ONBOARD_BUTTON_1_Pin GPIO_PIN_0
#define ONBOARD_BUTTON_1_GPIO_Port GPIOA
#define ONBOARD_BUTTON_2_Pin GPIO_PIN_1
#define ONBOARD_BUTTON_2_GPIO_Port GPIOA
#define ONBOARD_BUTTON_3_Pin GPIO_PIN_2
#define ONBOARD_BUTTON_3_GPIO_Port GPIOA
#define ONBOARD_BUTTON_4_Pin GPIO_PIN_3
#define ONBOARD_BUTTON_4_GPIO_Port GPIOA
#define ONBOARD_LED_1_Pin GPIO_PIN_4
#define ONBOARD_LED_1_GPIO_Port GPIOA
#define ONBOARD_LED_2_Pin GPIO_PIN_5
#define ONBOARD_LED_2_GPIO_Port GPIOA
#define ONBOARD_LED_3_Pin GPIO_PIN_6
#define ONBOARD_LED_3_GPIO_Port GPIOA
#define ONBOARD_LED_4_Pin GPIO_PIN_7
#define ONBOARD_LED_4_GPIO_Port GPIOA
#define ONBOARD_WRITE_4_Pin GPIO_PIN_6
#define ONBOARD_WRITE_4_GPIO_Port GPIOC
#define ONBOARD_WRITE_3_Pin GPIO_PIN_8
#define ONBOARD_WRITE_3_GPIO_Port GPIOA
#define ONBOARD_WRITE_2_Pin GPIO_PIN_10
#define ONBOARD_WRITE_2_GPIO_Port GPIOC
#define ONBOARD_WRITE_1_Pin GPIO_PIN_6
#define ONBOARD_WRITE_1_GPIO_Port GPIOB
#define ONBOARD_READ_4_Pin GPIO_PIN_7
#define ONBOARD_READ_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
