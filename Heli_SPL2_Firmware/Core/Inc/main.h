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

#define true                              1
#define false                             0

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim13;

extern uint32_t PinInterruptLastTime;

extern uint16_t fastPPM_ONTime;//ON time in microseconds
extern uint16_t fastPPM_OFFTime;//OFF time in microseconds
extern uint8_t fastPPM_powered;

extern uint16_t slowPPM1_ONTime;//ON time in microseconds
extern uint16_t slowPPM1_OFFTime;//OFF time in microseconds
extern uint8_t slowPPM1_powered;

extern void MX_I2C1_Init(void);

extern void ADC_Select_Channel_11();
extern void ADC_Select_Channel_12();


//#include "MPU6050.h"

//extern MPU6050 mpu;

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
#define ONBOARD_READ_IT_3_Pin GPIO_PIN_0
#define ONBOARD_READ_IT_3_GPIO_Port GPIOC
#define ONBOARD_READ_IT_3_EXTI_IRQn EXTI0_IRQn
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
