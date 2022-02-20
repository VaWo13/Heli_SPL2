/*
 * SBUS.cpp
 *
 *  Created on: 20.02.2022
 *      Author: valew
 */

#include "SBUS.h"

#include "stm32f2xx_hal.h"
#include "main.h"

#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint16_t SBUS_timerCount = 0;
uint8_t SBUS_RxBitString[numberOfBits];

void read_SBUS()
{
    EXTI->IMR &= ~(EXTI_LINE_0);
    SBUS_timerCount = TIM11->CNT + SBUS_StartTimeOffset;       //get current clock count register value + time offset
    for (size_t i = 0; i < numberOfBits; i++)
    {
        while (TIM11->CNT - SBUS_ClockCyclesPerBit < SBUS_ClockCyclesPerBit)
        {
        }
        SBUS_timerCount = TIM11->CNT;
        SBUS_RxBitString[i] = ((ONBOARD_READ_IT_3_GPIO_Port->IDR & ONBOARD_READ_IT_3_Pin) != 0 ? true : false);     //if the pin is HIGH then the value is 1 else 0
        HAL_GPIO_TogglePin(ONBOARD_WRITE_4_GPIO_Port, ONBOARD_WRITE_4_Pin);     //debug pin
    }
    HAL_GPIO_TogglePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin);     //debug pin
}