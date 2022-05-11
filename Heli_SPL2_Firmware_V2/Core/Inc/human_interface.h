#include <string.h>
#include <stdio.h>
#include "main.h"

#define LED_ToggleTime_MPU_initFail 125
#define LED_ToggleTime_SBUS         250
#define LED_ToggleTime_ESC1_cal     500
#define LED_ToggleTime_ESC2_cal     500
#define LED_ToggleTime_THR0         500
#define LED_ToggleTime_Gyro_cal     250
#define LED_ToggleTime_Gyro_calFail 250
#define LED_ToggleTime_Ready        125

void LED_SetAll();
void LED_ResetAll();
void LED_ToggleAll();
void LED_status_MPU_init();
void LED_status_MPU_initFail();
void LED_status_WaitingForSBUS();
void LED_status_ESC1_Cal();
void LED_status_ESC2_Cal();
void LED_status_WaitingForThrottle0();
void LED_status_Gyro_cal();
void LED_status_Gyro_calFail();
void LED_status_Ready();