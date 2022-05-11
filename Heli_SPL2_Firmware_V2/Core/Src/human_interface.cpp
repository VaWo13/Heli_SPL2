#include "human_interface.h"

uint32_t LED_TS_MPU_initFail = 0;
uint32_t LED_TS_SBUS         = 0;
uint32_t LED_TS_ESC1_cal     = 0;
uint32_t LED_TS_ESC2_cal     = 0;
uint32_t LED_TS_THR0         = 0;
uint32_t LED_TS_Gyro_cal     = 0;

uint8_t LED_SBUS_Frame       = 1;
uint8_t LED_THR0_Frame       = 1;
uint8_t LED_Gyro_cal_Frame   = 1;


void LED_SetAll()
{
  HAL_GPIO_WritePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin, GPIO_PIN_SET);
}

void LED_ResetAll()
{
  HAL_GPIO_WritePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin, GPIO_PIN_RESET);
}

void LED_ToggleAll()
{
  HAL_GPIO_TogglePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin);
  HAL_GPIO_TogglePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin);
  HAL_GPIO_TogglePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin);
  HAL_GPIO_TogglePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin);
}

void LED_status_MPU_init()
{
  LED_SetAll();
}

void LED_status_MPU_initFail()
{
  if ((HAL_GetTick() - LED_TS_MPU_initFail) > LED_ToggleTime_MPU_initFail)
  {
    LED_TS_MPU_initFail = HAL_GetTick();
    LED_ToggleAll();
  }
}

void LED_status_WaitingForSBUS()
{
  if ((HAL_GetTick() - LED_TS_SBUS) > LED_ToggleTime_SBUS)
  {
    LED_TS_SBUS = HAL_GetTick();
    switch (LED_SBUS_Frame)
    {
    case 1:
      HAL_GPIO_WritePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin, GPIO_PIN_SET);
      break;
    case 2:
      HAL_GPIO_WritePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_SET);
      break;
    case 3:
      HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_SET);
      break;
    case 4:
      HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin, GPIO_PIN_SET);
      LED_SBUS_Frame = 0;
      break;
    default:
      break;
    }
    LED_SBUS_Frame ++;
  }
}

void LED_status_ESC1_Cal()
{
  if ((HAL_GetTick() - LED_TS_ESC1_cal) > LED_ToggleTime_ESC1_cal)
  {
    LED_TS_ESC1_cal = HAL_GetTick();
    HAL_GPIO_TogglePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin);
  }
}

void LED_status_ESC2_Cal()
{
  if ((HAL_GetTick() - LED_TS_ESC2_cal) > LED_ToggleTime_ESC2_cal)
  {
    LED_TS_ESC2_cal = HAL_GetTick();
    HAL_GPIO_TogglePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin);
  }
}

void LED_status_WaitingForThrottle0()
{
  if ((HAL_GetTick() - LED_TS_THR0) > LED_ToggleTime_THR0)
  {
    LED_TS_THR0 = HAL_GetTick();
    switch (LED_THR0_Frame)
    {
    case 1:
      HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_SET);
      break;
    case 2:
      HAL_GPIO_WritePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin, GPIO_PIN_SET);
      LED_THR0_Frame = 0;
      break;
    default:
      break;
    }
    LED_THR0_Frame ++;
  }
}

void LED_status_Gyro_cal()
{
  if ((HAL_GetTick() - LED_TS_Gyro_cal) > LED_ToggleTime_Gyro_cal)
  {
    LED_TS_Gyro_cal = HAL_GetTick();
    switch (LED_Gyro_cal_Frame)
    {
    case 1:
      HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin, GPIO_PIN_SET);
      break;
    case 2:
      HAL_GPIO_WritePin(ONBOARD_LED_1_GPIO_Port, ONBOARD_LED_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_SET);
      break;
    case 3:
      HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_SET);
      break;
    case 4:
      HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin, GPIO_PIN_SET);
      break;
    case 5:
      HAL_GPIO_WritePin(ONBOARD_LED_4_GPIO_Port, ONBOARD_LED_4_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_SET);
      break;
    case 6:
      HAL_GPIO_WritePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin, GPIO_PIN_SET);
      LED_Gyro_cal_Frame = 0;
      break;
    default:
      break;
    }
    LED_Gyro_cal_Frame ++;
  }
}
void LED_status_Gyro_calFail()
{
  LED_SetAll();
  HAL_Delay(LED_ToggleTime_Gyro_calFail);
  LED_ResetAll();
  HAL_Delay(LED_ToggleTime_Gyro_calFail);
  LED_SetAll();
  HAL_Delay(LED_ToggleTime_Gyro_calFail);
  LED_ResetAll();
  HAL_Delay(LED_ToggleTime_Gyro_calFail);
}

void LED_status_Ready()
{
  LED_SetAll();
  HAL_Delay(LED_ToggleTime_Ready);
  LED_ResetAll();
  HAL_Delay(LED_ToggleTime_Ready);
  LED_SetAll();
  HAL_Delay(LED_ToggleTime_Ready);
  LED_ResetAll();
  HAL_Delay(LED_ToggleTime_Ready);
  LED_SetAll();
  HAL_Delay(LED_ToggleTime_Ready);
  LED_ResetAll();
  HAL_Delay(LED_ToggleTime_Ready);
}
