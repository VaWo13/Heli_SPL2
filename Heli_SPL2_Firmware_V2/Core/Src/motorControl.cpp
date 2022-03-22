#include "main.h"
#include "SBUS.h"
#include "motorControl.h"
#include "PID.h"

#include "usbd_cdc_if.h"
#include <math.h>

uint16_t adcValueChannel11;
uint16_t adcValueChannel12;
uint16_t mainMotorAngle;
int8_t mainMotorAngleOffset;

/**
 * @brief This method updates the main motor speed by first measuring both HALL sensors and determining the motor angle.
 * Then it combines the throttle input and the PITCH,ROLL Y values together with the angle to get the new motorspeed
 * 
 * --Custom Method!
 */
void updateMainMotorSpeed()
{
  ADC1->SMPR1 &= ~ADC_SMPR1(ADC_SMPR1_SMP10, ADC_CHANNEL_11);               //reset sample rate
  ADC1->SMPR1 |= ADC_SMPR1(ADC_SAMPLETIME_15CYCLES, ADC_CHANNEL_11);        //set new sample rate Channel 11
  ADC1->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, 1);                              //reset Rank
  ADC1->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_11, 1);                             //set new Rank Channel 11
  HAL_ADC_Start(&hadc1);                                                    //start ADC
	adcValueChannel11 = (uint16_t)ADC1->DR;                                   //read ADC value

	ADC1->SMPR1 &= ~ADC_SMPR1(ADC_SMPR1_SMP10, ADC_CHANNEL_12);               //reset sample rate
  ADC1->SMPR1 |= ADC_SMPR1(ADC_SAMPLETIME_15CYCLES, ADC_CHANNEL_12);        //set new sample rate Channel 12
  ADC1->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, 1);                              //reset Rank
  ADC1->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_12, 1);                             //set new Rank Channel 12
  HAL_ADC_Start(&hadc1);                                                    //start ADC
	adcValueChannel12 = (uint16_t)ADC1->DR;                                   //read ADC value

  //NOTDONE maybe apply offset rotation without converting to mainMotorAngle and then back but rather use complex numbers
  //NOTDONE disable PITCH,ROLL when throttle is 0
  //mainMotorAngle = motorAngle(adcValueChannel12 - 1250, adcValueChannel11 - 1250);
  mainMotorAngle = ((atan2((float)adcValueChannel12 - 1250, (float)adcValueChannel11 - 1250) * 180) / M_PI) + 180;       //NOTDONE use define for value fastPPM_CenterTime
  TIM4->CCR1 = (uint16_t)(fastPPM_MinTime + 500 + ((float)SBUS_Channels[2] / 2) + ((float)sin((mainMotorAngle + mainMotorAngleOffset) * (M_PI / 180)) * (PID_Pitch_y / 10)) + ((float)cos((mainMotorAngle + mainMotorAngleOffset) * (M_PI / 180)) * ((float)PID_Roll_y / 10))); //NOTDONE use define for value, try to simplify
}

/**
 * @brief This method updates the tail motor speed.
 * If the throttle is off the motor is disabled.
 * If the throttle is above a given deadzone the motor speed is YAW Y value
 * 
 * --Custom Method!
 */
void updateTailMotorSpeed()
{
  if (SBUS_Channels[2] > tailmotorDeadzone)
  {
    TIM3->CCR1 = (uint16_t)(slowPPM1_MinTime + PID_Yaw_y);
  }
  else
  {
    TIM3->CCR1 = slowPPM1_MinTime;
  }
}

/**
 * @brief Initializes TIM 3 and 4 used for generating the PPM signals
 * 
 * --Custom Method!
 */
void PPM_init()
{
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
}

/**
 * @brief When BT1 on the board is pressed while powering on, the programm enters an ESC calibration mode.
 * In this mode the raw remote throttle value is first given to the tail ESC, and upon pressing BT1 again
 * it is given to the main motor ESC. To exit BT1 must be pressed a second time.
 * If BT1 is not pressed, it gives the ESCs x time to initialize
 * 
 * --Custom Method!
 */
void ESCCalibration()
{
  if (HAL_GPIO_ReadPin(ONBOARD_BUTTON_1_GPIO_Port, ONBOARD_BUTTON_1_Pin) == true)
  {
    uint16_t counter = 0;
    HAL_GPIO_TogglePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin);   //NOTDONE debug
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);                                                                             //start PPM for tail motor
    uint16_t oldTime = TIM11->CNT;

    //main-motor
    while ((HAL_GPIO_ReadPin(ONBOARD_BUTTON_1_GPIO_Port, ONBOARD_BUTTON_1_Pin) == false) | (counter < BT_debounceTime))   //wait for BT1 to be pressed after BT_debounceTime has passed
    {
      if (SBUSNewPackage == true) SBUS_postProcessing();                                                                  //process new packets
      TIM3->CCR1 = (uint16_t)(slowPPM1_CenterTime + (SBUS_Channels[2] * PPMtailMotorScaler));                             //update motor speed
      while ((TIM11->CNT - oldTime) < 10000);                                                                             //wait for 10 milliseconds to pass
      oldTime = TIM11->CNT;
      counter += 10;
    }

    counter = 0;                                                                                                          //reset counter
    HAL_GPIO_TogglePin(ONBOARD_LED_3_GPIO_Port, ONBOARD_LED_3_Pin);   //NOTDONE debug
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);                                                                             //start PPM for main motor
    oldTime = TIM11->CNT;

    //tail-motor
    while ((HAL_GPIO_ReadPin(ONBOARD_BUTTON_1_GPIO_Port, ONBOARD_BUTTON_1_Pin) == false) | (counter < BT_debounceTime))   //wait for BT1 to be pressed after BT_debounceTime has passed
    {
      if (SBUSNewPackage == true) SBUS_postProcessing();                                                                  //process new packets
      TIM4->CCR1 = (uint16_t)(fastPPM_CenterTime + (SBUS_Channels[2] * PPMmainMotorScaler));                              //update motor speed
      while ((TIM11->CNT - oldTime) < 10000);                                                                             //wait for 10 milliseconds to pass
      oldTime = TIM11->CNT;
      counter += 10;
    }
  }
  else
  {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);                                                                             //start PPM for tail motor
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);                                                                             //start PPM for main motor

    TIM3->CCR1 = slowPPM1_MinTime;                                                                                        //set to 0%
    TIM4->CCR1 = fastPPM_MinTime;                                                                                         //set to 0%
    HAL_Delay(ESC_StartupDelay);
  }
}