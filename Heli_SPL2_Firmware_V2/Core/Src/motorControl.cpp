#include "main.h"
#include "MPU6050.h"
#include "SBUS.h"
#include "motorControl.h"
#include "PID.h"

#include "usbd_cdc_if.h"
#include <math.h>

uint16_t adcValueChannel11;
uint16_t adc_TimestampMicros;
uint16_t old_adc_TimestampMicros;
uint32_t adc_TimestampMillis;
uint32_t old_adc_TimestampMillis;
uint16_t old_adcValueChannel11;
uint16_t adcValueChannel12;
uint32_t MainMotor_CTS_Millis;
uint32_t old_MainMotor_CTS_Millis;
uint16_t MainMotor_CTS_Micros;
uint16_t old_MainMotor_CTS_Micros;
float mainMotorPeriod = 1000000;                             //NOTDONE
float mainMotorPeriod_DLPF;                             
uint16_t mainMotorAngle;
float mainMotorStartOffset = -135.25;
float mainMotorMaxOffset = -1.518;
float mainMotorSkewOffset = 0;
float smoothMainMotorSpeed;
float mainMotorCyclicAccel;
float mainMotorCyclicSpeed;
float sin_OffsetAngle = sin(((float)mainMotorStartOffset * M_PI) / 180);
float cos_OffsetAngle = cos(((float)mainMotorStartOffset * M_PI) / 180);
float sin_SkewOffset = sin((mainMotorSkewOffset * M_PI) / 180);
float cos_SkewOffset = cos((mainMotorSkewOffset * M_PI) / 180);


/**
 * @brief This method updates the main motor speed by first measuring both HALL sensors and determining the motor position.
 * Then it combines the throttle input and the PITCH,ROLL Y values together with the position and some offsets to get the
 * new motorspeed.  //NOTDONE
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
  HAL_ADC_PollForConversion(&hadc1, 100);                                   //wait for ADC to finish
	adcValueChannel11 = (uint16_t)ADC1->DR;                                   //read ADC value

	ADC1->SMPR1 &= ~ADC_SMPR1(ADC_SMPR1_SMP10, ADC_CHANNEL_12);               //reset sample rate
  ADC1->SMPR1 |= ADC_SMPR1(ADC_SAMPLETIME_15CYCLES, ADC_CHANNEL_12);        //set new sample rate Channel 12
  ADC1->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, 1);                              //reset Rank
  ADC1->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_12, 1);                             //set new Rank Channel 12
  HAL_ADC_Start(&hadc1);                                                    //start ADC
  HAL_ADC_PollForConversion(&hadc1, 100);                                   //wait for ADC to finish
	adcValueChannel12 = (uint16_t)ADC1->DR;                                   //read ADC value

  adc_TimestampMicros = TIM11->CNT;
  adc_TimestampMillis = HAL_GetTick();

  if (smoothMainMotorSpeed > motorDeadzone)                                 //if throttle is not off
  {
    //calculate motorspeed based on throttle, motor position, motorspeed, start-offset and skew-offset//NOTDONE

    mainMotorCyclicAccel =                                                          /* get motor acceleration ---> rotor blade-pitch angle*/                                                                                                                                                                                                                                                                       \
    + ((((((float)adcValueChannel12 - hall2_center) * hall2_scaler) * cos_OffsetAngle) - ((((float)adcValueChannel11 - hall1_center) * hall1_scaler) * sin_OffsetAngle)) * (Pitch_output * 0.01 * (smoothMainMotorSpeed + SBUS_mappedValueMax)))  \
    + ((((((float)adcValueChannel11 - hall1_center) * hall1_scaler) * cos_OffsetAngle) + ((((float)adcValueChannel12 - hall2_center) * hall2_scaler) * sin_OffsetAngle)) * (Roll_output  * 0.01 * (smoothMainMotorSpeed + SBUS_mappedValueMax)));
    
    mainMotorCyclicSpeed += mainMotorCyclicAccel;                                   //integrate

    mainMotorCyclicSpeed -= mainMotorCyclicSpeed * mainMotorCyclicSpeed_leaky_I_k;  //leak towards 0

    TIM4->CCR1 = (uint16_t)(                        \
      fastPPM_CenterTime                            \
    + (smoothMainMotorSpeed * PPMmainMotorScaler)   \
    + mainMotorCyclicSpeed);
    

    if (TIM4->CCR1 > fastPPM_MaxTime) TIM4->CCR1 = fastPPM_MaxTime;                 //limit to PPM range
    if (TIM4->CCR1 < fastPPM_MinTime) TIM4->CCR1 = fastPPM_MinTime;                 //limit to PPM range
  }
  else
  {
    TIM4->CCR1 = fastPPM_MinTime;                                                   //motor off
  }
}

/**
 * @brief Smooths the throttle input by a specified factor
 * 
 * --Custom Method!
 */
void MainMotorDLPF()
{
  smoothMainMotorSpeed += MainMotorSmoothingFactor * ((float)SBUS_Channels[2] - smoothMainMotorSpeed);
}

/**
 * @brief This method updates the tail motor speed.
 * If the throttle is off the motor is disabled and PID sums and orientation get reset.
 * If the throttle is above a given deadzone the motor speed is YAW Y value
 * 
 * --Custom Method!
 */
void updateTailMotorSpeed()
{
  if (SBUS_Channels[2] > motorDeadzone)
  {
    TIM3->CCR1 = (uint16_t)(slowPPM1_MinTime + PID_Yaw_y);
    if (TIM3->CCR1 > slowPPM1_MaxTime) TIM3->CCR1 = slowPPM1_MaxTime;                 //limit to PPM range    
    if (TIM3->CCR1 < slowPPM1_MinTime) TIM3->CCR1 = slowPPM1_MinTime;                 //limit to PPM range
  }
  else
  {
    TIM3->CCR1 = slowPPM1_MinTime;
    Pitch_I_Sum = 0;
    Roll_I_Sum  = 0;
    Yaw_I_Sum   = 0;
    reset_WQuaternion();
  }
}

/**
 * @brief gets the current motor speed    //NOTDONE
 * 
 * --Custom Method!
 */
void getMainMotorSpeed()
{
  if ((old_adcValueChannel11 <= hall1_center) & (adcValueChannel11 >= hall1_center))
  {
    if ((adc_TimestampMillis - old_MainMotor_CTS_Millis) >= 60)
    {
      MainMotor_CTS_Millis = old_adc_TimestampMillis + ((float)(adc_TimestampMillis - old_adc_TimestampMillis) * ((hall1_center - old_adcValueChannel11) / (float)(adcValueChannel11 - old_adcValueChannel11)));
      mainMotorPeriod = (MainMotor_CTS_Millis - old_MainMotor_CTS_Millis) * 1000;
    }
    else
    {
      MainMotor_CTS_Micros = old_adc_TimestampMicros + (uint16_t)((float)((uint16_t)adc_TimestampMicros - (uint16_t)old_adc_TimestampMicros) * ((hall1_center - old_adcValueChannel11) / (float)(adcValueChannel11 - old_adcValueChannel11)));
      mainMotorPeriod = (uint16_t)((uint16_t)MainMotor_CTS_Micros - (uint16_t)old_MainMotor_CTS_Micros);
    }
      old_MainMotor_CTS_Millis += (uint32_t)(mainMotorPeriod * 0.001);
      old_MainMotor_CTS_Micros += (uint16_t)mainMotorPeriod;
  }
  mainMotorPeriod_DLPF += periodSmoothingFactor * (mainMotorPeriod - mainMotorPeriod_DLPF);   //smoothing

  old_adc_TimestampMillis = adc_TimestampMillis;
  old_adc_TimestampMicros = adc_TimestampMicros;
  old_adcValueChannel11 = adcValueChannel11;
}

/**
 * @brief Gets the //NOTDONE
 * 
 * --Custom Method!
 */
void getMainMotorOffset()
{
  float currentAngleOffset = mainMotorStartOffset + ((1 / (mainMotorPeriod_DLPF * 0.000001)) * (mainMotorMaxOffset));
  sin_OffsetAngle = sin((currentAngleOffset * M_PI) / 180);
  cos_OffsetAngle = cos((currentAngleOffset * M_PI) / 180);
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
  if (HAL_GPIO_ReadPin(ONBOARD_BUTTON_4_GPIO_Port, ONBOARD_BUTTON_4_Pin) == true)
  {
    LED_ResetAll();
    uint16_t counter = 0;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);                                                                             //start PPM for tail motor
    uint16_t oldTime = TIM11->CNT;

    //main-motor
    while ((HAL_GPIO_ReadPin(ONBOARD_BUTTON_4_GPIO_Port, ONBOARD_BUTTON_4_Pin) == false) | (counter < BT_debounceTime))   //wait for BT1 to be pressed after BT_debounceTime has passed
    {
      if (SBUSNewPackage == true) SBUS_postProcessing();                                                                  //process new packets
      TIM3->CCR1 = (uint16_t)(slowPPM1_CenterTime + (SBUS_Channels[2] * PPMtailMotorScaler));                             //update motor speed
      LED_status_ESC1_Cal();

      while ((TIM11->CNT - oldTime) < 10000);                                                                             //wait for 10 milliseconds to pass
      oldTime = TIM11->CNT;
      counter += 10;
    }

    LED_ResetAll();
    counter = 0;                                                                                                          //reset counter
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);                                                                             //start PPM for main motor
    oldTime = TIM11->CNT;

    //tail-motor
    while ((HAL_GPIO_ReadPin(ONBOARD_BUTTON_4_GPIO_Port, ONBOARD_BUTTON_4_Pin) == false) | (counter < BT_debounceTime))   //wait for BT1 to be pressed after BT_debounceTime has passed
    {
      if (SBUSNewPackage == true) SBUS_postProcessing();                                                                  //process new packets
      TIM4->CCR1 = (uint16_t)(fastPPM_CenterTime + (SBUS_Channels[2] * PPMmainMotorScaler));                              //update motor speed
      LED_status_ESC2_Cal();

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
  }
}