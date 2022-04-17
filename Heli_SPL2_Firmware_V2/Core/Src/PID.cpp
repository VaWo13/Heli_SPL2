#include "PID.h"
#include "MPU6050.h"
#include "motorControl.h"
#include "SBUS.h"

#include "stm32f2xx_hal.h"
#include "main.h"

#include <math.h>


uint8_t tuningMode = 1;
uint8_t buttonPressed = false;

float PID_Pitch_xw_diff;
float PID_Roll_xw_diff;
float PID_Yaw_xw_diff;

float PID_Pitch_y;
float PID_Roll_y;
float PID_Yaw_y;

float Pitch_PID_k[3] = {0.0834, 0.0000, 0.0280};//{0.133, 0.0178, 0.1484};
float Roll_PID_k[3]  = {0.0765, 0.0088, 0.0448};//{0.0723, 0.0066, 0.0543};
float Yaw_PID_k[3]   = {0.3920, 0.1026, 2.7184};
float Pitch_I_Sum = 0;
float Roll_I_Sum  = 0;
float Yaw_I_Sum   = 0;            //NOTDONE unused? rename?
float Pitch_D_old = 0;
float Roll_D_old  = 0;
float Yaw_D_old   = 0;

float MPUoutputQuaternion[4];
float OriginQuaternion[4];
float OriginToOutputQuaternion[4];
float FrameOriginQuaternion[4] = {1, 0, 0, 0};
float LoopWQuaternion[4] = {1, 0, 0, 0};                  //NOTDONE unused variables? rename
float GyroOriginQuaternion[4] = {1, 1, 0, 0};
float LoopXWQuaternion[4];
float updateQuaternion[4];


/**
 * @brief This method gets the difference between X and W as a rotation in the
 * form of a Quaternion and then converts it to usable PITCH,ROLL,YAW angles
 * 
 * --Custom Method!
 */
void get_XW_diffAngles()
{
  //float *p = QuaternionSLERP(QuaternionProduct(QuaternionSLERP(&OriginQuaternion[0], &MPUoutputQuaternion[0]), &FrameOriginQuaternion[0]), &LoopWQuaternion[0]);
  //float *p = QuaternionNormalize(QuaternionSLERP(FrameOriginQuaternion, QuaternionNormalize(QuaternionProduct(QuaternionNormalize(QuaternionSLERP(QuaternionNormalize(QuaternionProduct(QuaternionNormalize(QuaternionSLERP(OriginQuaternion, MPUoutputQuaternion)), FrameOriginQuaternion)), FrameOriginQuaternion)), LoopWQuaternion))));
  float *p = QuaternionNormalize(QuaternionProduct(QuaternionNormalize(QuaternionSLERP(QuaternionNormalize(QuaternionProduct(QuaternionNormalize(QuaternionSLERP(GyroOriginQuaternion, QuaternionNormalize(QuaternionProduct(QuaternionNormalize(QuaternionSLERP(OriginQuaternion, GyroOriginQuaternion)), MPUoutputQuaternion)))), FrameOriginQuaternion)), FrameOriginQuaternion)), LoopWQuaternion));
  
  LoopXWQuaternion[0] = *p;
  LoopXWQuaternion[1] = *(p + 1);
  LoopXWQuaternion[2] = *(p + 2);
  LoopXWQuaternion[3] = *(p + 3);

  //difference x-w in degrees
  PID_Pitch_xw_diff = 2 * (((float)asin(LoopXWQuaternion[2]) * 180) / M_PI);
  PID_Roll_xw_diff  = 2 * (((float)asin(LoopXWQuaternion[1]) * 180) / M_PI);
  PID_Yaw_xw_diff   = 2 * (((float)atan(LoopXWQuaternion[3] / LoopXWQuaternion[0]) * 180) / M_PI);
}

/**
 * @brief 
 * 
 * //NOTDONE
 */
void reset_WQuaternion()
{
  float *p = QuaternionNormalize(QuaternionProduct(QuaternionNormalize(QuaternionSLERP(GyroOriginQuaternion, QuaternionNormalize(QuaternionProduct(QuaternionNormalize(QuaternionSLERP(OriginQuaternion, GyroOriginQuaternion)), MPUoutputQuaternion)))), FrameOriginQuaternion));
  LoopWQuaternion[0] = *p;
  LoopWQuaternion[1] = *(p + 1);
  LoopWQuaternion[2] = *(p + 2);
  LoopWQuaternion[3] = *(p + 3);
}

/**
 * @brief Gets PITCH,ROLL,YAW Y values and integrates/differentiates
 * 
 * --Custom Method!
 */
void update_PID()
{
  PID_Yaw_xw_diff = -PID_Yaw_xw_diff;
  Pitch_I_Sum += (PID_Pitch_xw_diff * Pitch_PID_k[1]);                                                                                  // integrate
  Roll_I_Sum  += (PID_Roll_xw_diff  * Roll_PID_k[1] );
  Yaw_I_Sum   += (PID_Yaw_xw_diff   * Yaw_PID_k[1]  );

  if (Pitch_I_Sum >  50) Pitch_I_Sum =  50;                         //constrain integration (-500 500)
  if (Roll_I_Sum  >  50) Roll_I_Sum  =  50;
  if (Yaw_I_Sum   > 200) Yaw_I_Sum   = 200;
  if (Pitch_I_Sum < -50) Pitch_I_Sum = -50;       //NOTDONE use defines for values
  if (Roll_I_Sum  < -50) Roll_I_Sum  = -50;
  if (Yaw_I_Sum   <   0) Yaw_I_Sum   =   0;

  PID_Pitch_y = (PID_Pitch_xw_diff * Pitch_PID_k[0] * 10) + Pitch_I_Sum + ((PID_Pitch_xw_diff - Pitch_D_old) * Pitch_PID_k[2] * 100);   //combine P,I,D values
  PID_Roll_y  = (PID_Roll_xw_diff  * Roll_PID_k[0]  * 10) + Roll_I_Sum  + ((PID_Roll_xw_diff  - Roll_D_old ) * Roll_PID_k[2]  * 100);
  PID_Yaw_y   = (PID_Yaw_xw_diff   * Yaw_PID_k[0]   * 20) + Yaw_I_Sum   + ((PID_Yaw_xw_diff   - Yaw_D_old  ) * Yaw_PID_k[2]   * 100);          //NOTDONE use defines for values

  Pitch_D_old = PID_Pitch_xw_diff;                                                                                                      // differentiate
  Roll_D_old  = PID_Roll_xw_diff ;
  Yaw_D_old   = PID_Yaw_xw_diff  ;

  if (PID_Pitch_y > 500 ) PID_Pitch_y = 500 ;
  if (PID_Roll_y  > 500 ) PID_Roll_y  = 500 ;                            //NOTDONE use defines for values
  if (PID_Yaw_y   > 1000) PID_Yaw_y   = 1000;
  if (PID_Pitch_y < -500) PID_Pitch_y = -500;
  if (PID_Roll_y  < -500) PID_Roll_y  = -500;
  if (PID_Yaw_y   <    0) PID_Yaw_y   =    0;
}

/**
 * @brief This method extracts the PID values for PITCH,ROLL,YAW that can be live tuned on the remote
 * 
 * --Custom Method!
 */
void getPIDValues()
{
  if (tuningMode == 1)
  {
    if ((SBUS_Channels[5] <= 1000) & (SBUS_Channels[5] >= 990))
    {
      if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) Pitch_PID_k[0] = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.08;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) Pitch_PID_k[1] = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.02;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <= -990) & (SBUS_Channels[6] >= -1000)) Pitch_PID_k[2] = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.08;    //NOTDONE use defines for values       
    }
    if ((SBUS_Channels[5] <= 10) & (SBUS_Channels[5] >= -10))
    {
      if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) Roll_PID_k[0]  = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.08;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) Roll_PID_k[1]  = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.02;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <= -990) & (SBUS_Channels[6] >= -1000)) Roll_PID_k[2]  = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.08;    //NOTDONE use defines for values       
    }
    if ((SBUS_Channels[5] <= -990) & (SBUS_Channels[5] >= -1000))
    {
      if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) Yaw_PID_k[0]   = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.8;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) Yaw_PID_k[1]   = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.1;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <= -990) & (SBUS_Channels[6] >= -1000)) Yaw_PID_k[2]   = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 3.2;    //NOTDONE use defines for values       
    }
  }
}

/**
 * @brief //NOTDONE
 * 
 * --Custom Method!
 */
void getAngleOffset()
{
  if (tuningMode == 2)
  {
    if ((SBUS_Channels[5] <= 1000) & (SBUS_Channels[5] >= 990))
    {
      //mainMotorMaxOffset = -139 + (((float)SBUS_Channels[7]) / 20);
    }
    if ((SBUS_Channels[5] <= 10) & (SBUS_Channels[5] >= -10))
    {
      mainMotorStartOffset = -53 + (((float)SBUS_Channels[7]) / 50);
    }
    if ((SBUS_Channels[5] <= -990) & (SBUS_Channels[5] >= -1000))
    {
      mainMotorSkewOffset = (float)SBUS_Channels[7] / 25;
      sin_SkewOffset = sin((mainMotorSkewOffset * M_PI) / 180);
      cos_SkewOffset = cos((mainMotorSkewOffset * M_PI) / 180);
    }

  }
}

/**
 * @brief //NOTDONE
 * 
 * --Custom Method!
 */
void switchTuningMode()
{
  if ((buttonPressed == false) & (HAL_GPIO_ReadPin(ONBOARD_BUTTON_2_GPIO_Port, ONBOARD_BUTTON_2_Pin) == true))
  {
    if (tuningMode == 1) tuningMode = 2;
    else tuningMode = 1;
    HAL_GPIO_TogglePin(ONBOARD_LED_2_GPIO_Port, ONBOARD_LED_2_Pin);
    buttonPressed = true;
  }
  if ((buttonPressed == true) & (HAL_GPIO_ReadPin(ONBOARD_BUTTON_2_GPIO_Port, ONBOARD_BUTTON_2_Pin) == false))
  {
    buttonPressed = false;
  }
}