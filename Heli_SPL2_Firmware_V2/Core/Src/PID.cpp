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

float PID_Pitch_DLPF_xw_diff;
float PID_Roll_DLPF_xw_diff;
float PID_Yaw_DLPF_xw_diff;

float PID_Pitch_y;
float PID_Roll_y;
float PID_Yaw_y;

float Pitch_output;
float Roll_output;

float outputStrength = 4;

float Pitch_PID_k[3] = {0.0000, 0.0000,  0.0000};
float Roll_PID_k[3]  = {0.0000, 0.0000,  0.0000};
float Yaw_PID_k[3]   = {0.4544, 0.1026, 12.8000};
float Roll_DCPL_k  = 0;
float Pitch_DCPL_k = 0;           //NOTDONE unused? rename
float DCPL_angle;
float Pitch_y_maxChangeRate = 10;
float Roll_y_maxChangeRate  = 10;
float Pitch_y_ChangeRate;
float Roll_y_ChangeRate;
float PID_Pitch_y_DLPF;
float PID_Roll_y_DLPF;

float Pitch_D_DLPF;
float Roll_D_DLPF;
float Yaw_D_DLPF;
float PID_Pitch_y_DCPL;
float PID_Roll_y_DCPL;
float Pitch_DCPL_angle;
float Roll_DCPL_angle;
float PID_Pitch_y_DLPF_old;
float PID_Roll_y_DLPF_old;
float Pitch_I_Sum;
float Roll_I_Sum ;
float Yaw_I_Sum  ;            //NOTDONE unused? rename?
float Pitch_D_old;
float Roll_D_old ;
float Yaw_D_old  ;

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

  //DLPF of x-w
  PID_Pitch_DLPF_xw_diff += xwSmoothingFactor * (PID_Pitch_xw_diff - PID_Pitch_DLPF_xw_diff);
  PID_Roll_DLPF_xw_diff  += xwSmoothingFactor * (PID_Roll_xw_diff  - PID_Roll_DLPF_xw_diff );
  PID_Yaw_DLPF_xw_diff   += xwSmoothingFactor * (PID_Yaw_xw_diff   - PID_Yaw_DLPF_xw_diff  );
}

/**
 * @brief This method resets the tracked orientation
 * 
 * --Custom Method!
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
  PID_Yaw_DLPF_xw_diff = -PID_Yaw_DLPF_xw_diff;

  Pitch_I_Sum += (PID_Pitch_DLPF_xw_diff * Pitch_PID_k[1]);                                                                                       // integrate
  Roll_I_Sum  += (PID_Roll_DLPF_xw_diff  * Roll_PID_k[1] );
  Yaw_I_Sum   += (PID_Yaw_DLPF_xw_diff   * Yaw_PID_k[1]  );

  if (Pitch_I_Sum >   8) Pitch_I_Sum =   8;                                                                                                       //constrain integration
  if (Roll_I_Sum  >   8) Roll_I_Sum  =   8;
  if (Yaw_I_Sum   > 200) Yaw_I_Sum   = 200;
  if (Pitch_I_Sum <  -8) Pitch_I_Sum =  -8;       //NOTDONE use defines for values
  if (Roll_I_Sum  <  -8) Roll_I_Sum  =  -8;
  if (Yaw_I_Sum   <   0) Yaw_I_Sum   =   0;
  
  Pitch_D_DLPF = Pitch_D_DLPF_k * ((PID_Pitch_DLPF_xw_diff - Pitch_D_old) - Pitch_D_DLPF);    //differential filtering
  Roll_D_DLPF  = Roll_D_DLPF_k  * ((PID_Roll_DLPF_xw_diff  - Roll_D_old ) - Roll_D_DLPF );
  Yaw_D_DLPF   = Yaw_D_DLPF_k   * ((PID_Yaw_DLPF_xw_diff   - Yaw_D_old  ) - Yaw_D_DLPF  );

  PID_Pitch_y = (PID_Pitch_DLPF_xw_diff * Pitch_PID_k[0] * 10) + Pitch_I_Sum + (Pitch_D_DLPF * Pitch_PID_k[2] * 100);   //combine P,I,D values
  PID_Roll_y  = (PID_Roll_DLPF_xw_diff  * Roll_PID_k[0]  * 10) + Roll_I_Sum  + (Roll_D_DLPF  * Roll_PID_k[2]  * 100);// + (SBUS_Channels[0] / 250);
  PID_Yaw_y   = (PID_Yaw_DLPF_xw_diff   * Yaw_PID_k[0]   * 20) + Yaw_I_Sum   + (Yaw_D_DLPF   * Yaw_PID_k[2]   * 100);          //NOTDONE use defines for values

  Pitch_D_old = PID_Pitch_DLPF_xw_diff;                                                                                                      // differentiate
  Roll_D_old  = PID_Roll_DLPF_xw_diff ;
  Yaw_D_old   = PID_Yaw_DLPF_xw_diff  ;
  
  if (PID_Pitch_y >    outputStrength) PID_Pitch_y =    outputStrength;                             //output limits
  if (PID_Roll_y  >    outputStrength) PID_Roll_y  =    outputStrength;                            //NOTDONE use defines for values
  if (PID_Yaw_y   >              1000) PID_Yaw_y   =              1000;
  if (PID_Pitch_y <   -outputStrength) PID_Pitch_y =   -outputStrength;
  if (PID_Roll_y  <   -outputStrength) PID_Roll_y  =   -outputStrength;
  if (PID_Yaw_y   <                 0) PID_Yaw_y   =                 0;

  Pitch_y_ChangeRate = PID_Pitch_y - PID_Pitch_y_DLPF;                                            //get change rate
  Roll_y_ChangeRate  = PID_Roll_y  - PID_Roll_y_DLPF ;

  if (Pitch_y_ChangeRate >  Pitch_y_maxChangeRate) Pitch_y_ChangeRate =  Pitch_y_maxChangeRate;   //constrain by max change rate
  if (Roll_y_ChangeRate  >  Roll_y_maxChangeRate ) Roll_y_ChangeRate  =  Roll_y_maxChangeRate ;
  if (Pitch_y_ChangeRate < -Pitch_y_maxChangeRate) Pitch_y_ChangeRate = -Pitch_y_maxChangeRate;
  if (Roll_y_ChangeRate  < -Roll_y_maxChangeRate ) Roll_y_ChangeRate  = -Roll_y_maxChangeRate ;

  PID_Pitch_y_DLPF += PID_Pitch_y_DLPF_k * Pitch_y_ChangeRate;                                    //filter
  PID_Roll_y_DLPF  += PID_Roll_y_DLPF_k  * Roll_y_ChangeRate ;

  PID_Pitch_y_DCPL = PID_Roll_y_DLPF ; // - PID_Roll_y_DLPF_old ;
  PID_Roll_y_DCPL  = PID_Pitch_y_DLPF; // - PID_Pitch_y_DLPF_old;                                    //ROLL-PITCH decoupling

  PID_Pitch_y_DLPF_old = PID_Pitch_y_DLPF;                                                      //differentiate
  PID_Roll_y_DLPF_old  = PID_Roll_y_DLPF ;


  Pitch_DCPL_angle = (/*(M_PI / 2) -*/ DCPL_angle) * 2 * atan(PID_Pitch_y_DCPL * Pitch_DCPL_k) / M_PI;    //phase shift compensation due to acceleration
  Roll_DCPL_angle  =                   DCPL_angle  * 2 * atan(PID_Roll_y_DCPL  * Roll_DCPL_k ) / M_PI;

  Pitch_output = (PID_Pitch_y_DLPF * cos(Roll_DCPL_angle )) + (PID_Roll_y_DLPF  * sin(Pitch_DCPL_angle));
  Roll_output  = (-PID_Roll_y_DLPF * cos(Pitch_DCPL_angle)) + (PID_Pitch_y_DLPF * sin(Roll_DCPL_angle ));
  
}

/**
 * @brief This method extracts the PID values for PITCH,ROLL,YAW that can be live tuned on the remote//NOTDONE
 * 
 * --Custom Method!
 */
void getPIDValues()
{
  if (tuningMode == 1)
  {
    if ((SBUS_Channels[5] <= 1000) & (SBUS_Channels[5] >= 990))
    {
      if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) Pitch_PID_k[0] = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) *  0.01 ;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) Pitch_PID_k[1] = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) *  0.001;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <= -990) & (SBUS_Channels[6] >= -1000)) Pitch_PID_k[2] = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) *  0.16 ;    //NOTDONE use defines for values       
    }
    if ((SBUS_Channels[5] <= 10) & (SBUS_Channels[5] >= -10))
    {
      if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) Roll_PID_k[0]  = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) *  0.01 ;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) Roll_PID_k[1]  = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) *  0.001;    //NOTDONE use defines for values
      if ((SBUS_Channels[6] <= -990) & (SBUS_Channels[6] >= -1000)) Roll_PID_k[2]  = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) *  0.16 ;    //NOTDONE use defines for values       
    }
    if ((SBUS_Channels[5] <= -990) & (SBUS_Channels[5] >= -1000))
    {
      mainMotorStartOffset = -135.25 + (((float)SBUS_Channels[7]) / 8);

      //if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) Yaw_PID_k[0]   = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.8;    //NOTDONE use defines for values
      //if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) Yaw_PID_k[1]   = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 0.1;    //NOTDONE use defines for values
      //if ((SBUS_Channels[6] <= -990) & (SBUS_Channels[6] >= -1000)) Yaw_PID_k[2]   = (1 + ((float)SBUS_Channels[7] / SBUS_mappedValueMax)) * 6.4;    //NOTDONE use defines for values       
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
      //mainMotorMaxOffset = -1.518 + (((float)SBUS_Channels[7]) / 1000);
      //outputStrength = 10 + ((float)SBUS_Channels[7] / 100);
      if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) Pitch_y_maxChangeRate = (float)SBUS_Channels[7] / 256;
      if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) Roll_y_maxChangeRate  = (float)SBUS_Channels[7] / 256;
      if ((SBUS_Channels[6] <= -990) & (SBUS_Channels[6] >= -1000)) outputStrength = 2 + ((float)SBUS_Channels[7] / 500);
    }
    if ((SBUS_Channels[5] <= 10) & (SBUS_Channels[5] >= -10))
    {
      if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) Pitch_DCPL_k = (float)SBUS_Channels[7] /  100;
      if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) Roll_DCPL_k  = (float)SBUS_Channels[7] / -100;
    }
    if ((SBUS_Channels[5] <= -990) & (SBUS_Channels[5] >= -1000))
    {
      if ((SBUS_Channels[6] <= 1000) & (SBUS_Channels[6] >=   990)) DCPL_angle = (M_PI / 4) + ((float)SBUS_Channels[7] * M_PI / 4000);
      if ((SBUS_Channels[6] <=   10) & (SBUS_Channels[6] >=   -10)) mainMotorStartOffset = -135.25 + (((float)SBUS_Channels[7]) / 8);
      //mainMotorSkewOffset = (float)SBUS_Channels[7] / 25;
      //sin_SkewOffset = sin((mainMotorSkewOffset * M_PI) / 180);
      //cos_SkewOffset = cos((mainMotorSkewOffset * M_PI) / 180);
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