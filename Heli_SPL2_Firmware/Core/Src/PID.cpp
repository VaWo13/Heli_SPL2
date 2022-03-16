#include "PID.h"
#include "SBUS.h"

#include "stm32f2xx_hal.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

float PID_Pitch_xw_diff;
float PID_Roll_xw_diff;
float PID_Yaw_xw_diff;

float PID_Pitch_y;
float PID_Roll_y;
float PID_Yaw_y;

float MPUoutputQuaternion[4];
float OriginQuaternion[4];
float OriginToOutputQuaternion[4];
float FrameOriginQuaternion[4] = {1, 0, 0, 0};
float LoopWQuaternion[4] = {1, 0, 0, 0};
float LoopXQuaternion[4];
float LoopWXQuaternion[4];
float updateQuaternion[4];

float Pitch_PID_k[3] = {0, 0, 0};
float Roll_PID_k[3]  = {0, 0, 0};
float Yaw_PID_k[3]   = {0, 0, 0};
float Pitch_I_Sum;
float Roll_I_Sum;
float Yaw_I_Sum;
float Pitch_D_old;
float Roll_D_old;
float Yaw_D_old;

void getWXQuaternion()
{
  float *p = QuaternionSLERP(QuaternionProduct(&FrameOriginQuaternion[0] , QuaternionSLERP(&OriginQuaternion[0], &MPUoutputQuaternion[0])), &LoopWQuaternion[0]);
  LoopWXQuaternion[0] = *p;
  LoopWXQuaternion[1] = *(p + 1);
  LoopWXQuaternion[2] = *(p + 2);
  LoopWXQuaternion[3] = *(p + 3);

  //difference x-w in degrees
  PID_Pitch_xw_diff = 2 * (((float)asin(LoopWXQuaternion[1]) * 180) / M_PI);
  PID_Roll_xw_diff  = 2 * (((float)asin(LoopWXQuaternion[2]) * 180) / M_PI);
  PID_Yaw_xw_diff   = 2 * (((float)atan(LoopWXQuaternion[3] / LoopWXQuaternion[0]) * 180) / M_PI);
}

void Update_FrameOriginQuaternion()
{
  updateQuaternion[0] = cos((float)SBUS_Channels[3] / 10000);
  updateQuaternion[1] = (float)sin((float)SBUS_Channels[1] / (float)10000);
  updateQuaternion[2] = (float)sin((float)SBUS_Channels[0] / (float)10000);
  updateQuaternion[3] = sin((float)SBUS_Channels[3] / 10000);

  float *p1 = QuaternionNormalize(&updateQuaternion[0]);
  updateQuaternion[0] = *p1;
  updateQuaternion[1] = *(p1 + 1);
  updateQuaternion[2] = *(p1 + 2);
  updateQuaternion[3] = *(p1 + 3);

  float *p2 = QuaternionProduct(&updateQuaternion[0], &LoopWQuaternion[0]);
  LoopWQuaternion[0] = *p2;
  LoopWQuaternion[1] = *(p2 + 1);
  LoopWQuaternion[2] = *(p2 + 2);
  LoopWQuaternion[3] = *(p2 + 3);

    float *p3 = QuaternionNormalize(&LoopWQuaternion[0]);
  LoopWQuaternion[0] = *p3;
  LoopWQuaternion[1] = *(p3 + 1);
  LoopWQuaternion[2] = *(p3 + 2);
  LoopWQuaternion[3] = *(p3 + 3);
}

void Update_PID()
{
  Pitch_I_Sum += (PID_Pitch_xw_diff * Pitch_PID_k[1]);
  Roll_I_Sum  += (PID_Roll_xw_diff  * Roll_PID_k[1] );
  Yaw_I_Sum   += (PID_Yaw_xw_diff   * Yaw_PID_k[1]  );

  if (Pitch_I_Sum > 500 ) Pitch_I_Sum = 500 ;
  if (Roll_I_Sum  > 500 ) Roll_I_Sum  = 500 ;
  if (Yaw_I_Sum   > 500 ) Yaw_I_Sum   = 500 ;
  if (Pitch_I_Sum < -500) Pitch_I_Sum = -500;
  if (Roll_I_Sum  < -500) Roll_I_Sum  = -500;
  if (Yaw_I_Sum   < -500) Yaw_I_Sum   = -500;
  

  PID_Pitch_y = (PID_Pitch_xw_diff * Pitch_PID_k[0] * 10) + Pitch_I_Sum + ((PID_Pitch_xw_diff - Pitch_D_old) * Pitch_PID_k[2] * 100);
  PID_Roll_y  = (PID_Roll_xw_diff  * Roll_PID_k[0]  * 10) + Roll_I_Sum  + ((PID_Roll_xw_diff  - Roll_D_old ) * Roll_PID_k[2]  * 100);
  PID_Yaw_y   = (PID_Yaw_xw_diff   * Yaw_PID_k[0]   * 10) + Yaw_I_Sum   + ((PID_Yaw_xw_diff   - Yaw_D_old  ) * Yaw_PID_k[2]   * 100);

  Pitch_D_old = PID_Pitch_xw_diff;
  Roll_D_old  = PID_Roll_xw_diff ;
  Yaw_D_old   = PID_Yaw_xw_diff  ;

  if (PID_Pitch_y > 500 ) PID_Pitch_y = 500 ;
  if (PID_Roll_y  > 500 ) PID_Roll_y  = 500 ;
  if (PID_Yaw_y   > 500 ) PID_Yaw_y   = 500 ;
  if (PID_Pitch_y < -500) PID_Pitch_y = -500;
  if (PID_Roll_y  < -500) PID_Roll_y  = -500;
  if (PID_Yaw_y   < -500) PID_Yaw_y   = -500;
}

void getPIDValues()
{
  switch (SBUS_Channels[5])
  {
  case -999:   //Pitch
    switch (SBUS_Channels[6])
    {
    case 999:   //P
      Pitch_PID_k[0] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
    default:     //I
      Pitch_PID_k[1] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
    case -999:  //D
      Pitch_PID_k[2] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
  }
    break;
  default:     //Roll
    switch (SBUS_Channels[6])
    {
    case 999:   //P
      Roll_PID_k[0] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
    default:     //I
      Roll_PID_k[1] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
    case -999:  //D
      Roll_PID_k[2] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
  }
    break;
  case 999:  //Yaw
    switch (SBUS_Channels[6])
    {
    case 999:   //P
      Yaw_PID_k[0] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
    default:     //I
      Yaw_PID_k[1] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
    case -999:  //D
      Yaw_PID_k[2] = 0.5 + ((float)SBUS_Channels[7] / 2000);
      break;
    }
    break;
  }
}

void MPU6050_GetOriginQuaternion()
{
  OriginQuaternion[0] = MPUoutputQuaternion[0];
  OriginQuaternion[1] = MPUoutputQuaternion[1];
  OriginQuaternion[2] = MPUoutputQuaternion[2];
  OriginQuaternion[3] = MPUoutputQuaternion[3];
}

/**
 * @brief Generates the product of 2 quaternions. !!ORDER MATTERS!!
 * 
 * @param q1 first Quaternion
 * @param q2 second Quaternion
 * @return float product-Quaternion
 */
float *QuaternionProduct(float *q1, float *q2)
{
  static float q3[4];
  q3[0] = (q1[0] * q2[0]) - (q1[1] * q2[1]) - (q1[2] * q2[2]) - (q1[3] * q2[3]);
  q3[1] = (q1[0] * q2[1]) + (q1[1] * q2[0]) + (q1[2] * q2[3]) - (q1[3] * q2[2]);
  q3[2] = (q1[0] * q2[2]) - (q1[1] * q2[3]) + (q1[2] * q2[0]) + (q1[3] * q2[1]);
  q3[3] = (q1[0] * q2[3]) + (q1[1] * q2[2]) - (q1[2] * q2[1]) + (q1[3] * q2[0]);
  return q3;
}

/**
 * @brief takes a Quaternion and inverses it
 * 
 * @param q1 quaternion to be inversed
 * @return float inverse Quaternion
 */
float *QuaternionInverse(float *q1)
{
  static float qi[4];
  qi[0] = q1[0];
  qi[1] = q1[1] * -1;
  qi[2] = q1[2] * -1;
  qi[3] = q1[3] * -1;
  return qi;
}

/**
 * @brief (SLERP)(Spherical Linear Interpolation): Gets the quaternion thats needed to get from
 *  the start quaternion(q1) to the end Quaternion(q2)
 * 
 * @param q1 start quaternion
 * @param q2 end quaternion
 * @return float interpolation quaternion
 */
float * QuaternionSLERP(float *q1, float *q2)
{
  return QuaternionProduct(&q2[0], QuaternionInverse(&q1[0]));
}

/**
 * @brief normalizes a quaternion
 * 
 * @param q1 quaternion to be normalized
 * @return float* pointer to normalized quaternion
 */
float *QuaternionNormalize(float *q1)
{
  float vectorlength = sqrt((q1[0] * q1[0]) + (q1[1] * q1[1]) + (q1[2] * q1[2]) + (q1[3] * q1[3]));
  static float qn[4];
  qn[0] = q1[0] / vectorlength;
  qn[1] = q1[1] / vectorlength;
  qn[2] = q1[2] / vectorlength;
  qn[3] = q1[3] / vectorlength;
  return qn;
}