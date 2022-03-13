#include "PID.h"

#include "stm32f2xx_hal.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

float MPUoutputQuaternion[4];
float OriginQuaternion[4];
float OriginToOutputQuaternion[4];
float FrameOriginQuaternion[4] = {1, 0, 0, 0};
float LoopWQuaternion[4] = {1, 0, 0, 0};
float LoopXQuaternion[4];
float LoopWXQuaternion[4];













void getWXQuaternion()
{
  float *p = QuaternionSLERP(QuaternionProduct(&FrameOriginQuaternion[0] , QuaternionSLERP(&OriginQuaternion[0], &MPUoutputQuaternion[0])), &LoopWQuaternion[0]);
  LoopWXQuaternion[0] = *p;
  LoopWXQuaternion[1] = *(p + 1);
  LoopWXQuaternion[2] = *(p + 2);
  LoopWXQuaternion[3] = *(p + 3);
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

void MPU6050_Calibration()
{
  OriginQuaternion[0] = MPUoutputQuaternion[0];
  OriginQuaternion[1] = MPUoutputQuaternion[1];
  OriginQuaternion[2] = MPUoutputQuaternion[2];
  OriginQuaternion[3] = MPUoutputQuaternion[3];
}
