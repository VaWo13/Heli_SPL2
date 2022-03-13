#include <string.h>
#include <stdio.h>
#include <math.h>



extern float MPUoutputQuaternion[4];
extern float OriginQuaternion[4];
extern float OriginToOutputQuaternion[4];
extern float FrameOriginQuaternion[4];
extern float LoopWQuaternion[4];
extern float LoopXQuaternion[4];
extern float LoopWXQuaternion[4];

void getWXQuaternion();

extern float *QuaternionProduct(float *q1, float *q2);
extern float *QuaternionInverse(float *q1);
extern float *QuaternionSLERP(float *q1, float *q2);

void MPU6050_Calibration();