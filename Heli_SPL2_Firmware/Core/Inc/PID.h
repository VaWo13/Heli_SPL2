#include <string.h>
#include <stdio.h>
#include <math.h>



extern float PID_Pitch_xw_diff;
extern float PID_Roll_xw_diff;
extern float PID_Yaw_xw_diff;

extern float PID_Pitch_y;
extern float PID_Roll_y;
extern float PID_Yaw_y;

extern float MPUoutputQuaternion[4];
extern float OriginQuaternion[4];
extern float OriginToOutputQuaternion[4];
extern float FrameOriginQuaternion[4];
extern float LoopWQuaternion[4];
extern float LoopXQuaternion[4];
extern float LoopWXQuaternion[4];
extern float updateQuaternion[4];

extern float Pitch_PID_k[3];
extern float Roll_PID_k[3];
extern float Yaw_PID_k[3];
extern float Pitch_I_Sum;
extern float Roll_I_Sum;
extern float Yaw_I_Sum;
extern float Pitch_D_old;
extern float Roll_D_old;
extern float Yaw_D_old;


void getWXQuaternion();
void Update_FrameOriginQuaternion();
void Update_PID();


extern float *QuaternionProduct(float *q1, float *q2);
extern float *QuaternionInverse(float *q1);
extern float *QuaternionSLERP(float *q1, float *q2);
extern float *QuaternionNormalize(float *q1);

void MPU6050_GetOriginQuaternion();