#include <stdio.h>

extern uint8_t tuningMode;
extern uint8_t buttonPressed;

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
extern float LoopWQuaternion[4];            //NOTDONE unused? rename?
extern float LoopXQuaternion[4];
extern float LoopXWQuaternion[4];
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


void get_XW_diffAngles();
void update_PID();
void getPIDValues();
void getAngleOffset();
void switchTuningMode();