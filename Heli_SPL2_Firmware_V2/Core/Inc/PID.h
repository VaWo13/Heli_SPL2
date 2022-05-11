#include <stdio.h>

#define xwSmoothingFactor 1

#define Pitch_D_DLPF_k 0.1
#define Roll_D_DLPF_k  0.1
#define Yaw_D_DLPF_k   0.3

#define PID_Pitch_y_DLPF_k 0.4
#define PID_Roll_y_DLPF_k  0.4

extern uint8_t tuningMode;
extern uint8_t buttonPressed;

extern float PID_Pitch_xw_diff;
extern float PID_Roll_xw_diff;
extern float PID_Yaw_xw_diff;

extern float PID_Pitch_DLPF_xw_diff;
extern float PID_Roll_DLPF_xw_diff;
extern float PID_Yaw_DLPF_xw_diff;

extern float PID_Pitch_y;
extern float PID_Roll_y;
extern float PID_Yaw_y;

extern float Pitch_output;
extern float Roll_output;

extern float outputStrength;

extern float MPUoutputQuaternion[4];
extern float OriginQuaternion[4];
extern float OriginToOutputQuaternion[4];
extern float FrameOriginQuaternion[4];
extern float LoopWQuaternion[4];            //NOTDONE unused? rename?
extern float GyroOriginQuaternion[4];
extern float LoopXWQuaternion[4];
extern float updateQuaternion[4];

extern float Pitch_PID_k[3];
extern float Roll_PID_k[3];
extern float Yaw_PID_k[3];
extern float Roll_DCPL_k;
extern float Pitch_DCPL_k;
extern float DCPL_angle;
extern float Pitch_y_maxChangeRate;
extern float Roll_y_maxChangeRate;
extern float Pitch_y_ChangeRate;
extern float Roll_y_ChangeRate;
extern float PID_Pitch_y_DLPF;
extern float PID_Roll_y_DLPF;

extern float Pitch_D_DLPF;
extern float Roll_D_DLPF;
extern float Yaw_D_DLPF;
extern float PID_Pitch_y_DCPL;
extern float PID_Roll_y_DCPL;
extern float Pitch_DCPL_angle;
extern float Roll_DCPL_angle;
extern float PID_Pitch_y_DLPF_old;
extern float PID_Roll_y_DLPF_old;
extern float Pitch_I_Sum;
extern float Roll_I_Sum;
extern float Yaw_I_Sum;
extern float Pitch_D_old;
extern float Roll_D_old;
extern float Yaw_D_old;


void get_XW_diffAngles();
void reset_WQuaternion();
void update_PID();
void getPIDValues();
void getAngleOffset();
void switchTuningMode();