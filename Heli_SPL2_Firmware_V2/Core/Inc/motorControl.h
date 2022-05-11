#include <stdio.h>
#include "human_interface.h"




#define fastPPM_MinTime 1000U            //minimum number of microseconds for ON time (0%)
#define fastPPM_MaxTime 1550U            //maximum number of microseconds for ON time (100%)
#define fastPPM_CenterTime fastPPM_MinTime + ((fastPPM_MaxTime - fastPPM_MinTime) / 2)
#define fastPPM_Pulselength 1667U		      //1Mhz/PPM Hz(400Hz) result is in miroseconds
#define slowPPM1_MinTime 1000U            //minimum number of microseconds for ON time (0%)
#define slowPPM1_MaxTime 2000U            //maximum number of microseconds for ON time (100%)
#define slowPPM1_CenterTime slowPPM1_MinTime + ((slowPPM1_MaxTime - slowPPM1_MinTime) / 2)
#define slowPPM1_Pulselength 20000U		    //1Mhz/PPM Hz(50Hz) result is in miroseconds

#define fastPPM_calcutationTime 50

#define motorDeadzone -950
#define BT_debounceTime 1000              //in milliseconds

#define PPMmainMotorScaler ((float)(fastPPM_MaxTime - fastPPM_MinTime) / (SBUS_mappedValueMax - SBUS_mappedValueMin))
#define PPMtailMotorScaler ((float)(slowPPM1_MaxTime - slowPPM1_MinTime) / (SBUS_mappedValueMax - SBUS_mappedValueMin))

#define hall1_min 245                                               //adcValueChannel11
#define hall1_max 2273
#define hall1_center (hall1_min + ((hall1_max - hall1_min) / 2.0))
#define hall1_scaler (1.0 / (hall1_max - hall1_center))
#define hall2_min 232                                               //adcValueChannel12
#define hall2_max 2295
#define hall2_center (hall2_min + ((hall2_max - hall2_min) / 2.0))
#define hall2_scaler (1.0 / (hall2_max - hall2_center))

#define MainMotorSmoothingFactor 0.05
#define periodSmoothingFactor 0.08

#define Pitch_y_scaler 0.5
#define Roll_y_scaler  0.5

#define mainMotorCyclicSpeed_leaky_I_k 0.1


extern uint16_t adcValueChannel11;

extern uint16_t old_adcValueChannel11;
extern uint16_t adc_TimestampMicros;
extern uint16_t old_adc_TimestampMicros;
extern uint32_t adc_TimestampMillis;
extern uint32_t old_adc_TimestampMillis;

extern uint16_t adcValueChannel12;

extern uint32_t MainMotor_CTS_Millis;
extern uint32_t old_MainMotor_CTS_Millis;
extern uint16_t MainMotor_CTS_Micros;
extern uint16_t old_MainMotor_CTS_Micros;
extern float mainMotorPeriod;
extern float mainMotorPeriod_DLPF;

extern uint16_t mainMotorAngle;
extern float mainMotorStartOffset;
extern float mainMotorMaxOffset;
extern float mainMotorSkewOffset;
extern float smoothMainMotorSpeed;
extern float mainMotorCyclicAccel;
extern float mainMotorCyclicSpeed;
extern float sin_OffsetAngle;
extern float cos_OffsetAngle;
extern float sin_SkewOffset;
extern float cos_SkewOffset;

void updateMainMotorSpeed();
void MainMotorDLPF();
void updateTailMotorSpeed();
void getMainMotorSpeed();
void getMainMotorOffset();
void PPM_init();
void ESCCalibration();