#include <stdio.h>




#define fastPPM_MinTime 1000U            //minimum number of microseconds for ON time (0%)
#define fastPPM_MaxTime 2000U            //maximum number of microseconds for ON time (100%)
#define fastPPM_CenterTime fastPPM_MinTime + ((fastPPM_MaxTime - fastPPM_MinTime) / 2)
#define fastPPM_Pulselength 2500U		      //1Mhz/PPM Hz(400Hz) result is in miroseconds
#define slowPPM1_MinTime 1000U            //minimum number of microseconds for ON time (0%)
#define slowPPM1_MaxTime 2000U            //maximum number of microseconds for ON time (100%)
#define slowPPM1_CenterTime slowPPM1_MinTime + ((slowPPM1_MaxTime - slowPPM1_MinTime) / 2)
#define slowPPM1_Pulselength 20000U		    //1Mhz/PPM Hz(50Hz) result is in miroseconds
#define tailmotorDeadzone -980
#define ESC_StartupDelay 1000             //in milliseconds
#define BT_debounceTime 1000              //in milliseconds

#define PPMmainMotorScaler ((float)(fastPPM_MaxTime - fastPPM_MinTime) / (SBUS_mappedValueMax - SBUS_mappedValueMin))
#define PPMtailMotorScaler ((float)(slowPPM1_MaxTime - slowPPM1_MinTime) / (SBUS_mappedValueMax - SBUS_mappedValueMin))

#define hall1_min 230                                               //adcValueChannel11
#define hall1_max 2300
#define hall1_center (hall1_min + ((hall1_max - hall1_min) / 2.0))
#define hall1_scaler (1.0 / (hall1_max - hall1_center))
#define hall2_min 240                                               //adcValueChannel12
#define hall2_max 2280
#define hall2_center (hall2_min + ((hall2_max - hall2_min) / 2.0))
#define hall2_scaler (1.0 / (hall2_max - hall2_center))

#define Pitch_y_scaler 0.5
#define Roll_y_scaler  0.5
extern uint16_t adcValueChannel11;
extern uint16_t adcValueChannel12;
extern uint16_t mainMotorAngle;
extern int8_t mainMotorAngleOffset;
extern float sin_OffsetAngle;
extern float cos_OffsetAngle;

void updateMainMotorSpeed();
void updateTailMotorSpeed();
void PPM_init();
void ESCCalibration();