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


extern uint16_t adcValueChannel11;
extern uint16_t adcValueChannel12;
extern uint16_t mainMotorAngle;
extern int8_t mainMotorAngleOffset;

void updateMainMotorSpeed();
void updateTailMotorSpeed();
void PPM_init();
void ESCCalibration();