//PWM defines/variables


#include <string.h>
#include <stdio.h>
#include <math.h>




#define fastPPM_MinTime 1000U            //minimum number of microseconds for ON time (0%)
#define fastPPM_MaxTime 2000U            //maximum number of microseconds for ON time (100%)
#define fastPPM_Pulselength 2500U		 //1Mhz/PPM Hz(400Hz) result is in miroseconds

#define slowPPM1_MinTime 1000U            //minimum number of microseconds for ON time (0%)
#define slowPPM1_MaxTime 2000U            //maximum number of microseconds for ON time (100%)
#define slowPPM1_Pulselength 20000U		 //1Mhz/PPM Hz(50Hz) result is in miroseconds

extern uint16_t fastPPM_ONTime;
extern uint16_t fastPPM_OFFTime;
extern uint8_t fastPPM_powered;

extern uint16_t slowPPM1_ONTime;
extern uint16_t slowPPM1_OFFTime;
extern uint8_t slowPPM1_powered;

extern uint16_t adcValuesArray[2];






void loop();

/**
 * @brief takes both hall values and returns the motor angle
 * @param hall_1 sensor 1 values of any proportional range
 * @param hall_2 sensor 2 values of any proportional range
 * @retval returns the motor angle 0 <-> 359°
 */
uint16_t motorAngle(int32_t hall_1, int32_t hall_2);

/**
 * @brief takes the recieved bits and converts them to channels
 */
void SBUS_PostProcessing();
