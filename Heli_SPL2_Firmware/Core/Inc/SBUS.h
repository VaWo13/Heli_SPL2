/*
 * SBUS.h
 *
 *  Created on: 20.02.2022
 *      Author: valew
 */
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#define numberOfBits 20
#define SBUS_Baud 10000
#define APB_Clockspeed 16000000
#define SBUS_StartTimeOffset 0  //in clock cycles



#define SBUS_ClockCyclesPerBit ((APB_Clockspeed / (TIM11->PSC + 1)) / SBUS_Baud)


extern uint16_t SBUS_timerCount;

extern uint8_t SBUS_RxBitString[numberOfBits];




void read_SBUS();




#endif /* INC_SBUS_H_ */
