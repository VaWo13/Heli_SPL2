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


//has start bit
//has 8 data bits
//has 1 odd parity bit
//has 2 stop bits
//total of 12 bit per byte

#define SBUS_BitsPerByte 12
#define SBUS_NumberOfBytes 25   //default 25
#define SBUS_NumberOfChannels 8
#define SBUS_NumberOfBits SBUS_BitsPerByte * SBUS_NumberOfBytes

#define SBUS_Baud 100000
#define APB_Clockspeed 16000000
#define SBUS_StartTimeOffset -5  //in clock cycles/(microseconds)






#define SBUS_ClockCyclesPerBit ((APB_Clockspeed / (TIM11->PSC)) / SBUS_Baud)


extern uint16_t SBUS_timerCount;

extern uint8_t SBUS_RxBitString[SBUS_NumberOfBits];

extern uint8_t SBUS_Bytes[SBUS_NumberOfBytes];

extern int16_t SBUS_TempChannels[SBUS_NumberOfChannels];
extern int16_t SBUS_Channels[SBUS_NumberOfChannels];        //values expected from -1000 to 1000

extern uint8_t SBUS_CorruptedPackage;
extern uint8_t SBUSNewPackage;
// extern uint8_t SBUS_RecieveDone;



void SBUS_RecieveBits();




#endif /* INC_SBUS_H_ */
