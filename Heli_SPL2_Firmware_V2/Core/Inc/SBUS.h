#include <stdio.h>

//SBUS protocoll
//has start bit
//has 8 data bits
//has 1 odd parity bit
//has 2 stop bits
//total of 12 bit per byte

//#define SBUS_BitsPerByte 12
#define SBUS_NumberOfBytes 25   //default 25
#define SBUS_NumberOfChannels 8
//#define SBUS_NumberOfBits SBUS_BitsPerByte * SBUS_NumberOfBytes
//#define SBUS_Baud 100000
//#define APB_Clockspeed 16000000
//#define SBUS_StartTimeOffset -3  //in clock cycles/(microseconds)
//#define SBUS_ClockCyclesPerBit ((APB_Clockspeed / (TIM11->PSC + 1)) / SBUS_Baud)
//#define SBUS_interruptDeactivationTime 2200

#define SBUS_mappedValueMax 1000
#define SBUS_mappedValueMin -1000
#define SBUS_mappedValueCenter 0
#define SBUS_rawValueMax 1814
#define SBUS_rawValueMin 172.5
#define SBUS_rawValueCenter (SBUS_rawValueMin + ((SBUS_rawValueMax - SBUS_rawValueMin) / 2.0))
#define SBUS_ConversionRation ((float)(SBUS_mappedValueMax - SBUS_mappedValueCenter) / (float)(SBUS_rawValueMax - SBUS_rawValueCenter))

//extern uint16_t SBUS_timerCount;
extern uint8_t SBUS_RxBytes[SBUS_NumberOfBytes];
extern uint8_t SBUS_Bytes[SBUS_NumberOfBytes];
extern int16_t SBUS_TempChannels[SBUS_NumberOfChannels];
extern int16_t SBUS_Channels[SBUS_NumberOfChannels];
//extern uint8_t SBUS_CorruptedPackage;
extern uint8_t SBUSNewPackage;
extern uint32_t PinInterruptLastTime;

// void SBUS_RecieveBits();
void SBUS_postProcessing();