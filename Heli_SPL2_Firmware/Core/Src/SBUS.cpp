/*
 * SBUS.cpp
 *
 *  Created on: 20.02.2022
 *      Author: valew
 */

#include "SBUS.h"

#include "stm32f2xx_hal.h"
#include "main.h"
#include "realMain.h"

#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint16_t SBUS_timerCount = 0;
uint8_t SBUS_RxBitString[SBUS_NumberOfBits];
uint8_t SBUS_Bytes[SBUS_NumberOfBytes];
int16_t SBUS_TempChannels[SBUS_NumberOfChannels];
int16_t SBUS_Channels[SBUS_NumberOfChannels];
uint8_t SBUS_CorruptedPackage = false;
uint8_t SBUSNewPackage = false;



void SBUS_RecieveBits()
{
  //collect bits:

  TIM4->CCR1 = (uint16_t)(fastPPM_MinTime + 500 + ((float)SBUS_Channels[2] / 2));
  SBUS_timerCount = TIM11->CNT + SBUS_StartTimeOffset;       //get current clock count register value + time offset
  SBUS_RxBitString[0] = true;
  //HAL_GPIO_TogglePin(ONBOARD_WRITE_3_GPIO_Port, ONBOARD_WRITE_3_Pin);   //debug Pin
  for (size_t i = 1; i < SBUS_NumberOfBits; i++)
  {
    ONBOARD_WRITE_3_GPIO_Port->BSRR = (uint32_t)ONBOARD_WRITE_3_Pin << 16U;
    SBUS_RxBitString[i] = ((ONBOARD_READ_IT_3_GPIO_Port->IDR & ONBOARD_READ_IT_3_Pin) != 0 ? true : false);     //if the pin is HIGH then the value is 1 else 0
    ONBOARD_WRITE_3_GPIO_Port->BSRR = ONBOARD_WRITE_3_Pin;
    while ((TIM11->CNT - SBUS_timerCount) < 10)
    {
    }
    SBUS_timerCount += 10;
  }
  SBUSNewPackage = true;
}

void SBUS_PostProcessing()
{
    //check validity:

  SBUS_CorruptedPackage = false;                      //reset corrupted package flag
  for (size_t i = 0; i < SBUS_NumberOfBytes; i++)      //repeat for each byte
  {
    if ((SBUS_RxBitString[0 + (i * SBUS_BitsPerByte)] == true) & (SBUS_RxBitString[10 + (i * SBUS_BitsPerByte)] == false) & (SBUS_RxBitString[11 + (i * SBUS_BitsPerByte)] == false))       //check start bit, 2 stop bit
    {
      uint8_t parityCheck = false;
      for (size_t x = 0; x < 8; x++)                                              //generate parity from 8 bits
      {
        parityCheck ^= SBUS_RxBitString[1 + x + (i * SBUS_BitsPerByte)];
      }
      if (parityCheck == SBUS_RxBitString[9 + (i * SBUS_BitsPerByte)])            //if parity fails set corrupted flag
      {
        SBUS_CorruptedPackage = true;
      }
    }
    else                                                                            //if start or stop bits fail set corrupted flag
    {
      SBUS_CorruptedPackage = true;
    }
  }
  SBUS_Bytes[0] = 0;
  for (size_t i = 0; i < 8; i++)
  {
    SBUS_Bytes[0] |= SBUS_RxBitString[1 + i] << (7 - i);
  }
  if (SBUS_Bytes[0] != 0x0FU)
  {
    SBUS_CorruptedPackage = true;
  }
  
  

  //assemble channels:

  if (SBUS_CorruptedPackage == false)
  {
    uint8_t byteNumber = 0;         //0 to 21
    uint8_t bitNumber = 0;          //0(LSB) to 7/(MSB)
    for (size_t i = 0; i < SBUS_NumberOfChannels; i++)
    {
      uint8_t bitInChannel = 0;   //0(LSB) to 10(MSB)
      SBUS_TempChannels[i] = 0;
      
      while (bitInChannel <= 10)
      {
        if (bitNumber <= 7)
        {
          //transfer bit
          SBUS_TempChannels[i] |= SBUS_RxBitString[13 + (bitNumber) + (byteNumber * SBUS_BitsPerByte)] << bitInChannel;
        }
        else
        {
          byteNumber ++;
          bitNumber = 0;
          //transfer bit
          SBUS_TempChannels[i] |= SBUS_RxBitString[13 + (bitNumber) + (byteNumber * SBUS_BitsPerByte)] << bitInChannel;
        }
        bitNumber ++;
        bitInChannel ++;
      }
      
      SBUS_TempChannels[i] = ((float)(SBUS_TempChannels[i] - 1054) * ((float)-1000 / (float)821));    //map from 233, 1875 to -1000, 1000
      SBUS_Channels[i] = SBUS_TempChannels[i];
    }
  }
  SBUSNewPackage = false;
}