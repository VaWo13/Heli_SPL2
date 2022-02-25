/*
 * SBUS.cpp
 *
 *  Created on: 20.02.2022
 *      Author: valew
 */

#include "SBUS.h"

#include "stm32f2xx_hal.h"
#include "main.h"

#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint16_t SBUS_timerCount = 0;
uint8_t SBUS_RxBitString[SBUS_NumberOfBits];
uint8_t SBUS_Bytes[SBUS_NuberOfBytes];
uint16_t SBUS_Channels[SBUS_NumberOfChannels];
uint8_t SBUS_CorruptedPackage = false;

void read_SBUS()
{
    //collect bits:

    SBUS_timerCount = TIM11->CNT + SBUS_StartTimeOffset;       //get current clock count register value + time offset
    SBUS_RxBitString[0] = true;
    
    HAL_GPIO_TogglePin(ONBOARD_WRITE_4_GPIO_Port, ONBOARD_WRITE_4_Pin);     //debug pin

    for (size_t i = 1; i < SBUS_NumberOfBits; i++)
    {
        HAL_GPIO_TogglePin(ONBOARD_WRITE_4_GPIO_Port, ONBOARD_WRITE_4_Pin);     //debug pin
        SBUS_RxBitString[i] = ((ONBOARD_READ_IT_3_GPIO_Port->IDR & ONBOARD_READ_IT_3_Pin) != 0 ? true : false);     //if the pin is HIGH then the value is 1 else 0

        while ((TIM11->CNT - SBUS_timerCount) < 10)
        {
        }
        SBUS_timerCount += 10;
    }

    //check validity:

    SBUS_CorruptedPackage = false;          //reset corrupted package flag

    for (size_t i = 0; i < SBUS_NuberOfBytes; i++)      //repeat for each byte
    {
        if ((SBUS_RxBitString[0 + (i * SBUS_BitsPerByte)] == true) & (SBUS_RxBitString[10 + (i * SBUS_BitsPerByte)] == false) & (SBUS_RxBitString[11 + (i * SBUS_BitsPerByte)] == false))       //check start bit, 2 stop bit
        {
            uint8_t parityCheck = false;
            for (size_t x = 0; x < 8; x++)          //generate parity from 8 bits
            {
                parityCheck ^= SBUS_RxBitString[1 + x + (i * SBUS_BitsPerByte)];
            }

            if (parityCheck != SBUS_RxBitString[9 + (i * SBUS_BitsPerByte)])            //if parity is ok then transfer the bits into the byte
            {
                // SBUS_Bytes[i] = 0;
                // SBUS_Bytes[i] |= SBUS_RxBitString[1 + 0 + (i * SBUS_BitsPerByte)] << 7;
                // SBUS_Bytes[i] |= SBUS_RxBitString[1 + 1 + (i * SBUS_BitsPerByte)] << 6;
                // SBUS_Bytes[i] |= SBUS_RxBitString[1 + 2 + (i * SBUS_BitsPerByte)] << 5;
                // SBUS_Bytes[i] |= SBUS_RxBitString[1 + 3 + (i * SBUS_BitsPerByte)] << 4;
                // SBUS_Bytes[i] |= SBUS_RxBitString[1 + 4 + (i * SBUS_BitsPerByte)] << 3;
                // SBUS_Bytes[i] |= SBUS_RxBitString[1 + 5 + (i * SBUS_BitsPerByte)] << 2;
                // SBUS_Bytes[i] |= SBUS_RxBitString[1 + 6 + (i * SBUS_BitsPerByte)] << 1;
                // SBUS_Bytes[i] |= SBUS_RxBitString[1 + 7 + (i * SBUS_BitsPerByte)]     ;
            }
            else    //if parity fails set corrupted flag
            {
                SBUS_CorruptedPackage = true;
            }
        }
        else    //if start or stop bits fail set corrupted flag
        {
            SBUS_CorruptedPackage = true;
        }
    }
    
    //assemble channels:

    if (SBUS_CorruptedPackage == false)
    {
        uint8_t byteNumber = 0;         //0 to 21
        uint8_t bitNumber = 0;          //0(LSB) to 7/(MSB)
        for (size_t i = 0; i < SBUS_NumberOfChannels; i++)
        {
            uint8_t bitInChannel = 0;   //0(LSB) to 10(MSB)
            SBUS_Channels[i] = 0;
            
            while (bitInChannel <= 10)
            {
                if (bitNumber <= 7)
                {
                    //transfer bit
                    SBUS_Channels[i] |= SBUS_RxBitString[13 + (bitNumber) + (byteNumber * SBUS_BitsPerByte)] << bitInChannel;
                }
                else
                {
                    byteNumber ++;
                    bitNumber = 0;
                    
                    //transfer bit
                    SBUS_Channels[i] |= SBUS_RxBitString[13 + (bitNumber) + (byteNumber * SBUS_BitsPerByte)] << bitInChannel;
                }
                bitNumber ++;
                bitInChannel ++;
            }
        }
    }
}