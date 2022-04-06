#include "SBUS.h"
#include "motorControl.h"

#include "stm32f2xx_hal.h"
#include "usbd_cdc_if.h"

uint16_t SBUS_timerCount = 0;
uint8_t SBUS_RxBitString[SBUS_NumberOfBits];
uint8_t SBUS_Bytes[SBUS_NumberOfBytes];
int16_t SBUS_TempChannels[SBUS_NumberOfChannels];
int16_t SBUS_Channels[SBUS_NumberOfChannels];
uint8_t SBUS_CorruptedPackage = true;
uint8_t SBUSNewPackage = false;
uint8_t interruptEnabled = true;
uint32_t PinInterruptLastTime = 0;

/**
 * @brief This method gets called when the SBUS pin detects a rising edge.
 * It then synchronizes itself with the xMhz Clock from TIM11 and reads the
 * pin every x clock pulses (x Baud), storing the values in an array
 * for later processing.
 * 
 * --Custom Method!
 */
void SBUS_RecieveBits()
{
  TIM4->CCR1 = (uint16_t)(fastPPM_CenterTime + (smoothMainMotorSpeed * PPMmainMotorScaler));
  SBUS_timerCount = TIM11->CNT + SBUS_StartTimeOffset;                                                      //get current clock count from TIM11 + time offset
  for (size_t i = 1; i < SBUS_NumberOfBits; i++)
  {
    ONBOARD_WRITE_3_GPIO_Port->BSRR = (uint32_t)ONBOARD_WRITE_3_Pin << 16U;                                 //NOTDONE debug
    SBUS_RxBitString[i] = ((ONBOARD_READ_IT_3_GPIO_Port->IDR & ONBOARD_READ_IT_3_Pin) != 0 ? true : false); //if the SBUS pin is HIGH then the value is 1 else 0
    ONBOARD_WRITE_3_GPIO_Port->BSRR = ONBOARD_WRITE_3_Pin;                                                  //NOTDONE debug
    while ((TIM11->CNT - SBUS_timerCount) < SBUS_ClockCyclesPerBit)                                         //wait until x clock pulses passed
    {
    }
    SBUS_timerCount += SBUS_ClockCyclesPerBit;
  }
  SBUS_RxBitString[0] = true;                                                                               //sets the first value in the array to 1 as the interrupt is not fast
  SBUSNewPackage = true;                                                                                    //\->enough to be able to measure the pin for the first bit
}

/**
 * @brief This methos gets called when there is a new set of SBUS bits available.
 * It first checks the validity of the packet by looking at the START, STOP and
 * PARITY bits of all bytes. It then takes the bits and assembles them into 11-Bit
 * channels.
 * 
 * --Custom Method!
 */
void SBUS_postProcessing()
{
  //verify:

  SBUS_CorruptedPackage = false;                                                                                          //reset corrupted package flag
  for (size_t i = 0; i < SBUS_NumberOfBytes; i++)                                                                         //repeat for each byte
  {
    if ((SBUS_RxBitString[0 + (i * SBUS_BitsPerByte)] == true) & (SBUS_RxBitString[10 + (i * SBUS_BitsPerByte)] == false) & (SBUS_RxBitString[11 + (i * SBUS_BitsPerByte)] == false))
    {                                                                                                                     //^->check start bit, 2 stop bit
      uint8_t parityCheck = false;
      for (size_t x = 0; x < 8; x++)                                                                                      //generate parity from 8 bits
      {
        parityCheck ^= SBUS_RxBitString[1 + x + (i * SBUS_BitsPerByte)];
      }
      if (parityCheck == SBUS_RxBitString[9 + (i * SBUS_BitsPerByte)]) SBUS_CorruptedPackage = true;                      //if parity fails set corrupted flag
    }
    else SBUS_CorruptedPackage = true;                                                                                    //if start or stop bits fail set corrupted flag
  }

  SBUS_Bytes[0] = 0;                                                                                                      //Reset the first byte (header byte)
  for (size_t i = 0; i < 8; i++)
  {
    SBUS_Bytes[0] |= SBUS_RxBitString[1 + i] << (7 - i);                                                                  //assemble new header byte
  }
  if (SBUS_Bytes[0] != 0x0FU) SBUS_CorruptedPackage = true;                                                               //check if header byte is 0x0F

  //assemble channels:

  if (SBUS_CorruptedPackage == false)
  {
    uint8_t byteNumber = 0;
    uint8_t bitNumber = 0;                                                                                                //0(LSB) to 7/(MSB)
    for (size_t i = 0; i < SBUS_NumberOfChannels; i++)
    {
      uint8_t bitInChannel = 0;                                                                                           //0(LSB) to 10(MSB)
      SBUS_TempChannels[i] = 0;
      while (bitInChannel <= 10)
      {
        if (bitNumber <= 7)
        {
          SBUS_TempChannels[i] |= SBUS_RxBitString[13 + (bitNumber) + (byteNumber * SBUS_BitsPerByte)] << bitInChannel;   //transfer bit
        }
        else
        {
          byteNumber ++;
          bitNumber = 0;
          SBUS_TempChannels[i] |= SBUS_RxBitString[13 + (bitNumber) + (byteNumber * SBUS_BitsPerByte)] << bitInChannel;   //transfer bit
        }
        bitNumber ++;
        bitInChannel ++;
      }
      
      SBUS_Channels[i] = ((SBUS_rawValueCenter - (float)SBUS_TempChannels[i]) * SBUS_ConversionRation);            //map from 233, 1875 to -1000, 1000 and transfer channel value
    }
  }
  SBUSNewPackage = false;                                                                                                 //reset flag
}