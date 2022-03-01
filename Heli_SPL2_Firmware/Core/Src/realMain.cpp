#include "main.h"
#include "realMain.h"
#include "SBUS.h"
#include "MPU6050.h"

#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint32_t timestamp = 0;
uint16_t adcValuesArray[2];

void loop()
{
  
  if ((HAL_GetTick() - timestamp) >= 50)
  {
    timestamp += 50;

     //for (size_t i = 0; i < 1; i++)
     //{
     //  unsigned char msg[300];
	   //  sprintf((char*)msg,"%hd %hd %hd %hd %hd %hd\r\n", MPU_Values[0], MPU_Values[1], MPU_Values[2], MPU_Values[3], MPU_Values[4], MPU_Values[5]);
	   //  uint8_t x = 0;
	   //  while (msg[x] != NULL)
	   //  {
	   //  	x++;
	   //  }
	   //  unsigned char msgTransmit[x];
	   //  for (size_t i = 0; i < x; i++)
	   //  {
	   //  	msgTransmit[i] = msg[i];
	   //  }
	   //  CDC_Transmit_FS((unsigned char*)msgTransmit, sizeof(msgTransmit));
     //}

    MPU6050_readDMP_Quaterions();

    EXTI->IMR |= (EXTI_LINE_0);   //enable Pin interrupt

    for (size_t i = 0; i < 1; i++)
    {
      unsigned char msg[300];
	    sprintf((char*)msg," %lu %lu %lu %lu %hu\r\n", Quaternions[0], Quaternions[1], Quaternions[2], Quaternions[3], FIFOCounter);
	    uint8_t x = 0;
	    while (msg[x] != NULL)
	    {
	    	x++;
	    }
	    unsigned char msgTransmit[x];
	    for (size_t i = 0; i < x; i++)
	    {
	    	msgTransmit[i] = msg[i];
	    }
	    CDC_Transmit_FS((unsigned char*)msgTransmit, sizeof(msgTransmit));
    }
    

  }
  ADC_Select_Channel_11();
	adcValuesArray[0] = (uint16_t)ADC1->DR;
	ADC_Select_Channel_12();
	adcValuesArray[1] = (uint16_t)ADC1->DR;

  uint16_t angle = motorAngle(adcValuesArray[1] - 1250, adcValuesArray[0] - 1250);

  slowPPM1_ONTime = (uint16_t)((((float)angle * (float)slowPPM1_MinTime) / (float)360) + (float)slowPPM1_MinTime);
  slowPPM1_OFFTime = slowPPM1_Pulselength - fastPPM_ONTime;//OFF time in microseconds

  fastPPM_ONTime = (uint16_t)((((float)angle * (float)fastPPM_MinTime) / (float)360) + (float)fastPPM_MinTime);
  fastPPM_OFFTime = fastPPM_Pulselength - fastPPM_ONTime;//OFF time in microseconds
}


uint16_t motorAngle(int32_t hall_1, int32_t hall_2)
{
  uint16_t angle = 0;
  uint16_t offset = 0;
  
  if ((hall_1 == 0) | (hall_2 == 0))
    {
      if (hall_1 == 0)
      {
        if (hall_2 > 0)
        {
          angle = 0;
        }
        else
        {
          angle = 180;
        }
      }
      else
      {
        if (hall_1 > 0)
        {
          angle = 90;
        }
        else
        {
          angle = 270;
        }
      }
    }
    else
    {
      if (hall_2 < 0)
      {
        offset = 180;
      }
      else
      {
        if (hall_1 > 0)
        {
          offset = 0;
        }
        else
        {
          offset = 360;
        }
      }  
      angle = offset + ((atan((float)hall_1 / (float)hall_2) * 180) / M_PI);
    }  
    return angle;
}
