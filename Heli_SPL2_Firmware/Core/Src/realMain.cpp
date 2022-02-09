#include "main.h"
#include "realMain.h"

#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint16_t adcValuesArray[2];

void loop()
{
    ADC_Select_Channel_11();
	adcValuesArray[0] = (uint16_t)ADC1->DR;
	ADC_Select_Channel_12();
	adcValuesArray[1] = (uint16_t)ADC1->DR;

    uint16_t alla = motorAngle(adcValuesArray[1] - 1250, adcValuesArray[0] - 1250);

    unsigned char msg[30];
	sprintf((char*)msg,"%lu %lu %lu %lu\r\n",adcValuesArray[0], adcValuesArray[1], alla, slowPPM1_ONTime);
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

    slowPPM1_ONTime = (uint16_t)((((float)alla * (float)slowPPM1_MinTime) / (float)360) + (float)slowPPM1_MinTime);
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
