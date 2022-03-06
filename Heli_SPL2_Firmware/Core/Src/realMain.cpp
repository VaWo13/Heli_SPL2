#include "main.h"
#include "realMain.h"
#include "SBUS.h"
#include "MPU6050.h"
#include "I2Cdev.h"

#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint32_t timestamp = 0;
uint16_t adcValuesArray[2];
uint16_t angle;
uint8_t Step = 0;

void loop()
{
  if (TIM4->CNT < 1000)
  {
    switch (Step)
    {
    case 0:
      HAL_NVIC_DisableIRQ(EXTI0_IRQn);
      break;
    case 4:
      HAL_NVIC_DisableIRQ(EXTI0_IRQn);
      break;
    case 5:
      HAL_NVIC_DisableIRQ(EXTI0_IRQn);
      break;

    default:
      break;
    }
    //get motor angle and update PPM_OnTime
    ADC_Select_Channel_11();
	  adcValuesArray[0] = (uint16_t)ADC1->DR;
	  ADC_Select_Channel_12();
	  adcValuesArray[1] = (uint16_t)ADC1->DR;
    angle = motorAngle(adcValuesArray[1] - 1250, adcValuesArray[0] - 1250);
    TIM4->CCR1 = (uint16_t)(fastPPM_MinTime + 500 + ((float)SBUS_Channels[2] / 2) + ((float)sin((angle + 45) * (M_PI / 180)) * ((float)SBUS_Channels[0] / 10)) + ((float)cos((angle + 45) * (M_PI / 180)) * ((float)SBUS_Channels[1] / 10)));


    switch (Step)
    {
    case 0:   //get quaternions
      
      writeBit(MPU6050_Adresse, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true); //reset FIFO
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      break;
    case 1:
      if (SBUSNewPackage == true)
      {
        SBUS_PostProcessing();
      }
      break;
    case 2:
      for (size_t i = 0; i < 1; i++)
      {
        unsigned char msg[300];
	      sprintf((char*)msg," %ld %ld %ld %ld %hd %hd %hd %hd %hd %hd \r\n"                                                                                                                                                                                                                                                  \
        , (Quaternions[0])                                                                                                                                                                                                                                                                                                  \
        , (Quaternions[1])                                                                                                                                                                                                                                                                                                  \
        , (Quaternions[2])                                                                                                                                                                                                                                                                                                  \
        , (Quaternions[3])                                                                                                                                                                                                                                                                                                  \
        , 2 * (int16_t)(((float)atan((float)Quaternions[3] / (float)Quaternions[2]) * 180) / M_PI)                                                                                                                                                                                                                          \
        , 2 * (int16_t)(((float)atan((float)Quaternions[3] / (float)Quaternions[1]) * 180) / M_PI)                                                                                                                                                                                                                          \
        , 2 * (int16_t)(((float)atan((float)Quaternions[1] / (float)Quaternions[2]) * 180) / M_PI)                                                                                                                                                                                                                          \
        , 2 * (int16_t)(((float)atan((float)Quaternions[0] / (float)Quaternions[1]) * 180) / M_PI)                                                                                                                                                                                                                          \
        , 2 * (int16_t)(((float)acos((float)Quaternions[0] / (float)1073741824) * 180) / M_PI)                                                                                                                                                                                                                              \
        , (int16_t)((float)sqrt((((float)Quaternions[1] / (float)1073741824) * ((float)Quaternions[1] / (float)1073741824)) + (((float)Quaternions[2] / (float)1073741824) * ((float)Quaternions[2] / (float)1073741824)) + (((float)Quaternions[3] / (float)1073741824) * ((float)Quaternions[3] / (float)1073741824))))); \
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

      //for (size_t i = 0; i < 1; i++)
      //{
      //  unsigned char msg[300];
      //  sprintf((char*)msg," %hd %hd %hd %hd %hd %hd %hd %hd %hu %hu \r\n", SBUS_Channels[0], SBUS_Channels[1], SBUS_Channels[2], SBUS_Channels[3], SBUS_Channels[4], SBUS_Channels[5], SBUS_Channels[6], SBUS_Channels[7], SBUS_CorruptedPackage, SBUS_Bytes[0]);
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

      break;
    case 3:
      
      break;
    case 4:
      readBytes(MPU6050_Adresse, MPU6050_RA_FIFO_COUNTH, 2, MPU6050_RX_buf);  //get FIFO count
      FIFOCounter = (((uint16_t)MPU6050_RX_buf[0]) << 8) | MPU6050_RX_buf[1];
      while (FIFOCounter < 42)
      {
        HAL_GPIO_TogglePin(ONBOARD_WRITE_2_GPIO_Port, ONBOARD_WRITE_2_Pin);   //debug Pin
        readBytes(MPU6050_Adresse, MPU6050_RA_FIFO_COUNTH, 2, MPU6050_RX_buf);  //get FIFO count
        FIFOCounter = (((uint16_t)MPU6050_RX_buf[0]) << 8) | MPU6050_RX_buf[1];
      }
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      break;
    case 5:
      if ((FIFOCounter == 42) | (FIFOCounter == 84))
      {
        readBytes(MPU6050_Adresse, MPU6050_RA_FIFO_R_W, 16, MPU6050_RX_buf);      //get FIFO data
        Quaternions[0] = (((uint32_t)MPU6050_RX_buf[0] << 24) |  ((uint32_t)MPU6050_RX_buf[1] << 16) |  ((uint32_t)MPU6050_RX_buf[2] << 8) |  MPU6050_RX_buf[3]);
        Quaternions[1] = (((uint32_t)MPU6050_RX_buf[4] << 24) |  ((uint32_t)MPU6050_RX_buf[5] << 16) |  ((uint32_t)MPU6050_RX_buf[6] << 8) |  MPU6050_RX_buf[7]);
        Quaternions[2] = (((uint32_t)MPU6050_RX_buf[8] << 24) |  ((uint32_t)MPU6050_RX_buf[9] << 16) |  ((uint32_t)MPU6050_RX_buf[10] << 8) | MPU6050_RX_buf[11]);
        Quaternions[3] = (((uint32_t)MPU6050_RX_buf[12] << 24) | ((uint32_t)MPU6050_RX_buf[13] << 16) | ((uint32_t)MPU6050_RX_buf[14] << 8) | MPU6050_RX_buf[15]);
      }
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      break;
    case 9:
      Step = 255;
      break;
    default:
      break;
    }
    Step ++;
    while (TIM4->CNT <= 1000)
    {
    }
  }
  
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

    // for (size_t i = 0; i < 1; i++)
    //{
      // unsigned char msg[300];
      // sprintf((char*)msg,"%hd %hd %hd \r\n", SBUS_Channels[2], angle, (int16_t)(sin((angle * M_PI) / 180) * ((float)SBUS_Channels[0] / 10)));
      // uint8_t x = 0;
      // while (msg[x] != NULL)
      // {
      	// x++;
      // }
      // unsigned char msgTransmit[x];
      // for (size_t i = 0; i < x; i++)
      // {
      	// msgTransmit[i] = msg[i];
      // }
      // CDC_Transmit_FS((unsigned char*)msgTransmit, sizeof(msgTransmit));
    //}


    if (SBUS_Channels[4] < 100)
    {
      //MPU6050_readDMP_Quaterions();
    }

  
  //ADC_Select_Channel_11();
	//adcValuesArray[0] = (uint16_t)ADC1->DR;
	//ADC_Select_Channel_12();
	//adcValuesArray[1] = (uint16_t)ADC1->DR;
//
  //angle = motorAngle(adcValuesArray[1] - 1250, adcValuesArray[0] - 1250);
//
  //// slowPPM1_ONTime = (uint16_t)((((float)angle * (float)slowPPM1_MinTime) / (float)360) + (float)slowPPM1_MinTime);
  //slowPPM1_ONTime = (uint16_t)(slowPPM1_MinTime + ((float)SBUS_Channels[2] / 2));
  //slowPPM1_OFFTime = slowPPM1_Pulselength - fastPPM_ONTime;//OFF time in microseconds
  //TIM3->CCR1 = (uint16_t)(slowPPM1_MinTime + 500 + ((float)SBUS_Channels[3] / 2));
//
  //// fastPPM_ONTime = (uint16_t)((((float)angle * (float)fastPPM_MinTime) / (float)360) + (float)fastPPM_MinTime);
  //fastPPM_ONTime = (uint16_t)(fastPPM_MinTime + ((float)SBUS_Channels[2] / 2));
  //fastPPM_OFFTime = fastPPM_Pulselength - fastPPM_ONTime;//OFF time in microseconds
  //TIM4->CCR1 = (uint16_t)(fastPPM_MinTime + 500 + ((float)SBUS_Channels[2] / 2) + ((float)sin((angle + 0) * (M_PI / 180)) * ((float)SBUS_Channels[0] / -10)) + ((float)cos((angle + 0) * (M_PI / 180)) * ((float)SBUS_Channels[1] / -10)));
  }
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