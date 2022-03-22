#include "realMain.h"
#include "main.h"
#include "motorControl.h"                   //NOTDONE needed?
#include "MPU6050.h"
#include "SBUS.h"
#include "PID.h"

uint8_t task;

void loop()
{
  if (TIM4->CNT < fastPPM_MinTime)
  {
    switch (task)                       //disable the SBUS pin interrupt for the selected tasks
    {
    case 1:
      HAL_NVIC_DisableIRQ(EXTI0_IRQn);
      break;
    case 5:
      HAL_NVIC_DisableIRQ(EXTI0_IRQn);
      break;
    case 6:
      HAL_NVIC_DisableIRQ(EXTI0_IRQn);
      break;
    default:
      break;
    }

    updateMainMotorSpeed();     //NOTDONE check is PPM value can be changed mid cycle or if it waits for the next one on oscilloscope

    switch (task)                       //execute the selected task
    {
    case 1:   //get quaternions
      MPU6050_resetFIFO();
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      break;
    case 2:
      if (SBUSNewPackage == true) SBUS_postProcessing();
      break;
    case 3:
       for (size_t i = 0; i < 1; i++)
       {
         unsigned char msg[300];
	       sprintf((char*)msg,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %hd %hd %f %hd \r\n"                      \
         , PID_Pitch_xw_diff                                                                                        \
         , PID_Roll_xw_diff                                                                                         \
         , PID_Yaw_xw_diff                                                                                          \
         , PID_Pitch_y                                                                                              \
         , PID_Roll_y                                                                                               \
         , PID_Yaw_y                                                                                                \
         , Pitch_PID_k[0] * 100                                                                                     \
         , Pitch_PID_k[1] * 100                                                                                     \
         , Pitch_PID_k[2] * 100                                                                                     \
         , Roll_PID_k[0] * 100                                                                                      \
         , Roll_PID_k[1] * 100                                                                                      \
         , Roll_PID_k[2] * 100                                                                                      \
         , Yaw_PID_k[0] * 100                                                                                       \
         , Yaw_PID_k[1] * 100                                                                                       \
         , Yaw_PID_k[2] * 100                                                                                       \
         , 2 * (int16_t)(((float)atan((float)MPUoutputQuaternion[0] / (float)MPUoutputQuaternion[1]) * 180) / M_PI) \
         , 2 * (int16_t)(((float)acos((float)MPUoutputQuaternion[0] / (float)1073741824) * 180) / M_PI)             \
         , (float)SBUS_Channels[5]                                                                                  \
         , mainMotorAngleOffset);                                                                                  \
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
      break;
    case 4:
      get_XW_diffAngles();
      update_FrameOriginQuaternion();
      break;
    case 5:
      MPU6050_WaitForQuaternionSet();
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      break;
    case 6:
      MPU6050_ConvertToQuaternions();
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      update_PID();
      updateTailMotorSpeed();
      break;
    case 7:
      switchTuningMode();
      if (SBUS_Channels[4] >= 990)  //if the set-PID-switch on the remote is on read the values
      {
        getPIDValues();
        getAngleOffset();
      }
      break;
    case 10:
      task = 0;                 //reset to task 1  (keep in mind task ++; below)
      break;
    default:
      break;
    }
    task ++;

    while (TIM4->CNT <= fastPPM_MinTime);     //wait until fastPPM_MinTime has passed
  }
}